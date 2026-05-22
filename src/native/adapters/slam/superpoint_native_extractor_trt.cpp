#include "adapters/slam/superpoint_native_extractor.h"

#include "adapters/slam/superpoint_lightglue_frontend_client.h"
#include "adapters/slam/superpoint_runtime_options.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "adapters/slam/slam_env.h"

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>

namespace SmartDrone::Adapters::Slam {

SuperPointNativeExtractor::SuperPointNativeExtractor() = default;

namespace {
#include "superpoint_native_extractor_common.inc"
#include "superpoint_native_extractor_postprocess.inc"
#include "superpoint_native_extractor_tensorrt_output.inc"
#include "superpoint_native_extractor_stereo_append.inc"
#include "superpoint_native_extractor_tensorrt_engines.inc"

} // namespace

#include "superpoint_native_extractor_impl.inc"

SuperPointNativeExtractor::~SuperPointNativeExtractor() = default;

bool SuperPointNativeExtractor::PrepareGrayImage(const cv::Mat &gray,
                                                 cv::Mat &gray8,
                                                 std::string *err)
{
    if (gray.type() == CV_8UC1 && gray.isContinuous()) {
        gray8 = gray;
        return true;
    }
    if (gray.channels() == 1) {
        gray.convertTo(gray8, CV_8UC1);
    } else {
        cv::cvtColor(gray, gray8, cv::COLOR_BGR2GRAY);
    }
    if (!gray8.isContinuous()) {
        gray8 = gray8.clone();
    }
    if (gray8.empty()) {
        if (err != nullptr) {
            *err = "SuperPoint native gray image preparation failed";
        }
        return false;
    }
    return true;
}

bool SuperPointNativeExtractor::Start(const std::string &repoPath,
                                      const std::string &device, int topK,
                                      int maxPoints, std::string *err)
{
    m_topK = topK;
    m_maxPoints = maxPoints;
    m_lastStats = Stats{};
    m_impl = std::make_unique<Impl>();
    if (!m_impl->Load(repoPath, device, err)) {
        m_impl.reset();
        m_running = false;
        return false;
    }
    m_running = true;
    return true;
}

void SuperPointNativeExtractor::Stop()
{
    m_running = false;
    m_lastStats = Stats{};
    m_impl.reset();
}

bool SuperPointNativeExtractor::Running() const
{
    return m_running;
}

SuperPointNativeExtractor::Stats SuperPointNativeExtractor::LastStats() const
{
    return m_lastStats;
}

void SuperPointNativeExtractor::SetLightGlueEveryNOverride(int everyN)
{
    m_lightGlueEveryNOverride = everyN > 0 ? std::clamp(everyN, 1, 120) : 0;
}

bool SuperPointNativeExtractor::Detect(const cv::Mat &gray,
                                       std::vector<cv::Point2f> &outPoints,
                                       std::string *err)
{
    SuperPointFeatureSet features;
    if (!DetectAndCompute(gray, features, err)) {
        return false;
    }
    outPoints = std::move(features.keypoints);
    return true;
}

bool SuperPointNativeExtractor::DetectAndCompute(
    const cv::Mat &gray, SuperPointFeatureSet &outFeatures, std::string *err)
{
    outFeatures = SuperPointFeatureSet{};
    m_lastStats = Stats{};
    if (!m_running || !m_impl) {
        if (err != nullptr) {
            *err = "SuperPoint TensorRT backend not running";
        }
        return false;
    }
    const auto totalStartTp = std::chrono::steady_clock::now();
    cv::Mat gray8;
    const auto prepareStartTp = totalStartTp;
    if (!PrepareGrayImage(gray, gray8, err)) {
        return false;
    }
    const auto prepareEndTp = std::chrono::steady_clock::now();
    std::vector<SuperPointFeatureSet> outputs;
    const auto inferStartTp = prepareEndTp;
    double inputMs = 0.0;
    double forwardMs = 0.0;
    double postMs = 0.0;
    const int detectMaxPoints = std::max(m_topK, m_maxPoints);
    if (!m_impl->DetectAndComputeBatch({gray8}, detectMaxPoints, detectMaxPoints,
                                       outputs, &inputMs, &forwardMs, &postMs,
                                       err) ||
        outputs.empty()) {
        return false;
    }
    const auto inferEndTp = std::chrono::steady_clock::now();
    outFeatures = std::move(outputs.front());
    m_lastStats.prepareMs = DurationMs(prepareStartTp, prepareEndTp);
    m_lastStats.inputMs = inputMs;
    m_lastStats.forwardMs = forwardMs;
    m_lastStats.postMs = postMs;
    m_lastStats.inferMs = DurationMs(inferStartTp, inferEndTp);
    m_lastStats.totalMs = DurationMs(totalStartTp, inferEndTp);
    m_lastStats.imageCount = 1;
    m_lastStats.payloadBytes = static_cast<uint32_t>(gray8.total());
    return true;
}

bool SuperPointNativeExtractor::DetectAndComputeStereo(
    const cv::Mat &leftGray, const cv::Mat &rightGray,
    SuperPointFeatureSet &leftFeatures, SuperPointFeatureSet &rightFeatures,
    std::string *err)
{
    leftFeatures = SuperPointFeatureSet{};
    rightFeatures = SuperPointFeatureSet{};
    m_lastStats = Stats{};
    if (!m_running || !m_impl) {
        if (err != nullptr) {
            *err = "SuperPoint TensorRT backend not running";
        }
        return false;
    }
    const auto totalStartTp = std::chrono::steady_clock::now();
    cv::Mat leftGray8;
    cv::Mat rightGray8;
    const auto prepareStartTp = totalStartTp;
    if (!PrepareGrayImage(leftGray, leftGray8, err) ||
        !PrepareGrayImage(rightGray, rightGray8, err)) {
        return false;
    }
    const auto prepareEndTp = std::chrono::steady_clock::now();
    std::vector<SuperPointFeatureSet> rawOutputs;
    const auto inferStartTp = prepareEndTp;
    double inputMs = 0.0;
    double forwardMs = 0.0;
    double postMs = 0.0;
    const int requestedMaxPoints = std::max(1, m_maxPoints);
    const int lightGluePoints = m_impl->maxPointsForLightGlue();
    const int descriptorCandidates =
        EnvIntClamped("SMART_DRONE_DESCRIPTOR_SUPPLEMENT_CANDIDATES",
                      requestedMaxPoints, 1, 4096);
    const int requiredFeatureBudget =
        std::max({requestedMaxPoints, lightGluePoints, descriptorCandidates}) +
        kSuperPointStereoExtractionSlack;
    const int extractionBudget = EnvIntClamped(
        "SMART_DRONE_SUPERPOINT_STEREO_EXTRACTION_BUDGET",
        std::min(std::max(m_topK, requestedMaxPoints), requiredFeatureBudget),
        requestedMaxPoints, 4096);
    const int descriptorLimit = EnvIntClamped(
        "SMART_DRONE_SUPERPOINT_DESCRIPTOR_LIMIT",
        std::max({requestedMaxPoints, lightGluePoints, descriptorCandidates}), 1,
        extractionBudget);
    if (!m_impl->DetectAndComputeBatch({leftGray8, rightGray8}, extractionBudget,
                                       descriptorLimit, rawOutputs, &inputMs,
                                       &forwardMs, &postMs, err) ||
        rawOutputs.size() != 2) {
        return false;
    }
    double lightGlueMatchMs = 0.0;
    bool usedLightGlue = false;
    bool usedDescriptorPrimary = false;
    bool usedDescriptorSupplement = false;
    bool skippedLightGlue = false;
    SuperPointFeatureSet lightGlueLeftFeatures;
    SuperPointFeatureSet lightGlueRightFeatures;
    SuperPointFeatureSet descriptorLeftFeatures;
    SuperPointFeatureSet descriptorRightFeatures;
    int lightGlueLeftCount = 0;
    int lightGlueRightCount = 0;
    int descriptorLeftCount = 0;
    int descriptorRightCount = 0;
    double descriptorMatchMs = 0.0;
    int descriptorMatchCalls = 0;
    const int maxStereoPairs = std::max(1, m_maxPoints);
    const int lightGlueEveryN =
        m_lightGlueEveryNOverride > 0
            ? std::clamp(m_lightGlueEveryNOverride, 1, 120)
            : EnvIntClamped("SMART_DRONE_LIGHTGLUE_EVERY_N", 4, 1, 120);
    const int lightGlueFrameIndex = m_impl->lightGlueFrameCounter++;
    const bool lightGlueCadenceSkip =
        lightGlueEveryN > 1 && (lightGlueFrameIndex % lightGlueEveryN) != 0;
    const char *lightGlueSkipReason = "none";
    const int lightGlueSupplementMinPairs =
        EnvIntClamped("SMART_DRONE_LIGHTGLUE_SUPPLEMENT_MIN_PAIRS",
                      kLightGlueMinStereoPairsForSupplement, 0, maxStereoPairs);
    auto buildDescriptorMatches = [&]() {
        if (!descriptorLeftFeatures.keypoints.empty() ||
            !descriptorRightFeatures.keypoints.empty()) {
            return;
        }
        const auto descriptorMatchStartTp = std::chrono::steady_clock::now();
        MatchStereoPairs(rawOutputs[0], rawOutputs[1], maxStereoPairs,
                         descriptorLeftFeatures, descriptorRightFeatures);
        const auto descriptorMatchEndTp = std::chrono::steady_clock::now();
        descriptorMatchMs +=
            DurationMs(descriptorMatchStartTp, descriptorMatchEndTp);
        ++descriptorMatchCalls;
        descriptorLeftCount =
            static_cast<int>(descriptorLeftFeatures.keypoints.size());
        descriptorRightCount =
            static_cast<int>(descriptorRightFeatures.keypoints.size());
    };
    m_impl->lastLightGlueMutualCount = 0;
    m_impl->lastLightGlueScorePassCount = 0;
    m_impl->lastLightGlueGeometryPassCount = 0;
    m_impl->lastLightGlueAcceptedCount = 0;
    m_impl->lastLightGlueMinScore = 0.0f;
    m_impl->lastLightGlueMaxScore = 0.0f;
    m_impl->lastLightGlueDecodeMs = 0.0;
    m_impl->lastLightGlueOrientation = "none";
    m_impl->lastLightGlueScoresLookLog = false;
    m_impl->lastLightGlueForwardStats = TensorRtForwardStats{};
    m_impl->lastLightGlueRequestedPointCount = 0;
    m_impl->lastLightGlueInputPointCount = 0;
    m_impl->lastLightGlueStaticShapeFallback = false;
    if (lightGlueCadenceSkip) {
        skippedLightGlue = true;
        lightGlueSkipReason = "cadence";
    } else if (m_impl->lightGlueSkipRemaining > 0) {
        --m_impl->lightGlueSkipRemaining;
        skippedLightGlue = true;
        lightGlueSkipReason = "cooldown";
    }
    if (!skippedLightGlue &&
        m_impl->MatchWithLightGlue(rawOutputs[0], rawOutputs[1], maxStereoPairs,
                                   leftGray8.cols, leftGray8.rows,
                                   lightGlueLeftFeatures, lightGlueRightFeatures,
                                   &lightGlueMatchMs, err)) {
        usedLightGlue = true;
        if (!lightGlueLeftFeatures.keypoints.empty() &&
            !lightGlueRightFeatures.keypoints.empty()) {
            lightGlueLeftCount =
                static_cast<int>(lightGlueLeftFeatures.keypoints.size());
            lightGlueRightCount =
                static_cast<int>(lightGlueRightFeatures.keypoints.size());
            leftFeatures = std::move(lightGlueLeftFeatures);
            rightFeatures = std::move(lightGlueRightFeatures);
            m_impl->lightGlueEmptyCount = 0;
            if (lightGlueLeftCount < m_impl->lightGlueLowYieldMinPairs) {
                ++m_impl->lightGlueLowYieldCount;
                if (m_impl->lightGlueLowYieldCount >=
                    m_impl->lightGlueLowYieldDisableThreshold) {
                    m_impl->lightGlueSkipRemaining = m_impl->lightGlueEmptyCooldownFrames;
                    m_impl->lightGlueLowYieldCount = 0;
                    std::cerr << "[lightglue_trt] low_yield_cooldown pairs="
                              << lightGlueLeftCount
                              << " frames=" << m_impl->lightGlueSkipRemaining << "\n";
                }
            } else {
                m_impl->lightGlueLowYieldCount = 0;
            }
            if (lightGlueSupplementMinPairs > 0 &&
                static_cast<int>(std::min(leftFeatures.keypoints.size(),
                                          rightFeatures.keypoints.size())) <
                    lightGlueSupplementMinPairs) {
                const size_t beforeCount = std::min(leftFeatures.keypoints.size(),
                                                    rightFeatures.keypoints.size());
                buildDescriptorMatches();
                AppendStereoFeaturePairs(leftFeatures, rightFeatures,
                                         descriptorLeftFeatures,
                                         descriptorRightFeatures, maxStereoPairs);
                usedDescriptorSupplement =
                    std::min(leftFeatures.keypoints.size(),
                             rightFeatures.keypoints.size()) > beforeCount;
            }
        } else {
            ++m_impl->lightGlueEmptyCount;
            ++m_impl->lightGlueLowYieldCount;
            if (m_impl->lightGlueEmptyCount >=
                m_impl->lightGlueEmptyDisableThreshold) {
                m_impl->lightGlueSkipRemaining = m_impl->lightGlueEmptyCooldownFrames;
                m_impl->lightGlueEmptyCount = 0;
                m_impl->lightGlueLowYieldCount = 0;
                std::cerr << "[lightglue_trt] empty_output_cooldown frames="
                          << m_impl->lightGlueSkipRemaining << "\n";
            }
            buildDescriptorMatches();
            leftFeatures = std::move(descriptorLeftFeatures);
            rightFeatures = std::move(descriptorRightFeatures);
            usedDescriptorPrimary = true;
        }
    } else {
        buildDescriptorMatches();
        leftFeatures = std::move(descriptorLeftFeatures);
        rightFeatures = std::move(descriptorRightFeatures);
        usedDescriptorPrimary = true;
    }
    const auto inferEndTp = std::chrono::steady_clock::now();
    m_lastStats.prepareMs = DurationMs(prepareStartTp, prepareEndTp);
    m_lastStats.inputMs = inputMs;
    m_lastStats.forwardMs = forwardMs;
    m_lastStats.postMs = postMs;
    m_lastStats.inferMs = DurationMs(inferStartTp, inferEndTp);
    m_lastStats.totalMs = DurationMs(totalStartTp, inferEndTp);
    m_lastStats.rawLeftCount = static_cast<int>(rawOutputs[0].keypoints.size());
    m_lastStats.rawRightCount = static_cast<int>(rawOutputs[1].keypoints.size());
    m_lastStats.stereoLeftCount = static_cast<int>(leftFeatures.keypoints.size());
    m_lastStats.stereoRightCount =
        static_cast<int>(rightFeatures.keypoints.size());
    m_lastStats.lightGlueUsed = usedLightGlue;
    m_lastStats.descriptorFallbackUsed =
        usedDescriptorPrimary || usedDescriptorSupplement;
    m_lastStats.imageCount = 2;
    m_lastStats.payloadBytes =
        static_cast<uint32_t>(leftGray8.total() + rightGray8.total());
    const TensorRtForwardStats &spStats = m_impl->lastSuperPointForwardStats;
    const TensorRtForwardStats &lgStats = m_impl->lastLightGlueForwardStats;
    const SuperPointPostStats &spPostStats = m_impl->lastSuperPointPostStats;
    std::cerr << "[superpoint_trt_perf] batch=2 input_ms=" << inputMs
              << " gpu_forward_ms=" << forwardMs << " cpu_post_ms=" << postMs
              << " sp_heatmap_ms=" << spPostStats.heatmapMs
              << " sp_nms_ms=" << spPostStats.nmsMs
              << " sp_scan_ms=" << spPostStats.scanMs
              << " sp_sort_ms=" << spPostStats.sortMs
              << " sp_desc_sample_ms=" << spPostStats.descriptorMs
              << " sp_candidates=" << spPostStats.candidateCount
              << " sp_selected=" << spPostStats.selectedCount
              << " sp_descriptor_rows=" << spPostStats.descriptorCount
              << " sp_h2d_ms=" << spStats.h2dMs
              << " sp_enqueue_ms=" << spStats.enqueueMs
              << " sp_d2h_ms=" << spStats.outputMs
              << " sp_convert_ms=" << spStats.outputConvertMs
              << " sp_sync_ms=" << spStats.syncMs
              << " sp_gpu_compute_ms=" << spStats.gpuComputeMs
              << " sp_gpu_d2h_ms=" << spStats.gpuOutputMs
              << " sp_event_timing=" << (spStats.eventTimingEnabled ? "Y" : "N")
              << " sp_pinned_host=" << (spStats.pinnedHostOutput ? "Y" : "N")
              << " sp_h2d_bytes=" << spStats.h2dBytes
              << " sp_d2h_bytes=" << spStats.d2hBytes << " sp_batched="
              << (m_impl->lastSuperPointBatchedForward ? "Y" : "N")
              << " lightglue=" << (usedLightGlue ? "Y" : "N")
              << " skipped_lightglue=" << (skippedLightGlue ? "Y" : "N")
              << " lg_every_n=" << lightGlueEveryN
              << " lg_frame_index=" << lightGlueFrameIndex
              << " lg_skip_reason=" << lightGlueSkipReason
              << " lg_h2d_ms=" << lgStats.h2dMs
              << " lg_enqueue_ms=" << lgStats.enqueueMs
              << " lg_d2h_ms=" << lgStats.outputMs
              << " lg_convert_ms=" << lgStats.outputConvertMs
              << " lg_sync_ms=" << lgStats.syncMs
              << " lg_gpu_compute_ms=" << lgStats.gpuComputeMs
              << " lg_gpu_d2h_ms=" << lgStats.gpuOutputMs
              << " lg_event_timing=" << (lgStats.eventTimingEnabled ? "Y" : "N")
              << " lg_pinned_host=" << (lgStats.pinnedHostOutput ? "Y" : "N")
              << " lg_h2d_bytes=" << lgStats.h2dBytes
              << " lg_d2h_bytes=" << lgStats.d2hBytes
              << " lg_requested_pts=" << m_impl->lastLightGlueRequestedPointCount
              << " lg_input_pts=" << m_impl->lastLightGlueInputPointCount
              << " lg_static_shape_fallback="
              << (m_impl->lastLightGlueStaticShapeFallback ? "Y" : "N")
              << " sp_descriptor="
              << (usedDescriptorPrimary
                      ? "primary"
                      : (usedDescriptorSupplement ? "supplement" : "none"))
              << " descriptor_match_ms=" << descriptorMatchMs
              << " descriptor_match_calls=" << descriptorMatchCalls
              << " descriptor_candidates=" << descriptorCandidates
              << " extraction_budget=" << extractionBudget
              << " descriptor_limit=" << descriptorLimit
              << " lg_score_range=" << m_impl->lastLightGlueMinScore << "/"
              << m_impl->lastLightGlueMaxScore << " lg_score_space="
              << (m_impl->lastLightGlueScoresLookLog ? "log" : "prob")
              << " lg_decode_ms=" << m_impl->lastLightGlueDecodeMs
              << " lg_orientation=" << m_impl->lastLightGlueOrientation
              << " lg_filter=" << m_impl->lastLightGlueMutualCount << "/"
              << m_impl->lastLightGlueScorePassCount << "/"
              << m_impl->lastLightGlueGeometryPassCount << "/"
              << m_impl->lastLightGlueAcceptedCount
              << " supplement_min=" << lightGlueSupplementMinPairs
              << " lightglue_ms=" << lightGlueMatchMs
              << " total_ms=" << m_lastStats.totalMs
              << " raw_pts=" << rawOutputs[0].keypoints.size() << "/"
              << rawOutputs[1].keypoints.size()
              << " lg_pts=" << lightGlueLeftCount << "/" << lightGlueRightCount
              << " desc_pts=" << descriptorLeftCount << "/"
              << descriptorRightCount
              << " stereo_pts=" << leftFeatures.keypoints.size() << "/"
              << rightFeatures.keypoints.size() << "\n";
    return true;
}

} // namespace SmartDrone::Adapters::Slam
