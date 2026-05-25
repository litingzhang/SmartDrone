#include "adapters/slam/superpoint/superpoint_native_extractor.h"

#include "adapters/slam/superpoint/superpoint_lightglue_frontend_client.h"
#include "adapters/slam/superpoint/superpoint_runtime_options.h"

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

#include "adapters/slam/engine/slam_env.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_impl.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_postprocess_descriptors.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_postprocess_heatmap.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_common.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_lightglue.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_output.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_superpoint.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_utils.h"

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>

namespace SmartDrone::Adapters::Slam {

SuperPointNativeExtractor::SuperPointNativeExtractor() = default;

using SuperPointTensorRtInternal::AppendStereoFeaturePairs;
using SuperPointTensorRtInternal::DurationMs;
using SuperPointTensorRtInternal::EnvIntClamped;
using SuperPointTensorRtInternal::LIGHT_GLUE_MIN_STEREO_PAIRS_FOR_SUPPLEMENT;
using SuperPointTensorRtInternal::SUPER_POINT_STEREO_EXTRACTION_SLACK;
using SuperPointTensorRtInternal::MatchStereoPairs;
using SuperPointTensorRtInternal::SuperPointPostStats;
using SuperPointTensorRtInternal::TensorRtForwardStats;

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

struct SuperPointNativeExtractor::StereoComputeContext {
    std::chrono::steady_clock::time_point totalStartTp{};
    std::chrono::steady_clock::time_point prepareStartTp{};
    std::chrono::steady_clock::time_point prepareEndTp{};
    std::chrono::steady_clock::time_point inferStartTp{};
    std::chrono::steady_clock::time_point inferEndTp{};
    cv::Mat leftGray8;
    cv::Mat rightGray8;
    std::vector<SuperPointFeatureSet> rawOutputs;
    double inputMs{0.0};
    double forwardMs{0.0};
    double postMs{0.0};
    int requestedMaxPoints{0};
    int lightGluePoints{0};
    int descriptorCandidates{0};
    int extractionBudget{0};
    int descriptorLimit{0};
    int maxStereoPairs{0};
};

struct SuperPointNativeExtractor::StereoMatchState {
    SuperPointFeatureSet descriptorLeftFeatures;
    SuperPointFeatureSet descriptorRightFeatures;
    int descriptorLeftCount{0};
    int descriptorRightCount{0};
    double descriptorMatchMs{0.0};
    int descriptorMatchCalls{0};
    double lightGlueMatchMs{0.0};
    bool usedLightGlue{false};
    bool usedDescriptorPrimary{false};
    bool usedDescriptorSupplement{false};
    bool skippedLightGlue{false};
    int lightGlueEveryN{1};
    int lightGlueFrameIndex{0};
    const char *lightGlueSkipReason{"none"};
    int lightGlueSupplementMinPairs{0};
    int lightGlueLeftCount{0};
    int lightGlueRightCount{0};
};

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
    const std::vector<cv::Mat> grayBatch{gray8};
    if (!m_impl->DetectAndComputeBatch(
            {grayBatch, detectMaxPoints, detectMaxPoints, outputs, &inputMs,
             &forwardMs, &postMs, err}) ||
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

bool SuperPointNativeExtractor::PrepareStereoGrayImages(
    const cv::Mat &leftGray, const cv::Mat &rightGray,
    StereoComputeContext &context, std::string *err)
{
    context.totalStartTp = std::chrono::steady_clock::now();
    context.prepareStartTp = context.totalStartTp;
    if (!PrepareGrayImage(leftGray, context.leftGray8, err) ||
        !PrepareGrayImage(rightGray, context.rightGray8, err)) {
        return false;
    }
    context.prepareEndTp = std::chrono::steady_clock::now();
    return true;
}

void SuperPointNativeExtractor::ConfigureStereoExtractionBudgets(
    StereoComputeContext &context) const
{
    context.requestedMaxPoints = std::max(1, m_maxPoints);
    context.lightGluePoints = m_impl->MaxPointsForLightGlue();
    context.descriptorCandidates =
        EnvIntClamped("SMART_DRONE_DESCRIPTOR_SUPPLEMENT_CANDIDATES",
                      context.requestedMaxPoints, 1, 4096);
    const int requiredFeatureBudget =
        std::max({context.requestedMaxPoints, context.lightGluePoints,
                  context.descriptorCandidates}) +
        SUPER_POINT_STEREO_EXTRACTION_SLACK;
    context.extractionBudget = EnvIntClamped(
        "SMART_DRONE_SUPERPOINT_STEREO_EXTRACTION_BUDGET",
        std::min(std::max(m_topK, context.requestedMaxPoints),
                 requiredFeatureBudget),
        context.requestedMaxPoints, 4096);
    context.descriptorLimit = EnvIntClamped(
        "SMART_DRONE_SUPERPOINT_DESCRIPTOR_LIMIT",
        std::max({context.requestedMaxPoints, context.lightGluePoints,
                  context.descriptorCandidates}),
        1, context.extractionBudget);
    context.maxStereoPairs = std::max(1, m_maxPoints);
}

bool SuperPointNativeExtractor::RunStereoSuperPointBatch(
    StereoComputeContext &context, std::string *err)
{
    ConfigureStereoExtractionBudgets(context);
    context.inferStartTp = context.prepareEndTp;
    const std::vector<cv::Mat> grayBatch{context.leftGray8,
                                         context.rightGray8};
    if (!m_impl->DetectAndComputeBatch(
            {grayBatch, context.extractionBudget, context.descriptorLimit,
             context.rawOutputs, &context.inputMs, &context.forwardMs,
             &context.postMs, err}) ||
        context.rawOutputs.size() != 2) {
        return false;
    }
    return true;
}

void SuperPointNativeExtractor::ConfigureStereoMatchState(
    const StereoComputeContext &context, StereoMatchState &match)
{
    match.lightGlueEveryN =
        m_lightGlueEveryNOverride > 0
            ? std::clamp(m_lightGlueEveryNOverride, 1, 120)
            : EnvIntClamped("SMART_DRONE_LIGHTGLUE_EVERY_N", 4, 1, 120);
    match.lightGlueFrameIndex = m_impl->NextLightGlueFrameIndex();
    match.skippedLightGlue = match.lightGlueEveryN > 1 &&
                             (match.lightGlueFrameIndex %
                              match.lightGlueEveryN) != 0;
    match.lightGlueSkipReason = match.skippedLightGlue ? "cadence" : "none";
    match.lightGlueSupplementMinPairs =
        EnvIntClamped("SMART_DRONE_LIGHTGLUE_SUPPLEMENT_MIN_PAIRS",
                      LIGHT_GLUE_MIN_STEREO_PAIRS_FOR_SUPPLEMENT, 0,
                      context.maxStereoPairs);
    m_impl->ResetLightGlueStats();
    if (!match.skippedLightGlue && m_impl->ConsumeLightGlueCooldown()) {
        match.skippedLightGlue = true;
        match.lightGlueSkipReason = "cooldown";
    }
}

void SuperPointNativeExtractor::BuildDescriptorMatches(
    const StereoComputeContext &context, StereoMatchState &match)
{
    if (!match.descriptorLeftFeatures.keypoints.empty() ||
        !match.descriptorRightFeatures.keypoints.empty()) {
        return;
    }
    const auto startTp = std::chrono::steady_clock::now();
    MatchStereoPairs(context.rawOutputs[0], context.rawOutputs[1],
                     context.maxStereoPairs, match.descriptorLeftFeatures,
                     match.descriptorRightFeatures);
    const auto endTp = std::chrono::steady_clock::now();
    match.descriptorMatchMs += DurationMs(startTp, endTp);
    ++match.descriptorMatchCalls;
    match.descriptorLeftCount =
        static_cast<int>(match.descriptorLeftFeatures.keypoints.size());
    match.descriptorRightCount =
        static_cast<int>(match.descriptorRightFeatures.keypoints.size());
}

void SuperPointNativeExtractor::UseDescriptorPrimary(
    const StereoComputeContext &context, StereoMatchState &match,
    SuperPointFeatureSet &leftFeatures, SuperPointFeatureSet &rightFeatures)
{
    BuildDescriptorMatches(context, match);
    leftFeatures = std::move(match.descriptorLeftFeatures);
    rightFeatures = std::move(match.descriptorRightFeatures);
    match.usedDescriptorPrimary = true;
}

void SuperPointNativeExtractor::AppendDescriptorSupplementIfNeeded(
    const StereoComputeContext &context, StereoMatchState &match,
    SuperPointFeatureSet &leftFeatures, SuperPointFeatureSet &rightFeatures)
{
    if (match.lightGlueSupplementMinPairs <= 0 ||
        static_cast<int>(std::min(leftFeatures.keypoints.size(),
                                  rightFeatures.keypoints.size())) >=
            match.lightGlueSupplementMinPairs) {
        return;
    }
    const size_t beforeCount =
        std::min(leftFeatures.keypoints.size(), rightFeatures.keypoints.size());
    BuildDescriptorMatches(context, match);
    AppendStereoFeaturePairs(leftFeatures, rightFeatures,
                             match.descriptorLeftFeatures,
                             match.descriptorRightFeatures,
                             context.maxStereoPairs);
    match.usedDescriptorSupplement =
        std::min(leftFeatures.keypoints.size(), rightFeatures.keypoints.size()) >
        beforeCount;
}

bool SuperPointNativeExtractor::TryUseLightGlue(
    const StereoComputeContext &context, StereoMatchState &match,
    SuperPointFeatureSet &leftFeatures, SuperPointFeatureSet &rightFeatures,
    std::string *err)
{
    if (match.skippedLightGlue) {
        return false;
    }
    SuperPointFeatureSet lightGlueLeftFeatures;
    SuperPointFeatureSet lightGlueRightFeatures;
    if (!m_impl->MatchWithLightGlue(
            {context.rawOutputs[0], context.rawOutputs[1],
             context.maxStereoPairs, context.leftGray8.cols,
             context.leftGray8.rows, lightGlueLeftFeatures,
             lightGlueRightFeatures, &match.lightGlueMatchMs, err})) {
        return false;
    }

    match.usedLightGlue = true;
    if (lightGlueLeftFeatures.keypoints.empty() ||
        lightGlueRightFeatures.keypoints.empty()) {
        m_impl->RecordLightGlueEmptyOutput();
        return false;
    }
    match.lightGlueLeftCount =
        static_cast<int>(lightGlueLeftFeatures.keypoints.size());
    match.lightGlueRightCount =
        static_cast<int>(lightGlueRightFeatures.keypoints.size());
    leftFeatures = std::move(lightGlueLeftFeatures);
    rightFeatures = std::move(lightGlueRightFeatures);
    m_impl->ResetLightGlueEmptyCount();
    m_impl->RecordLightGlueLowYield(match.lightGlueLeftCount);
    AppendDescriptorSupplementIfNeeded(context, match, leftFeatures,
                                       rightFeatures);
    return true;
}

void SuperPointNativeExtractor::UpdateStereoStats(
    const StereoComputeContext &context, const StereoMatchState &match,
    const SuperPointFeatureSet &leftFeatures,
    const SuperPointFeatureSet &rightFeatures)
{
    m_lastStats.prepareMs =
        DurationMs(context.prepareStartTp, context.prepareEndTp);
    m_lastStats.inputMs = context.inputMs;
    m_lastStats.forwardMs = context.forwardMs;
    m_lastStats.postMs = context.postMs;
    m_lastStats.inferMs = DurationMs(context.inferStartTp, context.inferEndTp);
    m_lastStats.totalMs =
        DurationMs(context.totalStartTp, context.inferEndTp);
    m_lastStats.rawLeftCount =
        static_cast<int>(context.rawOutputs[0].keypoints.size());
    m_lastStats.rawRightCount =
        static_cast<int>(context.rawOutputs[1].keypoints.size());
    m_lastStats.stereoLeftCount = static_cast<int>(leftFeatures.keypoints.size());
    m_lastStats.stereoRightCount =
        static_cast<int>(rightFeatures.keypoints.size());
    m_lastStats.lightGlueUsed = match.usedLightGlue;
    m_lastStats.descriptorFallbackUsed =
        match.usedDescriptorPrimary || match.usedDescriptorSupplement;
    m_lastStats.imageCount = 2;
    m_lastStats.payloadBytes =
        static_cast<uint32_t>(context.leftGray8.total() +
                              context.rightGray8.total());
}

void SuperPointNativeExtractor::LogSuperPointPerformanceFields(
    std::ostream &out) const
{
    const SuperPointDiagnostics diagnostics =
        m_impl->CurrentSuperPointDiagnostics();
    const TensorRtForwardStats &spStats = diagnostics.forwardStats;
    const SuperPointPostStats &spPostStats = diagnostics.postStats;
    out << " sp_heatmap_ms=" << spPostStats.heatmapMs
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
              << (diagnostics.batchedForward ? "Y" : "N");
}

void SuperPointNativeExtractor::LogLightGluePerformanceFields(
    std::ostream &out, const StereoMatchState &match) const
{
    const LightGlueDiagnostics diagnostics =
        m_impl->CurrentLightGlueDiagnostics();
    const TensorRtForwardStats &lgStats = diagnostics.forwardStats;
    out << " lightglue=" << (match.usedLightGlue ? "Y" : "N")
              << " skipped_lightglue="
              << (match.skippedLightGlue ? "Y" : "N")
              << " lg_every_n=" << match.lightGlueEveryN
              << " lg_frame_index=" << match.lightGlueFrameIndex
              << " lg_skip_reason=" << match.lightGlueSkipReason
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
              << " lg_requested_pts=" << diagnostics.requestedPointCount
              << " lg_input_pts=" << diagnostics.inputPointCount
              << " lg_static_shape_fallback="
              << (diagnostics.staticShapeFallback ? "Y" : "N");
}

void SuperPointNativeExtractor::LogDescriptorPerformanceFields(
    std::ostream &out, const StereoComputeContext &context,
    const StereoMatchState &match) const
{
    const LightGlueDiagnostics diagnostics =
        m_impl->CurrentLightGlueDiagnostics();
    out << " sp_descriptor="
              << (match.usedDescriptorPrimary
                      ? "primary"
                      : (match.usedDescriptorSupplement ? "supplement" : "none"))
              << " descriptor_match_ms=" << match.descriptorMatchMs
              << " descriptor_match_calls=" << match.descriptorMatchCalls
              << " descriptor_candidates=" << context.descriptorCandidates
              << " extraction_budget=" << context.extractionBudget
              << " descriptor_limit=" << context.descriptorLimit
              << " lg_score_range=" << diagnostics.minScore << "/"
              << diagnostics.maxScore << " lg_score_space="
              << (diagnostics.scoresLookLog ? "log" : "prob")
              << " lg_decode_ms=" << diagnostics.decodeMs
              << " lg_orientation=" << diagnostics.orientation
              << " lg_filter=" << diagnostics.mutualCount << "/"
              << diagnostics.scorePassCount << "/"
              << diagnostics.geometryPassCount << "/"
              << diagnostics.acceptedCount;
}

void SuperPointNativeExtractor::LogStereoPointSummaryFields(
    std::ostream &out, const StereoComputeContext &context,
    const StereoMatchState &match, const SuperPointFeatureSet &leftFeatures,
    const SuperPointFeatureSet &rightFeatures) const
{
    out
              << " supplement_min=" << match.lightGlueSupplementMinPairs
              << " lightglue_ms=" << match.lightGlueMatchMs
              << " total_ms=" << m_lastStats.totalMs
              << " raw_pts=" << context.rawOutputs[0].keypoints.size() << "/"
              << context.rawOutputs[1].keypoints.size()
              << " lg_pts=" << match.lightGlueLeftCount << "/"
              << match.lightGlueRightCount
              << " desc_pts=" << match.descriptorLeftCount << "/"
              << match.descriptorRightCount
              << " stereo_pts=" << leftFeatures.keypoints.size() << "/"
              << rightFeatures.keypoints.size();
}

void SuperPointNativeExtractor::LogStereoPerformance(
    const StereoComputeContext &context, const StereoMatchState &match,
    const SuperPointFeatureSet &leftFeatures,
    const SuperPointFeatureSet &rightFeatures) const
{
    std::cerr << "[superpoint_trt_perf] batch=2 input_ms=" << context.inputMs
              << " gpu_forward_ms=" << context.forwardMs
              << " cpu_post_ms=" << context.postMs;
    LogSuperPointPerformanceFields(std::cerr);
    LogLightGluePerformanceFields(std::cerr, match);
    LogDescriptorPerformanceFields(std::cerr, context, match);
    LogStereoPointSummaryFields(std::cerr, context, match, leftFeatures,
                                rightFeatures);
    std::cerr << "\n";
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

    StereoComputeContext context;
    if (!PrepareStereoGrayImages(leftGray, rightGray, context, err) ||
        !RunStereoSuperPointBatch(context, err)) {
        return false;
    }

    StereoMatchState match;
    ConfigureStereoMatchState(context, match);
    if (!TryUseLightGlue(context, match, leftFeatures, rightFeatures, err)) {
        UseDescriptorPrimary(context, match, leftFeatures, rightFeatures);
    }
    context.inferEndTp = std::chrono::steady_clock::now();
    UpdateStereoStats(context, match, leftFeatures, rightFeatures);
    LogStereoPerformance(context, match, leftFeatures, rightFeatures);
    return true;
}

} // namespace SmartDrone::Adapters::Slam
