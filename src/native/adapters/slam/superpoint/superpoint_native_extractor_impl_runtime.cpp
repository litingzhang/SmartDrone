#include "adapters/slam/superpoint/superpoint_native_extractor_impl.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace SmartDrone::Adapters::Slam {

void SuperPointNativeExtractor::Impl::RecordLightGlueLowYield(int lightGlueLeftCount)
{
    if (lightGlueLeftCount >= lightGlueLowYieldMinPairs) {
        lightGlueLowYieldCount = 0;
        return;
    }
    ++lightGlueLowYieldCount;
    if (lightGlueLowYieldCount < lightGlueLowYieldDisableThreshold) {
        return;
    }
    lightGlueSkipRemaining = lightGlueEmptyCooldownFrames;
    lightGlueLowYieldCount = 0;
    std::cerr << "[lightglue_trt] low_yield_cooldown pairs="
              << lightGlueLeftCount << " frames="
              << lightGlueSkipRemaining << "\n";
}

void SuperPointNativeExtractor::Impl::RecordLightGlueEmptyOutput()
{
    ++lightGlueEmptyCount;
    ++lightGlueLowYieldCount;
    if (lightGlueEmptyCount < lightGlueEmptyDisableThreshold) {
        return;
    }
    lightGlueSkipRemaining = lightGlueEmptyCooldownFrames;
    lightGlueEmptyCount = 0;
    lightGlueLowYieldCount = 0;
    std::cerr << "[lightglue_trt] empty_output_cooldown frames="
              << lightGlueSkipRemaining << "\n";
}

void SuperPointNativeExtractor::Impl::ResetLightGlueEmptyCount()
{
    lightGlueEmptyCount = 0;
}

int SuperPointNativeExtractor::Impl::NextLightGlueFrameIndex()
{
    return lightGlueFrameCounter++;
}

bool SuperPointNativeExtractor::Impl::ConsumeLightGlueCooldown()
{
    if (lightGlueSkipRemaining <= 0) {
        return false;
    }
    --lightGlueSkipRemaining;
    return true;
}

bool SuperPointTensorRtDeviceSupported(const std::string &deviceText,
                                       std::string *err)
{
    const std::string device = LowerCopy(deviceText);
    if (device.empty() || device == "auto" || device == "cuda") {
        return true;
    }
    if (err != nullptr) {
        *err = "native TensorRT SuperPoint only supports device=auto|cuda";
    }
    return false;
}

bool ResolveSuperPointTensorRtEngine(
    const std::string &repoPath,
    const SuperPointTensorRtRuntimeOptions &runtimeOptions,
    std::filesystem::path &enginePath, std::string *err)
{
    enginePath = ResolveSuperPointEnginePath(
        repoPath, runtimeOptions.inputMaxWidth, runtimeOptions.inputMaxHeight);
    if (!enginePath.empty()) {
        return true;
    }
    if (err != nullptr) {
        *err = "SuperPoint TensorRT engine not found under repo: " + repoPath;
    }
    return false;
}

bool SuperPointNativeExtractor::Impl::LoadSuperPointEngine(
    const std::filesystem::path &enginePath,
    const SuperPointTensorRtRuntimeOptions &runtimeOptions, std::string *err)
{
    auto candidate = std::make_unique<TensorRtSuperPointEngine>();
    if (!candidate->Load(enginePath, err)) {
        return false;
    }
    if (!candidate->PreferredInputSize(inputHeight, inputWidth)) {
        inputHeight = runtimeOptions.inputMaxHeight;
        inputWidth = runtimeOptions.inputMaxWidth;
    }
    trtEngine = std::move(candidate);
    std::cerr << "[superpoint_trt] loaded engine=" << enginePath.string()
              << " input=" << inputWidth << "x" << inputHeight << "\n";
    return true;
}

void SuperPointNativeExtractor::Impl::ApplyLightGlueRuntimeOptions(
    int fixedPointCount, const SuperPointTensorRtRuntimeOptions &runtimeOptions)
{
    lightGluePointCount =
        fixedPointCount > 0 ? fixedPointCount : runtimeOptions.lightGluePoints;
    lightGlueDynamicPointCountDisabled = fixedPointCount > 0;
    lightGlueMinScore = runtimeOptions.lightGlueMinScore;
    lightGlueMaxYDiffPx = runtimeOptions.lightGlueMaxYDiffPx;
    lightGlueMinDisparityPx = runtimeOptions.lightGlueMinDisparityPx;
    lightGlueEmptyDisableThreshold =
        runtimeOptions.lightGlueEmptyDisableThreshold;
    lightGlueLowYieldDisableThreshold =
        runtimeOptions.lightGlueLowYieldDisableThreshold;
    lightGlueLowYieldMinPairs = runtimeOptions.lightGlueLowYieldMinPairs;
    lightGlueEmptyCooldownFrames = runtimeOptions.lightGlueEmptyCooldownFrames;
}

void SuperPointNativeExtractor::Impl::LogLightGlueLoaded(
    const std::filesystem::path &lightGluePath, int fixedPointCount) const
{
    std::cerr << "[lightglue_trt] loaded engine=" << lightGluePath.string()
              << " points=" << lightGluePointCount
              << " fixed_points=" << (fixedPointCount > 0 ? "Y" : "N")
              << " min_score=" << lightGlueMinScore
              << " max_y_diff_px=" << lightGlueMaxYDiffPx
              << " min_disparity_px=" << lightGlueMinDisparityPx
              << " empty_disable_threshold=" << lightGlueEmptyDisableThreshold
              << " low_yield_disable_threshold="
              << lightGlueLowYieldDisableThreshold
              << " low_yield_min_pairs=" << lightGlueLowYieldMinPairs
              << " empty_cooldown_frames=" << lightGlueEmptyCooldownFrames
              << "\n";
}

void SuperPointNativeExtractor::Impl::TryLoadLightGlueEngine(
    const std::string &repoPath,
    const SuperPointTensorRtRuntimeOptions &runtimeOptions)
{
    const std::filesystem::path lightGluePath =
        ResolveLightGlueEnginePath(repoPath, runtimeOptions.lightGluePoints);
    if (lightGluePath.empty()) {
        std::cerr << "[lightglue_trt] engine not found; sp_descriptor=primary\n";
        return;
    }
    auto matcher = std::make_unique<TensorRtLightGlueEngine>();
    std::string lgErr;
    if (!matcher->Load(lightGluePath, &lgErr)) {
        std::cerr << "[lightglue_trt] warning: failed to load engine="
                  << lightGluePath.string() << " err=" << lgErr
                  << "; sp_descriptor=primary\n";
        return;
    }
    const int fixedPointCount = matcher->FixedPointCount();
    lightGlueEngine = std::move(matcher);
    ApplyLightGlueRuntimeOptions(fixedPointCount, runtimeOptions);
    LogLightGlueLoaded(lightGluePath, fixedPointCount);
}

bool SuperPointNativeExtractor::Impl::Load(const std::string &repoPath, const std::string &deviceText,
          std::string *err)
{
    if (!SuperPointTensorRtDeviceSupported(deviceText, err)) {
        return false;
    }
    const SuperPointTensorRtRuntimeOptions runtimeOptions =
        LoadSuperPointTensorRtRuntimeOptions();
    std::filesystem::path enginePath;
    if (!ResolveSuperPointTensorRtEngine(repoPath, runtimeOptions, enginePath,
                                         err)) {
        return false;
    }
    if (!LoadSuperPointEngine(enginePath, runtimeOptions, err)) {
        return false;
    }
    TryLoadLightGlueEngine(repoPath, runtimeOptions);
    return true;
}

int SuperPointNativeExtractor::Impl::MaxPointsForLightGlue() const
{
    return LoadSuperPointTensorRtRuntimeOptions().lightGluePoints;
}

struct SuperPointTensorOutputShape {
    int heatmapWidth{0};
    int heatmapHeight{0};
};

struct SuperPointDescriptorWriteRequest {
    const TensorBlob &descriptorBlob;
    int tensorBatch;
    int descriptorCount;
    double ratioW;
    double ratioH;
    SuperPointFeatureSet &output;
    SuperPointPostScratch &scratch;
    bool useDescriptorHwc;
    bool useDescriptorNearest;
};

bool ValidateSuperPointTensorOutputs(
    const SuperPointPopulateOutputRequest &request,
    SuperPointTensorOutputShape &shape)
{
    const TensorBlob &detectorBlob = request.detectorBlob;
    const TensorBlob &descriptorBlob = request.descriptorBlob;
    if (detectorBlob.dims.size() < 4 || descriptorBlob.dims.size() < 4 ||
        detectorBlob.Dim(0) <= request.tensorBatch ||
        descriptorBlob.Dim(0) <= request.tensorBatch ||
        detectorBlob.Dim(1) < 65 ||
        descriptorBlob.Dim(1) != SUPER_POINT_DESCRIPTOR_DIM) {
        if (request.err != nullptr) {
            *request.err = "TensorRT SuperPoint outputs have unexpected shapes";
        }
        return false;
    }
    shape.heatmapWidth = detectorBlob.Dim(3) * SUPER_POINT_CELL_SIZE;
    shape.heatmapHeight = detectorBlob.Dim(2) * SUPER_POINT_CELL_SIZE;
    if (shape.heatmapWidth <= request.targetWidth &&
        shape.heatmapHeight <= request.targetHeight) {
        return true;
    }
    if (request.err != nullptr) {
        *request.err =
            "TensorRT SuperPoint detector output size does not match input";
    }
    return false;
}

SuperPointPostStats ExtractSuperPointCandidates(
    const SuperPointPopulateOutputRequest &request,
    const SuperPointTensorOutputShape &shape)
{
    SuperPointPostStats imagePostStats;
    const CandidateExtractionRequest extractionRequest{
        request.detectorBlob, request.tensorBatch, shape.heatmapWidth,
        shape.heatmapHeight, request.maxPoints, request.scratch,
        &imagePostStats};
    if (EnvFlag("SMART_DRONE_SUPERPOINT_FAST_NMS", false)) {
        ExtractCandidatesFastNms(extractionRequest);
    } else {
        ExtractCandidates(extractionRequest);
    }
    return imagePostStats;
}

float SuperPointDescriptorSampleAxis(int point, int gridSize)
{
    const float cellSize = static_cast<float>(SUPER_POINT_CELL_SIZE);
    const float numerator =
        static_cast<float>(point) - cellSize / 2.0f + 0.5f;
    const float denominator =
        static_cast<float>(gridSize * SUPER_POINT_CELL_SIZE) - cellSize / 2.0f -
        0.5f;
    return numerator / denominator * static_cast<float>(gridSize - 1);
}

void SampleSuperPointDescriptor(const SuperPointDescriptorWriteRequest &request,
                                const Candidate &candidate, float *descriptor)
{
    const float sampleX =
        SuperPointDescriptorSampleAxis(candidate.x, request.descriptorBlob.Dim(3));
    const float sampleY =
        SuperPointDescriptorSampleAxis(candidate.y, request.descriptorBlob.Dim(2));
    if (request.useDescriptorNearest) {
        SampleDescriptorNearest(request.descriptorBlob, request.tensorBatch,
                                sampleX, sampleY, descriptor);
        return;
    }
    if (request.useDescriptorHwc) {
        SampleDescriptorBilinearHwc(
            {request.scratch.descriptorHwc, request.descriptorBlob.Dim(2),
             request.descriptorBlob.Dim(3), sampleX, sampleY, descriptor});
        return;
    }
    SampleDescriptorBilinear(request.descriptorBlob, request.tensorBatch,
                             sampleX, sampleY, descriptor);
}

void PopulateSuperPointKeypointsAndDescriptors(
    const SuperPointDescriptorWriteRequest &request)
{
    const std::vector<Candidate> &candidates = request.scratch.candidates;
    for (size_t i = 0; i < candidates.size(); ++i) {
        const Candidate &candidate = candidates[i];
        request.output.keypoints.emplace_back(
            static_cast<float>(candidate.x * request.ratioW),
            static_cast<float>(candidate.y * request.ratioH));
        if (static_cast<int>(i) >= request.descriptorCount) {
            continue;
        }
        float *descriptor =
            request.output.descriptors.ptr<float>(static_cast<int>(i));
        SampleSuperPointDescriptor(request, candidate, descriptor);
    }
}

int PrepareSuperPointOutputDescriptors(
    const SuperPointPopulateOutputRequest &request)
{
    SuperPointFeatureSet &output = request.output;
    const std::vector<Candidate> &candidates = request.scratch.candidates;
    output.keypoints.reserve(candidates.size());
    const int descriptorCount =
        std::min(static_cast<int>(candidates.size()),
                 std::clamp(request.descriptorLimit, 0, request.maxPoints));
    if (descriptorCount > 0) {
        output.descriptors =
            cv::Mat(descriptorCount, SUPER_POINT_DESCRIPTOR_DIM, CV_32F);
    }
    return descriptorCount;
}

bool SuperPointNativeExtractor::Impl::PopulateOutputFromTensors(
    const SuperPointPopulateOutputRequest &request)
{
    const TensorBlob &descriptorBlob = request.descriptorBlob;
    SuperPointFeatureSet &output = request.output;
    SuperPointPostScratch &scratch = request.scratch;
    output = SuperPointFeatureSet{};
    SuperPointTensorOutputShape shape;
    if (!ValidateSuperPointTensorOutputs(request, shape)) {
        return false;
    }

    const double ratioH = static_cast<double>(request.sourceImage.rows) /
                          static_cast<double>(request.targetHeight);
    const double ratioW = static_cast<double>(request.sourceImage.cols) /
                          static_cast<double>(request.targetWidth);
    SuperPointPostStats imagePostStats =
        ExtractSuperPointCandidates(request, shape);
    const int descriptorCount = PrepareSuperPointOutputDescriptors(request);
    const auto descriptorStartTp = std::chrono::steady_clock::now();
    const bool useDescriptorHwc =
        EnvFlag("SMART_DRONE_SUPERPOINT_DESCRIPTOR_HWC", false);
    const bool useDescriptorNearest =
        EnvFlag("SMART_DRONE_SUPERPOINT_DESCRIPTOR_NEAREST", false);
    if (useDescriptorHwc && descriptorCount > 0) {
        BuildDescriptorGridHwc(descriptorBlob, request.tensorBatch,
                               scratch.descriptorHwc);
    }
    PopulateSuperPointKeypointsAndDescriptors(
        {descriptorBlob, request.tensorBatch, descriptorCount, ratioW, ratioH,
         output, scratch, useDescriptorHwc, useDescriptorNearest});
    const auto descriptorEndTp = std::chrono::steady_clock::now();
    imagePostStats.descriptorMs +=
        DurationMs(descriptorStartTp, descriptorEndTp);
    imagePostStats.descriptorCount += descriptorCount;
    if (request.postStats != nullptr) {
        request.postStats->Add(imagePostStats);
    }
    return true;
}

} // namespace SmartDrone::Adapters::Slam
