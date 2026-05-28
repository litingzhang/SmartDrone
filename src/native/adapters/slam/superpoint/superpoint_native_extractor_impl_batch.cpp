#include "adapters/slam/superpoint/superpoint_native_extractor_impl.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "common/environment.h"

namespace SmartDrone::Adapters::Slam {

bool SuperPointNativeExtractor::Impl::DetectAndComputeBatch(
    const SuperPointBatchComputeRequest &request)
{
    const std::vector<cv::Mat> &grayImages = request.grayImages;
    std::vector<SuperPointFeatureSet> &outputs = request.outputs;
    outputs.assign(grayImages.size(), SuperPointFeatureSet{});
    if (!trtEngine || grayImages.empty()) {
        if (request.err != nullptr) {
            *request.err = "SuperPoint TensorRT backend is not ready";
        }
        return false;
    }
    lastSuperPointBatchedForward = false;
    lastSuperPointForwardStats = TensorRtForwardStats{};
    lastSuperPointPostStats = SuperPointPostStats{};
    SuperPointBatchContext context{
        request,
        static_cast<int>(grayImages.size()),
        inputHeight > 0 ? inputHeight : grayImages.front().rows,
        inputWidth > 0 ? inputWidth : grayImages.front().cols,
        {}};
    if (TryDetectAndComputeTensorRtBatch(context)) {
        WriteSuperPointBatchTiming(request, context.timing);
        return true;
    }
    for (size_t batchIndex = 0; batchIndex < grayImages.size(); ++batchIndex) {
        if (!DetectAndComputeSingleImage({request, batchIndex, context.timing})) {
            return false;
        }
    }
    WriteSuperPointBatchTiming(request, context.timing);
    return true;
}

bool SuperPointNativeExtractor::Impl::TryDetectAndComputeTensorRtBatch(
    SuperPointBatchContext &context)
{
    const SuperPointBatchComputeRequest &request = context.request;
    if (context.batchSize <= 1 || superPointBatchDisabled ||
        !trtEngine->SupportsBatchSize(context.batchSize)) {
        return false;
    }
    const auto inputStartTp = std::chrono::steady_clock::now();
    BuildInputBatch(request.grayImages, context.targetHeight, context.targetWidth,
                    superPointInputBatch);
    const auto inputEndTp = std::chrono::steady_clock::now();
    std::string batchErr;
    TensorRtForwardStats batchStats;
    if (!trtEngine->Forward({superPointInputBatch, context.batchSize,
                             context.targetHeight, context.targetWidth, detector,
                             descriptors, &batchStats, &batchErr})) {
        superPointBatchDisabled = true;
        std::cerr << "[superpoint_trt] warning: batch_size=" << context.batchSize
                  << " forward failed; falling back to single-image inference";
        if (!batchErr.empty()) {
            std::cerr << " err=" << batchErr;
        }
        std::cerr << "\n";
        return false;
    }
    const auto forwardEndTp = std::chrono::steady_clock::now();
    for (size_t batchIndex = 0; batchIndex < request.grayImages.size();
         ++batchIndex) {
        if (!PopulateOutputFromTensors(
                {detector, descriptors, static_cast<int>(batchIndex),
                 request.grayImages[batchIndex], context.targetHeight,
                 context.targetWidth, request.maxPoints, request.descriptorLimit,
                 request.outputs[batchIndex], mainPostScratch,
                 &lastSuperPointPostStats, request.err})) {
            return false;
        }
    }
    const auto postEndTp = std::chrono::steady_clock::now();
    context.timing.inputMs += DurationMs(inputStartTp, inputEndTp);
    context.timing.forwardMs += DurationMs(inputEndTp, forwardEndTp);
    context.timing.postMs += DurationMs(forwardEndTp, postEndTp);
    lastSuperPointForwardStats = batchStats;
    lastSuperPointBatchedForward = true;
    return true;
}

bool SuperPointNativeExtractor::Impl::DetectAndComputeSingleImage(
    const SuperPointSingleForwardRequest &request)
{
    const cv::Mat &gray = request.batch.grayImages[request.batchIndex];
    const int targetHeight = inputHeight > 0 ? inputHeight : gray.rows;
    const int targetWidth = inputWidth > 0 ? inputWidth : gray.cols;
    const auto inputStartTp = std::chrono::steady_clock::now();
    BuildInputBatch({gray}, targetHeight, targetWidth, superPointInputBatch);
    const auto inputEndTp = std::chrono::steady_clock::now();
    TensorRtForwardStats singleStats;
    if (!trtEngine->Forward({superPointInputBatch, 1, targetHeight, targetWidth,
                             detector, descriptors, &singleStats,
                             request.batch.err})) {
        return false;
    }
    const auto forwardEndTp = std::chrono::steady_clock::now();
    if (!PopulateOutputFromTensors(
            {detector, descriptors, 0, gray, targetHeight, targetWidth,
             request.batch.maxPoints, request.batch.descriptorLimit,
             request.batch.outputs[request.batchIndex], mainPostScratch,
             &lastSuperPointPostStats, request.batch.err})) {
        return false;
    }
    const auto postEndTp = std::chrono::steady_clock::now();
    request.timing.inputMs += DurationMs(inputStartTp, inputEndTp);
    request.timing.forwardMs += DurationMs(inputEndTp, forwardEndTp);
    request.timing.postMs += DurationMs(forwardEndTp, postEndTp);
    RecordSuperPointSingleStats(singleStats);
    return true;
}

void SuperPointNativeExtractor::Impl::RecordSuperPointSingleStats(
    const TensorRtForwardStats &singleStats)
{
    lastSuperPointForwardStats.h2dMs += singleStats.h2dMs;
    lastSuperPointForwardStats.enqueueMs += singleStats.enqueueMs;
    lastSuperPointForwardStats.outputMs += singleStats.outputMs;
    lastSuperPointForwardStats.outputConvertMs += singleStats.outputConvertMs;
    lastSuperPointForwardStats.syncMs += singleStats.syncMs;
    lastSuperPointForwardStats.gpuComputeMs += singleStats.gpuComputeMs;
    lastSuperPointForwardStats.gpuOutputMs += singleStats.gpuOutputMs;
    lastSuperPointForwardStats.eventTimingEnabled =
        lastSuperPointForwardStats.eventTimingEnabled ||
        singleStats.eventTimingEnabled;
    lastSuperPointForwardStats.h2dBytes += singleStats.h2dBytes;
    lastSuperPointForwardStats.d2hBytes += singleStats.d2hBytes;
}

void SuperPointNativeExtractor::Impl::WriteSuperPointBatchTiming(
    const SuperPointBatchComputeRequest &request,
    const SuperPointBatchTiming &timing)
{
    if (request.inputMs != nullptr) {
        *request.inputMs = timing.inputMs;
    }
    if (request.forwardMs != nullptr) {
        *request.forwardMs = timing.forwardMs;
    }
    if (request.postMs != nullptr) {
        *request.postMs = timing.postMs;
    }
}

bool SuperPointNativeExtractor::Impl::MatchWithLightGlue(
    const SuperPointLightGlueMatchRequest &request)
{
    request.leftOut = SuperPointFeatureSet{};
    request.rightOut = SuperPointFeatureSet{};
    ResetLightGlueStats();
    LightGlueMatchContext context{request};
    if (!PrepareLightGlueMatchContext(request, context)) {
        return false;
    }
    const auto matchStart = std::chrono::steady_clock::now();
    TensorRtForwardStats matchStats;
    if (!RunLightGlueForward(context, matchStats)) {
        return false;
    }
    const auto matchEnd = std::chrono::steady_clock::now();
    lastLightGlueForwardStats = matchStats;
    if (request.matchMs != nullptr) {
        *request.matchMs = DurationMs(matchStart, matchEnd);
    }

    const auto decodeStart = std::chrono::steady_clock::now();
    DecodeLightGlueScoreShape(context);
    const LightGlueScoreRange scoreRange = ComputeLightGlueScoreRange(context);
    if (scoreRange.valid) {
        lastLightGlueMinScore = scoreRange.minScore;
        lastLightGlueMaxScore = scoreRange.maxScore;
        lastLightGlueScoresLookLog = scoreRange.maxScore <= 0.0f;
    }
    std::vector<LightGlueMatchPair> pairs;
    SelectLightGluePairs(context, pairs);
    SortAndLimitLightGluePairs(pairs, request.maxPoints);
    lastLightGlueDecodeMs = DurationMs(decodeStart,
                                       std::chrono::steady_clock::now());
    WriteLightGlueOutputs(request, pairs);
    return true;
}

void SuperPointNativeExtractor::Impl::ResetLightGlueStats()
{
    lastLightGlueMutualCount = 0;
    lastLightGlueScorePassCount = 0;
    lastLightGlueGeometryPassCount = 0;
    lastLightGlueAcceptedCount = 0;
    lastLightGlueMinScore = 0.0f;
    lastLightGlueMaxScore = 0.0f;
    lastLightGlueDecodeMs = 0.0;
    lastLightGlueOrientation = "none";
    lastLightGlueScoresLookLog = false;
    lastLightGlueForwardStats = TensorRtForwardStats{};
    lastLightGlueRequestedPointCount = 0;
    lastLightGlueInputPointCount = 0;
    lastLightGlueStaticShapeFallback = false;
}

LightGlueDiagnostics
SuperPointNativeExtractor::Impl::CurrentLightGlueDiagnostics() const
{
    return {lastLightGlueForwardStats,
            lastLightGlueRequestedPointCount,
            lastLightGlueInputPointCount,
            lastLightGlueMutualCount,
            lastLightGlueScorePassCount,
            lastLightGlueGeometryPassCount,
            lastLightGlueAcceptedCount,
            lastLightGlueStaticShapeFallback,
            lastLightGlueMinScore,
            lastLightGlueMaxScore,
            lastLightGlueScoresLookLog,
            lastLightGlueDecodeMs,
            lastLightGlueOrientation};
}

SuperPointDiagnostics
SuperPointNativeExtractor::Impl::CurrentSuperPointDiagnostics() const
{
    return {lastSuperPointForwardStats, lastSuperPointPostStats,
            lastSuperPointBatchedForward};
}

bool SuperPointNativeExtractor::Impl::PrepareLightGlueMatchContext(
    const SuperPointLightGlueMatchRequest &request,
    LightGlueMatchContext &context)
{
    if (!lightGlueEngine || lightGluePointCount <= 0 ||
        request.leftRaw.descriptors.empty() ||
        request.rightRaw.descriptors.empty() ||
        request.leftRaw.descriptors.type() != CV_32F ||
        request.rightRaw.descriptors.type() != CV_32F ||
        request.leftRaw.descriptors.cols != SUPER_POINT_DESCRIPTOR_DIM ||
        request.rightRaw.descriptors.cols != SUPER_POINT_DESCRIPTOR_DIM) {
        return false;
    }
    context.leftCount =
        std::min({static_cast<int>(request.leftRaw.keypoints.size()),
                  request.leftRaw.descriptors.rows, lightGluePointCount});
    context.rightCount =
        std::min({static_cast<int>(request.rightRaw.keypoints.size()),
                  request.rightRaw.descriptors.rows, lightGluePointCount});
    if (context.leftCount <= 0 || context.rightCount <= 0) {
        return false;
    }
    context.requestedPointCount = std::min(
        {std::max(context.leftCount, context.rightCount), request.maxPoints,
         lightGluePointCount});
    context.inputPointCount = lightGlueDynamicPointCountDisabled
                                  ? lightGluePointCount
                                  : context.requestedPointCount;
    lastLightGlueRequestedPointCount = context.requestedPointCount;
    lastLightGlueInputPointCount = context.inputPointCount;
    return true;
}

void SuperPointNativeExtractor::Impl::PackLightGlueInputs(
    const LightGlueMatchContext &context)
{
    const size_t kptsSize = static_cast<size_t>(context.inputPointCount) * 2U;
    const size_t descSize = static_cast<size_t>(context.inputPointCount) *
                            SUPER_POINT_DESCRIPTOR_DIM;
    lightGlueKpts0.assign(kptsSize, -1000.0f);
    lightGlueKpts1.assign(kptsSize, -1000.0f);
    lightGlueDesc0.assign(descSize, 0.0f);
    lightGlueDesc1.assign(descSize, 0.0f);
    for (int i = 0; i < std::min(context.inputPointCount, context.leftCount); ++i) {
        lightGlueKpts0[static_cast<size_t>(i) * 2U] =
            context.request.leftRaw.keypoints[static_cast<size_t>(i)].x;
        lightGlueKpts0[static_cast<size_t>(i) * 2U + 1U] =
            context.request.leftRaw.keypoints[static_cast<size_t>(i)].y;
        const float *src = context.request.leftRaw.descriptors.ptr<float>(i);
        std::copy(src, src + SUPER_POINT_DESCRIPTOR_DIM,
                  lightGlueDesc0.data() +
                      static_cast<size_t>(i) * SUPER_POINT_DESCRIPTOR_DIM);
    }
    for (int i = 0; i < std::min(context.inputPointCount, context.rightCount);
         ++i) {
        lightGlueKpts1[static_cast<size_t>(i) * 2U] =
            context.request.rightRaw.keypoints[static_cast<size_t>(i)].x;
        lightGlueKpts1[static_cast<size_t>(i) * 2U + 1U] =
            context.request.rightRaw.keypoints[static_cast<size_t>(i)].y;
        const float *src = context.request.rightRaw.descriptors.ptr<float>(i);
        std::copy(src, src + SUPER_POINT_DESCRIPTOR_DIM,
                  lightGlueDesc1.data() +
                      static_cast<size_t>(i) * SUPER_POINT_DESCRIPTOR_DIM);
    }
}

bool SuperPointNativeExtractor::Impl::RunLightGlueForward(
    LightGlueMatchContext &context, TensorRtForwardStats &matchStats)
{
    const std::array<float, 2> imageSize{
        static_cast<float>(context.request.imageWidth),
        static_cast<float>(context.request.imageHeight)};
    PackLightGlueInputs(context);
    if (!lightGlueEngine->Forward(
            {lightGlueKpts0, lightGlueKpts1, lightGlueDesc0, lightGlueDesc1,
             imageSize, imageSize, context.inputPointCount, lightGlueScores,
             &matchStats, context.request.err})) {
        if (context.inputPointCount < lightGluePointCount) {
            std::string fallbackErr;
            lightGlueDynamicPointCountDisabled = true;
            lastLightGlueStaticShapeFallback = true;
            context.inputPointCount = lightGluePointCount;
            lastLightGlueInputPointCount = context.inputPointCount;
            PackLightGlueInputs(context);
            if (!lightGlueEngine->Forward(
                    {lightGlueKpts0, lightGlueKpts1, lightGlueDesc0,
                     lightGlueDesc1, imageSize, imageSize,
                     context.inputPointCount, lightGlueScores, &matchStats,
                     &fallbackErr})) {
                if (context.request.err != nullptr) {
                    *context.request.err = std::move(fallbackErr);
                }
                return false;
            }
        } else {
            return false;
        }
    }
    return true;
}

void SuperPointNativeExtractor::Impl::DecodeLightGlueScoreShape(
    LightGlueMatchContext &context) const
{
    context.matrixRows = lightGlueScores.dims.size() == 3
                             ? lightGlueScores.Dim(1)
                             : lightGlueScores.Dim(0);
    context.matrixCols = lightGlueScores.dims.size() == 3
                             ? lightGlueScores.Dim(2)
                             : lightGlueScores.Dim(1);
    context.outLeftCount = std::min(context.leftCount, context.matrixRows);
    context.outRightCount = std::min(context.rightCount, context.matrixCols);
}

LightGlueScoreRange SuperPointNativeExtractor::Impl::ComputeLightGlueScoreRange(
    const LightGlueMatchContext &context) const
{
    float minFiniteScore = std::numeric_limits<float>::infinity();
    float maxFiniteScore = -std::numeric_limits<float>::infinity();
    for (int li = 0; li < context.outLeftCount; ++li) {
        for (int ri = 0; ri < context.outRightCount; ++ri) {
            const float score = LightGlueScoreAt(li, ri);
            if (!std::isfinite(score)) {
                continue;
            }
            minFiniteScore = std::min(minFiniteScore, score);
            maxFiniteScore = std::max(maxFiniteScore, score);
        }
    }
    return {minFiniteScore, maxFiniteScore,
            std::isfinite(minFiniteScore) && std::isfinite(maxFiniteScore)};
}

float SuperPointNativeExtractor::Impl::LightGlueScoreAt(int row, int col) const
{
    const int cols = lightGlueScores.dims.size() == 3 ? lightGlueScores.Dim(2)
                                                       : lightGlueScores.Dim(1);
    return lightGlueScores.FloatData()[static_cast<size_t>(row) *
                                           static_cast<size_t>(cols) +
                                       static_cast<size_t>(col)];
}

float SuperPointNativeExtractor::Impl::LightGluePairScore(
    const LightGluePairBuildRequest &request, int leftIndex,
    int rightIndex) const
{
    return request.transpose ? LightGlueScoreAt(rightIndex, leftIndex)
                             : LightGlueScoreAt(leftIndex, rightIndex);
}

LightGlueBestMatches SuperPointNativeExtractor::Impl::FindLightGlueBestMatches(
    const LightGluePairBuildRequest &request) const
{
    const LightGlueMatchContext &context = request.context;
    LightGlueBestMatches matches;
    matches.bestRightForLeft.assign(static_cast<size_t>(context.outLeftCount), -1);
    matches.bestScoreForLeft.assign(
        static_cast<size_t>(context.outLeftCount),
        -std::numeric_limits<float>::infinity());
    matches.bestLeftForRight.assign(static_cast<size_t>(context.outRightCount), -1);
    matches.bestScoreForRight.assign(
        static_cast<size_t>(context.outRightCount),
        -std::numeric_limits<float>::infinity());
    for (int li = 0; li < context.outLeftCount; ++li) {
        for (int ri = 0; ri < context.outRightCount; ++ri) {
            const float score = LightGluePairScore(request, li, ri);
            if (!std::isfinite(score)) {
                continue;
            }
            if (score > matches.bestScoreForLeft[static_cast<size_t>(li)]) {
                matches.bestScoreForLeft[static_cast<size_t>(li)] = score;
                matches.bestRightForLeft[static_cast<size_t>(li)] = ri;
            }
            if (score > matches.bestScoreForRight[static_cast<size_t>(ri)]) {
                matches.bestScoreForRight[static_cast<size_t>(ri)] = score;
                matches.bestLeftForRight[static_cast<size_t>(ri)] = li;
            }
        }
    }
    return matches;
}

void SuperPointNativeExtractor::Impl::MaybeAppendLightGluePair(
    const LightGluePairBuildRequest &request,
    const LightGlueBestMatches &bestMatches, int leftIndex,
    std::vector<LightGlueMatchPair> &candidatePairs)
{
    const LightGlueMatchContext &context = request.context;
    const int ri = bestMatches.bestRightForLeft[static_cast<size_t>(leftIndex)];
    const float score = bestMatches.bestScoreForLeft[static_cast<size_t>(leftIndex)];
    if (ri < 0 || ri >= context.outRightCount ||
        bestMatches.bestLeftForRight[static_cast<size_t>(ri)] != leftIndex) {
        return;
    }
    ++request.stats.mutualCount;
    const float probability = lastLightGlueScoresLookLog ? std::exp(score) : score;
    if (!std::isfinite(probability) || probability < lightGlueMinScore) {
        return;
    }
    ++request.stats.scorePassCount;
    const cv::Point2f &lp =
        context.request.leftRaw.keypoints[static_cast<size_t>(leftIndex)];
    const cv::Point2f &rp =
        context.request.rightRaw.keypoints[static_cast<size_t>(ri)];
    const float disparity = lp.x - rp.x;
    if (std::abs(lp.y - rp.y) > lightGlueMaxYDiffPx ||
        disparity <= lightGlueMinDisparityPx ||
        disparity > STEREO_MAX_DISPARITY_PX) {
        return;
    }
    ++request.stats.geometryPassCount;
    candidatePairs.push_back(
        LightGlueMatchPair{leftIndex, ri, score, disparity});
}

std::vector<LightGlueMatchPair> SuperPointNativeExtractor::Impl::BuildLightGluePairs(
    const LightGluePairBuildRequest &request)
{
    const LightGlueMatchContext &context = request.context;
    const LightGlueBestMatches bestMatches = FindLightGlueBestMatches(request);
    std::vector<LightGlueMatchPair> candidatePairs;
    candidatePairs.reserve(static_cast<size_t>(context.outLeftCount));
    for (int li = 0; li < context.outLeftCount; ++li) {
        MaybeAppendLightGluePair(request, bestMatches, li, candidatePairs);
    }
    return candidatePairs;
}

void SuperPointNativeExtractor::Impl::SelectLightGluePairs(
    const LightGlueMatchContext &context, std::vector<LightGlueMatchPair> &pairs)
{
    LightGluePairBuildStats directStats;
    LightGluePairBuildStats transposeStats;
    const std::string orientation = LowerCopy(
        SmartDrone::Common::EnvStringValue(
            "SMART_DRONE_LIGHTGLUE_SCORE_ORIENTATION", "auto"));
    const bool useDirect = orientation != "transpose";
    const bool useTranspose = orientation != "direct";
    std::vector<LightGlueMatchPair> directPairs;
    if (useDirect) {
        directPairs = BuildLightGluePairs({context, false, directStats});
    }
    std::vector<LightGlueMatchPair> transposePairs;
    if (useTranspose) {
        transposePairs = BuildLightGluePairs({context, true, transposeStats});
    }
    pairs = std::move(directPairs);
    lastLightGlueMutualCount = directStats.mutualCount;
    lastLightGlueScorePassCount = directStats.scorePassCount;
    lastLightGlueGeometryPassCount = directStats.geometryPassCount;
    lastLightGlueOrientation = useDirect ? "direct" : "transpose";
    if (useTranspose &&
        (!useDirect ||
         transposeStats.geometryPassCount > directStats.geometryPassCount)) {
        pairs = std::move(transposePairs);
        lastLightGlueMutualCount = transposeStats.mutualCount;
        lastLightGlueScorePassCount = transposeStats.scorePassCount;
        lastLightGlueGeometryPassCount = transposeStats.geometryPassCount;
        lastLightGlueOrientation = "transpose";
    }
    lastLightGlueAcceptedCount = static_cast<int>(pairs.size());
}

void SuperPointNativeExtractor::Impl::SortAndLimitLightGluePairs(
    std::vector<LightGlueMatchPair> &pairs, int maxPoints)
{
    std::sort(pairs.begin(), pairs.end(),
              [](const LightGlueMatchPair &lhs,
                 const LightGlueMatchPair &rhs) {
                  if (std::abs(lhs.score - rhs.score) > 1.0e-6f) {
                      return lhs.score > rhs.score;
                  }
                  return lhs.disparity > rhs.disparity;
              });
    if (static_cast<int>(pairs.size()) > std::max(1, maxPoints)) {
        pairs.resize(static_cast<size_t>(std::max(1, maxPoints)));
    }
}

void SuperPointNativeExtractor::Impl::WriteLightGlueOutputs(
    const SuperPointLightGlueMatchRequest &request,
    const std::vector<LightGlueMatchPair> &pairs)
{
    request.leftOut.keypoints.reserve(pairs.size());
    request.rightOut.keypoints.reserve(pairs.size());
    request.leftOut.descriptors = cv::Mat(
        static_cast<int>(pairs.size()), SUPER_POINT_DESCRIPTOR_DIM, CV_32F);
    request.rightOut.descriptors =
        cv::Mat(static_cast<int>(pairs.size()), SUPER_POINT_DESCRIPTOR_DIM, CV_32F);
    for (size_t i = 0; i < pairs.size(); ++i) {
        request.leftOut.keypoints.push_back(
            request.leftRaw.keypoints[static_cast<size_t>(pairs[i].left)]);
        request.rightOut.keypoints.push_back(
            request.rightRaw.keypoints[static_cast<size_t>(pairs[i].right)]);
        request.leftRaw.descriptors.row(pairs[i].left)
            .copyTo(request.leftOut.descriptors.row(static_cast<int>(i)));
        request.rightRaw.descriptors.row(pairs[i].right)
            .copyTo(request.rightOut.descriptors.row(static_cast<int>(i)));
    }
}

} // namespace SmartDrone::Adapters::Slam
