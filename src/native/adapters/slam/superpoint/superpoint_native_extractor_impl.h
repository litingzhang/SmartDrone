#pragma once

#include "adapters/slam/superpoint/superpoint_native_extractor.h"

#include "adapters/slam/superpoint/superpoint_native_extractor_postprocess_descriptors.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_postprocess_heatmap.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_common.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_lightglue.h"
#include "adapters/slam/superpoint/superpoint_native_extractor_tensorrt_superpoint.h"
#include "adapters/slam/superpoint/superpoint_runtime_options.h"

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace SmartDrone::Adapters::Slam {

using SuperPointTensorRtInternal::BuildDescriptorGridHwc;
using SuperPointTensorRtInternal::BuildInputBatch;
using SuperPointTensorRtInternal::Candidate;
using SuperPointTensorRtInternal::CudaPinnedHostBuffer;
using SuperPointTensorRtInternal::DurationMs;
using SuperPointTensorRtInternal::EnvFlag;
using SuperPointTensorRtInternal::ExtractCandidates;
using SuperPointTensorRtInternal::ExtractCandidatesFastNms;
using SuperPointTensorRtInternal::STEREO_MAX_DISPARITY_PX;
using SuperPointTensorRtInternal::SUPER_POINT_CELL_SIZE;
using SuperPointTensorRtInternal::SUPER_POINT_DESCRIPTOR_DIM;
using SuperPointTensorRtInternal::LowerCopy;
using SuperPointTensorRtInternal::ResolveLightGlueEnginePath;
using SuperPointTensorRtInternal::ResolveSuperPointEnginePath;
using SuperPointTensorRtInternal::SampleDescriptorBilinear;
using SuperPointTensorRtInternal::SampleDescriptorBilinearHwc;
using SuperPointTensorRtInternal::SampleDescriptorNearest;
using SuperPointTensorRtInternal::SuperPointPostScratch;
using SuperPointTensorRtInternal::SuperPointPostStats;
using SuperPointTensorRtInternal::TensorBlob;
using SuperPointTensorRtInternal::TensorRtForwardStats;
using SuperPointTensorRtInternal::TensorRtLightGlueEngine;
using SuperPointTensorRtInternal::TensorRtSuperPointEngine;

struct SuperPointPopulateOutputRequest {
    const TensorBlob &detectorBlob;
    const TensorBlob &descriptorBlob;
    int tensorBatch;
    const cv::Mat &sourceImage;
    int targetHeight;
    int targetWidth;
    int maxPoints;
    int descriptorLimit;
    SuperPointFeatureSet &output;
    SuperPointPostScratch &scratch;
    SuperPointPostStats *postStats;
    std::string *err;
};

struct SuperPointBatchComputeRequest {
    const std::vector<cv::Mat> &grayImages;
    int maxPoints;
    int descriptorLimit;
    std::vector<SuperPointFeatureSet> &outputs;
    double *inputMs;
    double *forwardMs;
    double *postMs;
    std::string *err;
};

struct SuperPointBatchTiming {
    double inputMs{0.0};
    double forwardMs{0.0};
    double postMs{0.0};
};

struct SuperPointBatchContext {
    const SuperPointBatchComputeRequest &request;
    int batchSize{0};
    int targetHeight{0};
    int targetWidth{0};
    SuperPointBatchTiming timing;
};

struct SuperPointSingleForwardRequest {
    const SuperPointBatchComputeRequest &batch;
    size_t batchIndex{0U};
    SuperPointBatchTiming &timing;
};

struct SuperPointLightGlueMatchRequest {
    const SuperPointFeatureSet &leftRaw;
    const SuperPointFeatureSet &rightRaw;
    int maxPoints;
    int imageWidth;
    int imageHeight;
    SuperPointFeatureSet &leftOut;
    SuperPointFeatureSet &rightOut;
    double *matchMs;
    std::string *err;
};

struct LightGlueMatchPair {
    int left{0};
    int right{0};
    float score{0.0f};
    float disparity{0.0f};
};

struct LightGlueMatchContext {
    const SuperPointLightGlueMatchRequest &request;
    int leftCount{0};
    int rightCount{0};
    int requestedPointCount{0};
    int inputPointCount{0};
    int matrixRows{0};
    int matrixCols{0};
    int outLeftCount{0};
    int outRightCount{0};
};

struct LightGluePairBuildStats {
    int mutualCount{0};
    int scorePassCount{0};
    int geometryPassCount{0};
};

struct LightGluePairBuildRequest {
    const LightGlueMatchContext &context;
    bool transpose{false};
    LightGluePairBuildStats &stats;
};

struct LightGlueScoreRange {
    float minScore{0.0f};
    float maxScore{0.0f};
    bool valid{false};
};

struct LightGlueBestMatches {
    std::vector<int> bestRightForLeft;
    std::vector<float> bestScoreForLeft;
    std::vector<int> bestLeftForRight;
    std::vector<float> bestScoreForRight;
};

struct LightGlueDiagnostics {
    const TensorRtForwardStats &forwardStats;
    int requestedPointCount{0};
    int inputPointCount{0};
    int mutualCount{0};
    int scorePassCount{0};
    int geometryPassCount{0};
    int acceptedCount{0};
    bool staticShapeFallback{false};
    float minScore{0.0f};
    float maxScore{0.0f};
    bool scoresLookLog{false};
    double decodeMs{0.0};
    const std::string &orientation;
};

struct SuperPointDiagnostics {
    const TensorRtForwardStats &forwardStats;
    const SuperPointPostStats &postStats;
    bool batchedForward{false};
};

struct SuperPointNativeExtractor::Impl {
    std::unique_ptr<TensorRtSuperPointEngine> trtEngine;
    std::unique_ptr<TensorRtLightGlueEngine> lightGlueEngine;
    TensorBlob detector;
    TensorBlob descriptors;
    TensorBlob lightGlueScores;
    std::vector<float> superPointInputBatch;
    std::vector<float> lightGlueKpts0;
    std::vector<float> lightGlueKpts1;
    std::vector<float> lightGlueDesc0;
    std::vector<float> lightGlueDesc1;
    SuperPointPostScratch mainPostScratch;
    int inputHeight{0};
    int inputWidth{0};
    int lightGluePointCount{0};
    float lightGlueMinScore{0.02f};
    float lightGlueMaxYDiffPx{1.5f};
    float lightGlueMinDisparityPx{0.8f};
    int lightGlueEmptyDisableThreshold{3};
    int lightGlueLowYieldDisableThreshold{3};
    int lightGlueLowYieldMinPairs{8};
    int lightGlueEmptyCooldownFrames{120};
    int lightGlueEmptyCount{0};
    int lightGlueLowYieldCount{0};
    int lightGlueSkipRemaining{0};
    int lightGlueFrameCounter{0};
    int lastLightGlueMutualCount{0};
    int lastLightGlueScorePassCount{0};
    int lastLightGlueGeometryPassCount{0};
    int lastLightGlueAcceptedCount{0};
    int lastLightGlueRequestedPointCount{0};
    int lastLightGlueInputPointCount{0};
    bool lastLightGlueStaticShapeFallback{false};
    float lastLightGlueMinScore{0.0f};
    float lastLightGlueMaxScore{0.0f};
    double lastLightGlueDecodeMs{0.0};
    std::string lastLightGlueOrientation{"none"};
    bool lastLightGlueScoresLookLog{false};
    bool superPointBatchDisabled{false};
    bool lastSuperPointBatchedForward{false};
    TensorRtForwardStats lastSuperPointForwardStats{};
    TensorRtForwardStats lastLightGlueForwardStats{};
    SuperPointPostStats lastSuperPointPostStats{};
    bool lightGlueDynamicPointCountDisabled{false};

    void RecordLightGlueLowYield(int lightGlueLeftCount);
    void RecordLightGlueEmptyOutput();
    void ResetLightGlueEmptyCount();
    int NextLightGlueFrameIndex();
    bool ConsumeLightGlueCooldown();
    LightGlueDiagnostics CurrentLightGlueDiagnostics() const;
    SuperPointDiagnostics CurrentSuperPointDiagnostics() const;
    bool LoadSuperPointEngine(
        const std::filesystem::path &enginePath,
        const SuperPointTensorRtRuntimeOptions &runtimeOptions,
        std::string *err);
    void ApplyLightGlueRuntimeOptions(
        int fixedPointCount,
        const SuperPointTensorRtRuntimeOptions &runtimeOptions);
    void LogLightGlueLoaded(const std::filesystem::path &lightGluePath,
                            int fixedPointCount) const;
    void TryLoadLightGlueEngine(
        const std::string &repoPath,
        const SuperPointTensorRtRuntimeOptions &runtimeOptions);
    bool Load(const std::string &repoPath, const std::string &deviceText,
              std::string *err);
    int MaxPointsForLightGlue() const;
    bool PopulateOutputFromTensors(
        const SuperPointPopulateOutputRequest &request);
    bool DetectAndComputeBatch(const SuperPointBatchComputeRequest &request);
    bool TryDetectAndComputeTensorRtBatch(SuperPointBatchContext &context);
    bool DetectAndComputeSingleImage(const SuperPointSingleForwardRequest &request);
    void RecordSuperPointSingleStats(const TensorRtForwardStats &singleStats);
    static void WriteSuperPointBatchTiming(
        const SuperPointBatchComputeRequest &request,
        const SuperPointBatchTiming &timing);
    bool MatchWithLightGlue(const SuperPointLightGlueMatchRequest &request);
    void ResetLightGlueStats();
    bool PrepareLightGlueMatchContext(const SuperPointLightGlueMatchRequest &request,
                                      LightGlueMatchContext &context) const;
    void PackLightGlueInputs(const LightGlueMatchContext &context);
    bool RunLightGlueForward(LightGlueMatchContext &context,
                             TensorRtForwardStats &matchStats);
    void DecodeLightGlueScoreShape(LightGlueMatchContext &context) const;
    LightGlueScoreRange ComputeLightGlueScoreRange(
        const LightGlueMatchContext &context) const;
    float LightGlueScoreAt(int row, int col) const;
    float LightGluePairScore(const LightGluePairBuildRequest &request,
                             int leftIndex, int rightIndex) const;
    LightGlueBestMatches FindLightGlueBestMatches(
        const LightGluePairBuildRequest &request) const;
    void MaybeAppendLightGluePair(
        const LightGluePairBuildRequest &request,
        const LightGlueBestMatches &bestMatches, int leftIndex,
        std::vector<LightGlueMatchPair> &candidatePairs);
    std::vector<LightGlueMatchPair> BuildLightGluePairs(
        const LightGluePairBuildRequest &request);
    void SelectLightGluePairs(
        const LightGlueMatchContext &context,
        std::vector<LightGlueMatchPair> &pairs);
    void SortAndLimitLightGluePairs(std::vector<LightGlueMatchPair> &pairs,
                                    int maxPoints);
    static void WriteLightGlueOutputs(
        const SuperPointLightGlueMatchRequest &request,
        const std::vector<LightGlueMatchPair> &pairs);
};

} // namespace SmartDrone::Adapters::Slam
