using SuperPointTensorRtInternal::BuildDescriptorGridHwc;
using SuperPointTensorRtInternal::BuildInputBatch;
using SuperPointTensorRtInternal::Candidate;
using SuperPointTensorRtInternal::CudaPinnedHostBuffer;
using SuperPointTensorRtInternal::DurationMs;
using SuperPointTensorRtInternal::EnvFlag;
using SuperPointTensorRtInternal::ExtractCandidates;
using SuperPointTensorRtInternal::ExtractCandidatesFastNms;
using SuperPointTensorRtInternal::kStereoMaxDisparityPx;
using SuperPointTensorRtInternal::kSuperPointCellSize;
using SuperPointTensorRtInternal::kSuperPointDescriptorDim;
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
    bool Load(const std::string &repoPath, const std::string &deviceText,
              std::string *err);
    int maxPointsForLightGlue() const;
    bool PopulateOutputFromTensors(
        const TensorBlob &detectorBlob, const TensorBlob &descriptorBlob,
        int tensorBatch, const cv::Mat &sourceImage, int targetHeight,
        int targetWidth, int maxPoints, int descriptorLimit,
        SuperPointFeatureSet &output, SuperPointPostScratch &scratch,
        SuperPointPostStats *postStats, std::string *err);
    bool DetectAndComputeBatch(const std::vector<cv::Mat> &grayImages,
                               int maxPoints, int descriptorLimit,
                               std::vector<SuperPointFeatureSet> &outputs,
                               double *inputMs, double *forwardMs, double *postMs,
                               std::string *err);
    bool MatchWithLightGlue(const SuperPointFeatureSet &leftRaw,
                            const SuperPointFeatureSet &rightRaw, int maxPoints,
                            int imageWidth, int imageHeight,
                            SuperPointFeatureSet &leftOut,
                            SuperPointFeatureSet &rightOut, double *matchMs,
                            std::string *err);
};
