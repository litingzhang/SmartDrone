using DpvoTensorRtInternal::CudaStreamHandle;
using DpvoTensorRtInternal::DimsToString;
using DpvoTensorRtInternal::DpvoCudaKernelRuntime;
using DpvoTensorRtInternal::DpvoFrameState;
using DpvoTensorRtInternal::DpvoGraphState;
using DpvoTensorRtInternal::DpvoIntrinsics;
using DpvoTensorRtInternal::DpvoNativeSolver;
using DpvoTensorRtInternal::DpvoPatchifierRun;
using DpvoTensorRtInternal::DpvoPatchifierRuntime;
using DpvoTensorRtInternal::DpvoPatchState;
using DpvoTensorRtInternal::DpvoUpdatePostAggRuntime;
using DpvoTensorRtInternal::DpvoUpdatePreAggRuntime;
using DpvoTensorRtInternal::DpvoUpdateRun;
using DpvoTensorRtInternal::DpvoUpdateRuntime;
using DpvoTensorRtInternal::TensorRtEngineHandle;

struct DpvoTensorRtEngine::Impl {
    explicit Impl(DpvoTensorRtConfig cfg)
        : config(std::move(cfg))
    {
    }
    ~Impl()
    {
        Stop();
    }

    struct DpvoEnginePaths {
        std::filesystem::path patch;
        std::filesystem::path update;
        std::filesystem::path preAgg;
        std::filesystem::path postAgg;
    };

    struct DpvoStartWarmupRuns {
        DpvoUpdateRun update;
        DpvoUpdateRun preAgg;
        DpvoUpdateRun postAgg;
    };

    struct DpvoFramePreparation {
        bool valid{false};
        float scaleX{1.0F};
        float scaleY{1.0F};
    };

    struct DpvoFrameSchedule {
        bool heavyFrame{false};
        bool rightFrame{false};
    };

    struct DpvoPatchifierFrameRuns {
        DpvoPatchifierRun left;
        DpvoPatchifierRun right;
        std::string leftError;
        std::string rightError;
        bool rightRequested{false};
    };

    struct DpvoNativeStepResult {
        bool poseUpdated{false};
        double nativeUpdateMs{0.0};
        std::string error;
    };

    DpvoEnginePaths ResolveStartEnginePaths() const;
    bool CheckRequiredEnginePaths(const DpvoEnginePaths &paths) const;
    bool LoadBaseEngines(const DpvoEnginePaths &paths, std::string &err);
    bool LoadSplitSoftAggEngines(const DpvoEnginePaths &paths,
                                 std::string &err);
    bool InitializeCudaSupport(std::string &err);
    bool InitializeTensorRtRuntimes(std::string &err);
    bool WarmupTensorRtRuntimes(DpvoStartWarmupRuns &warmups,
                                std::string &err);
    void ResetStartState();
    void ConfigurePacing();
    void LogReady(const DpvoStartWarmupRuns &warmups) const;
    void LogPacingOnce();
    bool LoadCalibration();
    bool Start();
    Core::Ports::SlamOutput Process(const Core::Ports::SlamInputBatch &input,
                                    bool extractFeatures,
                                    bool extractPointCloud);
    Core::Ports::SlamOutput BuildBaseOutput(
        const Core::Ports::SlamInputBatch &input) const;
    void ResizeFrameInput(const cv::Mat &input, cv::Mat &output);
    DpvoFramePreparation PrepareFrameInput(
        const Core::Ports::SlamInputBatch &input,
        Core::Ports::SlamOutput &out);
    DpvoFrameSchedule ResolveFrameSchedule();
    Core::Ports::SlamOutput BuildPacedFrameOutput(
        const std::chrono::steady_clock::time_point &start);
    DpvoPatchifierFrameRuns RunPatchifierFrame(bool rightFrame);
    DpvoIntrinsics MakeFrameIntrinsics(
        const DpvoFramePreparation &preparation) const;
    void LogPatchifierShape(const DpvoPatchifierRun &patchRun);
    void LogRightPatchifierError(const DpvoPatchifierFrameRuns &runs);
    void ApplyRightPatchifierDepth(const DpvoPatchifierRun &rightPatchRun,
                                   const DpvoIntrinsics &intrinsics);
    void ApplyPatchifierRuns(const DpvoPatchifierFrameRuns &runs,
                             const DpvoIntrinsics &intrinsics,
                             const Core::Ports::SlamInputBatch &input,
                             Core::Ports::SlamOutput &out);
    DpvoNativeStepResult RunNativeStep(const DpvoPatchifierRun &patchRun,
                                       const DpvoIntrinsics &intrinsics);
    void PopulateGraphCounts(Core::Ports::SlamOutput &out) const;
    void LogKeyframeRemovals(int keyframeRemovalsBefore);
    void UpdateLastPose(bool poseUpdated);
    void ResolveTrackingState(const DpvoNativeStepResult &step,
                              Core::Ports::SlamOutput &out);
    void ApplyNativeStepResult(const DpvoNativeStepResult &step,
                               Core::Ports::SlamOutput &out);
    void PopulateFeatureOutput(bool extractFeatures, float scaleX,
                               float scaleY, Core::Ports::SlamOutput &out);
    Core::Ports::SlamOutput FinishFrameOutput(
        const std::chrono::steady_clock::time_point &start,
        bool extractFeatures, const DpvoFramePreparation &preparation,
        Core::Ports::SlamOutput &out);

    void Stop()
    {
        running.store(false, std::memory_order_release);
        nativeSolver.Reset();
        cudaKernelReady = false;
        cudaStream.Reset();
    }

    DpvoTensorRtConfig config;
    TensorRtEngineHandle patchEngine;
    TensorRtEngineHandle updateEngine;
    TensorRtEngineHandle updatePreAggEngine;
    TensorRtEngineHandle updatePostAggEngine;
    CudaStreamHandle cudaStream;
    DpvoPatchifierRuntime patchifierRuntime;
    DpvoPatchifierRuntime patchifierRightRuntime;
    DpvoUpdateRuntime updateRuntime;
    DpvoUpdatePreAggRuntime updatePreAggRuntime;
    DpvoUpdatePostAggRuntime updatePostAggRuntime;
    DpvoCudaKernelRuntime cudaKernelRuntime;
    DpvoGraphState graphState;
    DpvoNativeSolver nativeSolver;
    SlamModeSharedState voState;
    cv::Mat resizedGray;
    cv::Mat resizedRightGray;
    Core::Ports::PoseEstimate lastPose{};
    uint64_t processedFrameCount{0};
    int heavyEveryN{1};
    int rightEveryN{1};
    int heavyIntervalMs{0};
    int warmupFullFrames{8};
    std::atomic<int64_t> lastHeavyUpdateEndNs{0};
    bool haveLastPose{false};
    std::atomic<bool> running{false};
    bool softAggSplitReady{false};
    bool cudaKernelReady{false};
    bool loggedEpgPacing{false};
    bool loggedPatchifierShape{false};
    bool loggedPatchifierError{false};
    bool loggedRightPatchifierError{false};
    bool loggedNativeSolverWait{false};
    bool loggedStereoDepthInit{false};
    int loggedKeyframeRemovals{0};
};
