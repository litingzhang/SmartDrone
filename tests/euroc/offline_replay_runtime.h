using ReplayPoseSamples = std::vector<SmartDrone::Tests::ReplayPoseSample>;
using ReplaySlamControl = SmartDrone::Adapters::Slam::ISlamRuntimeControl;
using ReplaySlamEngine = SmartDrone::Core::Ports::ISlamEngine;
using ReplayVisualFrontend =
    SmartDrone::Adapters::Slam::IManagedVisualFeatureFrontend;

struct ReplayRuntime {
    SmartDrone::Tests::ReplayCameraProvider camera;
    SmartDrone::Tests::ReplayImuProvider imu;
    std::unique_ptr<ReplaySlamEngine> slamEngine;
    ReplaySlamControl *slamControl{nullptr};
    std::unique_ptr<ReplayVisualFrontend> visualFeatureFrontendClient;

    explicit ReplayRuntime(const SmartDrone::Tests::ReplayDataset &dataset)
        : camera(dataset), imu(dataset)
    {
    }
};

struct ReplayOutputAdjustment {
    int64_t timestampOffsetNs{0};
    double positionScale{1.0};
    std::optional<Sophus::SE3f> bodyFromCameraExtrinsics;
};

struct ReplaySummary {
    size_t frameCount{0};
    size_t poseValidCount{0};
    size_t trackingOkCount{0};
    size_t trackingLostCount{0};
    size_t identityPoseCount{0};
    double orbTrackMsMean{0.0};
    double orbExtractMsMean{0.0};
    double orbStereoMsMean{0.0};
    double orbTrackMsMax{0.0};
    double orbExtractMsMax{0.0};
    double orbStereoMsMax{0.0};
    MetricAccumulator replayAcquireMs;
    MetricAccumulator replayImuMs;
    MetricAccumulator slamTotalMs;
    MetricAccumulator inputPrepareMs;
    MetricAccumulator frontendMs;
    MetricAccumulator stereoPairMs;
    MetricAccumulator featurePackMs;
    MetricAccumulator monoAugmentMs;
    MetricAccumulator lkRectifyMs;
    MetricAccumulator lkDisparityMs;
    MetricAccumulator lkGfttMs;
    MetricAccumulator lkFlowMs;
    MetricAccumulator lkCandidateMs;
    MetricAccumulator lkPnpMs;
    MetricAccumulator lkUpdateMs;
    MetricAccumulator visualFeatureFrontendMs;
    MetricAccumulator visualFeatureMatchMs;
    MetricAccumulator visualFeatureTotalMs;
};

void LogReplayStart(const OfflineReplayOptions &opts)
{
    std::cerr << "[offline_replay] slam_backend="
              << ToSlamBackendText(opts.slamBackend) << " feature_frontend="
              << ToFeatureFrontendText(opts.featureFrontend) << "\n";
    if (opts.slamBackend != SlamBackend::DpvoTensorRt) {
        return;
    }
    std::cerr << "[offline_replay] dpvo_repo=" << opts.dpvoRepo
              << " patch_engine=" << opts.dpvoPatchEngine
              << " update_engine=" << opts.dpvoUpdateEngine << "\n";
}

bool EnsureOrbSlamReplayAvailable(const OfflineReplayOptions &opts)
{
    if (opts.slamBackend != SlamBackend::OrbSlam3) {
        return true;
    }
    if (OrbSlam3BackendAvailable()) {
        ApplyOrbAccelerationEnvironment(opts.orbAcceleration);
        return true;
    }
    std::cerr << "error: ORB-SLAM3 backend is not compiled into this offline "
                 "replay target\n";
    return false;
}

RuntimeConfig BuildReplayRuntimeConfig(const OfflineReplayOptions &opts)
{
    RuntimeConfig replayRuntime{};
    replayRuntime.slamBackend = opts.slamBackend;
    replayRuntime.featureFrontend = opts.featureFrontend;
    replayRuntime.slamOperationMode = opts.slamMode;
    replayRuntime.dpvoRepo = opts.dpvoRepo;
    replayRuntime.dpvoPatchEngine = opts.dpvoPatchEngine;
    replayRuntime.dpvoUpdateEngine = opts.dpvoUpdateEngine;
    replayRuntime.dpvoInputWidth = opts.dpvoInputWidth;
    replayRuntime.dpvoInputHeight = opts.dpvoInputHeight;
    replayRuntime.dpvoPatchesPerFrame = opts.dpvoPatchesPerFrame;
    replayRuntime.dpvoOptimizationWindow = opts.dpvoOptimizationWindow;
    replayRuntime.visualFeatureRepo = opts.visualFeatureRepo;
    replayRuntime.visualFeatureDevice = opts.visualFeatureDevice;
    replayRuntime.visualFeatureTopK = opts.visualFeatureTopK;
    replayRuntime.visualFeatureMaxPoints = opts.visualFeatureMaxPoints;
    replayRuntime.visualFeatureInputMaxWidth = opts.visualFeatureInputMaxWidth;
    replayRuntime.visualFeatureInputMaxHeight = opts.visualFeatureInputMaxHeight;
    replayRuntime.lkLoopClosure = opts.lkRuntimeLoopClosure;
    replayRuntime.lkLoopScale = opts.lkLoopScale;
    replayRuntime.lkLoopRelaxation = opts.lkLoopRelaxation;
    replayRuntime.lkPerFrameAcceleration = opts.lkPerFrameAcceleration;
    replayRuntime.orbAcceleration = opts.orbAcceleration;
    return replayRuntime;
}

SmartDrone::Adapters::Slam::SlamEngineFactoryConfig
BuildReplayEngineConfig(const OfflineReplayOptions &opts)
{
    SmartDrone::Adapters::Slam::SlamEngineFactoryConfig engineConfig{};
    engineConfig.backend = opts.slamBackend;
    engineConfig.vocabularyPath = opts.vocab;
    engineConfig.settingsPath = opts.settings;
    engineConfig.sensorMode = opts.sensorMode;
    engineConfig.useViewer = false;
    engineConfig.useImu = UseImu(opts.sensorMode);
    engineConfig.inputMode = ResolveSlamInputMode(opts.sensorMode);
    engineConfig.dpvoRuntime.repoPath = opts.dpvoRepo;
    engineConfig.dpvoRuntime.patchEnginePath = opts.dpvoPatchEngine;
    engineConfig.dpvoRuntime.updateEnginePath = opts.dpvoUpdateEngine;
    engineConfig.dpvoRuntime.inputWidth = opts.dpvoInputWidth;
    engineConfig.dpvoRuntime.inputHeight = opts.dpvoInputHeight;
    engineConfig.dpvoRuntime.patchesPerFrame = opts.dpvoPatchesPerFrame;
    engineConfig.dpvoRuntime.optimizationWindow = opts.dpvoOptimizationWindow;
    return engineConfig;
}

int CreateReplaySlamEngine(const OfflineReplayOptions &opts,
                           ReplayRuntime &runtime)
{
    SmartDrone::Adapters::Slam::ControlledSlamEngine controlled =
        SmartDrone::Adapters::Slam::CreateSlamEngine(
            BuildReplayEngineConfig(opts));
    if (controlled.engine == nullptr) {
        std::cerr << "error: SLAM backend failed to initialize\n";
        return 2;
    }
    runtime.slamControl = controlled.control;
    runtime.slamEngine = std::move(controlled.engine);
    return 0;
}

void ConfigureReplaySlamControl(const OfflineReplayOptions &opts,
                                ReplaySlamControl &slamControl)
{
    slamControl.SetOperationMode(opts.slamMode);
    slamControl.SetFeatureFrontend(opts.featureFrontend);
    slamControl.SetVisualFeatureInputSizeLimit(opts.visualFeatureInputMaxWidth,
                                               opts.visualFeatureInputMaxHeight);
    slamControl.SetStereoVoLoopClosure(opts.lkRuntimeLoopClosure,
                                       opts.lkLoopScale, opts.lkLoopRelaxation);
    slamControl.SetStereoVoPerFrameAcceleration(opts.lkPerFrameAcceleration);
}

bool VisualFrontendRequested(const OfflineReplayOptions &opts,
                             const ReplayRuntime &runtime)
{
    return runtime.slamControl != nullptr &&
           IsVisualFeatureLightGlueFrontend(opts.featureFrontend) &&
           SmartDrone::Adapters::Slam::VisualFeatureFrontendClientEnabled(
               opts.featureFrontend);
}

SmartDrone::Adapters::Slam::VisualFeatureFrontendRuntimeConfig
BuildReplayVisualFrontendConfig(const OfflineReplayOptions &opts)
{
    SmartDrone::Adapters::Slam::VisualFeatureFrontendRuntimeConfig config{};
    config.repoPath =
        SmartDrone::Adapters::Slam::ResolveVisualFeatureFrontendRepo(
            opts.featureFrontend, opts.visualFeatureRepo);
    config.device = opts.visualFeatureDevice;
    config.topK = opts.visualFeatureTopK;
    config.maxPoints = opts.visualFeatureMaxPoints;
    config.inputMaxWidth = opts.visualFeatureInputMaxWidth;
    config.inputMaxHeight = opts.visualFeatureInputMaxHeight;
    SmartDrone::Adapters::Slam::ConfigureVisualFeatureFrontendDefaults(
        opts.featureFrontend, config);
    return config;
}

int StartReplayVisualFrontend(const OfflineReplayOptions &opts,
                              ReplayRuntime &runtime)
{
    if (!VisualFrontendRequested(opts, runtime)) {
        return 0;
    }
    runtime.visualFeatureFrontendClient =
        SmartDrone::Adapters::Slam::CreateVisualFeatureFrontendClient(
            opts.featureFrontend);
    if (runtime.visualFeatureFrontendClient == nullptr) {
        std::cerr << "error: no visual feature frontend client registered for "
                  << ToFeatureFrontendText(opts.featureFrontend) << "\n";
        return 2;
    }
    std::string featureErr;
    const auto featureConfig = BuildReplayVisualFrontendConfig(opts);
    if (!runtime.visualFeatureFrontendClient->Start(featureConfig, &featureErr)) {
        std::cerr << "error: " << ToFeatureFrontendText(opts.featureFrontend)
                  << " frontend start failed: " << featureErr << "\n";
        return 2;
    }
    runtime.slamControl->SetVisualFeatureFrontend(
        runtime.visualFeatureFrontendClient.get());
    return 0;
}

int InitializeReplayRuntime(const OfflineReplayOptions &opts,
                            ReplayRuntime &runtime)
{
    if (const int status = CreateReplaySlamEngine(opts, runtime); status != 0) {
        return status;
    }
    if (runtime.slamControl != nullptr) {
        ConfigureReplaySlamControl(opts, *runtime.slamControl);
    }
    return StartReplayVisualFrontend(opts, runtime);
}

void CreateDirectoryForPath(const fs::path &path)
{
    if (!path.empty() && !path.parent_path().empty()) {
        fs::create_directories(path.parent_path());
    }
}

void PrepareReplayOutputDirectories(const OfflineReplayOptions &opts)
{
    CreateDirectoryForPath(opts.outputCsv);
    CreateDirectoryForPath(opts.summaryJson);
    CreateDirectoryForPath(opts.finalEurocTrajectory);
}

std::optional<Sophus::SE3f>
LoadReplayBodyExtrinsics(const OfflineReplayOptions &opts)
{
    if (UseImu(opts.sensorMode) || opts.sensorMode != SensorMode::Stereo ||
        !EnvFlagEnabled("SMART_DRONE_EUROC_OUTPUT_BODY_FRAME", false)) {
        return std::nullopt;
    }
    auto extrinsics = LoadBodyToCameraExtrinsics(opts.settings);
    if (!extrinsics.has_value()) {
        return std::nullopt;
    }
    const Eigen::Vector3f t = extrinsics->translation();
    std::cerr << "[offline_replay] pure stereo realtime CSV uses body frame "
                 "via T_b_c1"
              << " tx=" << t.x() << " ty=" << t.y() << " tz=" << t.z()
              << "\n";
    return extrinsics;
}

SmartDrone::Tests::ReplaySlamRunnerConfig
BuildReplayRunnerConfig(const OfflineReplayOptions &opts)
{
    return {.cameraFps = opts.cameraFps,
            .slamInputFps = opts.slamInputFps,
            .useImu = UseImu(opts.sensorMode),
            .preferLatestFrame = true,
            .timeoutMs = opts.timeoutMs,
            .shutdownEngineOnFinish = false};
}

ReplayPoseSamples RunReplayFrames(const OfflineReplayOptions &opts,
                                  ReplayRuntime &runtime, std::ostream &csv,
                                  const ReplayOutputAdjustment &adjustment)
{
    SmartDrone::Tests::ReplaySlamRunner runner(
        runtime.camera, runtime.imu, *runtime.slamEngine,
        BuildReplayRunnerConfig(opts));
    SmartDrone::Core::Application::FrameTimingTracker timingTracker(512);
    return runner.Run(
        opts.maxFrames, &timingTracker,
        [&](const SmartDrone::Tests::ReplayPoseSample &sample) {
            WriteReplayCsvSample(
                csv, AdjustReplayOutputSample(
                         sample, adjustment.timestampOffsetNs,
                         adjustment.positionScale,
                         adjustment.bodyFromCameraExtrinsics));
            csv.flush();
        });
}

void StopReplayVisualFrontend(ReplayRuntime &runtime)
{
    if (runtime.visualFeatureFrontendClient != nullptr) {
        runtime.visualFeatureFrontendClient->Stop();
    }
}

int HandleEmptyReplayOutput(ReplayRuntime &runtime)
{
    runtime.slamEngine->Stop();
    StopReplayVisualFrontend(runtime);
    std::cerr << "offline replay failed: no output frames; check dataset, "
                 "camera provider, or SLAM backend startup\n";
    return 3;
}
