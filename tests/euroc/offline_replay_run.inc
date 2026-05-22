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
    engineConfig.runtime = BuildReplayRuntimeConfig(opts);
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

void WarnRealtimeLoopClosureDisabled(const OfflineReplayOptions &opts)
{
    if (opts.lkLoopClosure && opts.featureFrontend == FeatureFrontend::LK) {
        std::cerr << "warning: --lk-loop-closure is disabled for euroc_pose.csv "
                     "because replay output is realtime-only\n";
    }
}

bool SampleHasIdentityPose(const SmartDrone::Tests::ReplayPoseSample &sample)
{
    return sample.poseValid && sample.pose.x == 0.0f &&
           sample.pose.y == 0.0f && sample.pose.z == 0.0f &&
           sample.pose.qw == 1.0f && sample.pose.qx == 0.0f &&
           sample.pose.qy == 0.0f && sample.pose.qz == 0.0f;
}

void UpdateReplayPoseSummary(ReplaySummary &summary,
                             const SmartDrone::Tests::ReplayPoseSample &sample)
{
    summary.poseValidCount += sample.poseValid ? 1u : 0u;
    summary.trackingOkCount +=
        (sample.trackingState == 2 || sample.trackingState == 3) ? 1u : 0u;
    summary.trackingLostCount += sample.trackingState == 1 ? 1u : 0u;
    summary.identityPoseCount += SampleHasIdentityPose(sample) ? 1u : 0u;
}

void UpdateReplayOrbSummary(ReplaySummary &summary,
                            const SmartDrone::Tests::ReplayPoseSample &sample)
{
    summary.orbTrackMsMean += sample.orbTrackMs;
    summary.orbExtractMsMean += sample.orbExtractMs;
    summary.orbStereoMsMean += sample.orbStereoMatchMs;
    summary.orbTrackMsMax = std::max(summary.orbTrackMsMax, sample.orbTrackMs);
    summary.orbExtractMsMax =
        std::max(summary.orbExtractMsMax, sample.orbExtractMs);
    summary.orbStereoMsMax =
        std::max(summary.orbStereoMsMax, sample.orbStereoMatchMs);
}

void UpdateReplayTimingSummary(
    ReplaySummary &summary, const SmartDrone::Tests::ReplayPoseSample &sample)
{
    summary.replayAcquireMs.Add(sample.replayAcquireMs);
    summary.replayImuMs.Add(sample.replayImuMs);
    summary.slamTotalMs.Add(sample.slamTotalMs);
    summary.inputPrepareMs.Add(sample.inputPrepareMs);
    summary.frontendMs.Add(sample.frontendMs);
    summary.stereoPairMs.Add(sample.stereoPairMs);
    summary.featurePackMs.Add(sample.featurePackMs);
    summary.monoAugmentMs.Add(sample.monoAugmentMs);
    summary.lkRectifyMs.Add(sample.lkRectifyMs);
    summary.lkDisparityMs.Add(sample.lkDisparityMs);
    summary.lkGfttMs.Add(sample.lkGfttMs);
    summary.lkFlowMs.Add(sample.lkFlowMs);
    summary.lkCandidateMs.Add(sample.lkCandidateMs);
    summary.lkPnpMs.Add(sample.lkPnpMs);
    summary.lkUpdateMs.Add(sample.lkUpdateMs);
    summary.visualFeatureFrontendMs.Add(sample.visualFeatureFrontendMs);
    summary.visualFeatureMatchMs.Add(sample.visualFeatureStereoMatchMs);
    summary.visualFeatureTotalMs.Add(sample.visualFeatureTotalMs);
}

void FinalizeReplaySummary(ReplaySummary &summary)
{
    const double frameCountForMean =
        static_cast<double>(std::max<size_t>(1, summary.frameCount));
    summary.orbTrackMsMean /= frameCountForMean;
    summary.orbExtractMsMean /= frameCountForMean;
    summary.orbStereoMsMean /= frameCountForMean;
}

ReplaySummary BuildReplaySummary(const ReplayPoseSamples &outputs)
{
    ReplaySummary summary;
    summary.frameCount = outputs.size();
    for (const auto &sample : outputs) {
        UpdateReplayPoseSummary(summary, sample);
        UpdateReplayOrbSummary(summary, sample);
        UpdateReplayTimingSummary(summary, sample);
    }
    FinalizeReplaySummary(summary);
    return summary;
}

int SaveFinalTrajectoryOrStop(const OfflineReplayOptions &opts,
                              ReplayRuntime &runtime)
{
    if (opts.finalEurocTrajectory.empty()) {
        runtime.slamEngine->Stop();
        return 0;
    }
    if (runtime.slamControl == nullptr) {
        std::cerr << "failed to save final EuRoC trajectory: final trajectory "
                     "export is ORB-SLAM3 only\n";
        return 1;
    }
    if (runtime.slamControl->ShutdownAndSaveTrajectoryEuRoC(
            opts.finalEurocTrajectory.string())) {
        return 0;
    }
    std::cerr << "failed to save final EuRoC trajectory: "
              << opts.finalEurocTrajectory << "\n";
    return 1;
}

void PrintReplayMetric(const char *label, const MetricAccumulator &metric,
                       size_t frameCount)
{
    std::cout << "  " << label << "_mean/max: " << metric.Mean(frameCount)
              << "/" << metric.max << "\n";
}

void PrintReplayConfigSummary(const OfflineReplayOptions &opts,
                              const ReplayOutputAdjustment &adjustment)
{
    std::cout << "offline replay complete\n";
    std::cout << "  dataset: " << opts.datasetRoot << "\n";
    std::cout << "  settings: " << opts.settings << "\n";
    if (opts.slamBackend == SlamBackend::OrbSlam3) {
        std::cout << "  vocab: " << opts.vocab << "\n";
    }
    std::cout << "  sensor_mode: " << ToSensorModeText(opts.sensorMode) << "\n";
    std::cout << "  slam_backend: " << ToSlamBackendText(opts.slamBackend)
              << "\n";
    std::cout << "  feature_frontend: "
              << ToFeatureFrontendText(opts.featureFrontend) << "\n";
    std::cout << "  euroc_output_timestamp_offset_ms: "
              << (static_cast<double>(adjustment.timestampOffsetNs) / 1000000.0)
              << "\n";
    std::cout << "  euroc_output_position_scale: " << adjustment.positionScale
              << "\n";
    std::cout << "  orb_accel: " << opts.orbAcceleration << "\n";
    std::cout << "  lk_per_frame_accel: " << opts.lkPerFrameAcceleration << "\n";
}

void PrintReplayDpvoSummary(const OfflineReplayOptions &opts)
{
    if (opts.slamBackend != SlamBackend::DpvoTensorRt) {
        return;
    }
    std::cout << "  dpvo_repo: " << opts.dpvoRepo << "\n";
    std::cout << "  dpvo_patch_engine: " << opts.dpvoPatchEngine << "\n";
    std::cout << "  dpvo_update_engine: " << opts.dpvoUpdateEngine << "\n";
    std::cout << "  dpvo_input: " << std::clamp(opts.dpvoInputWidth, 160, 1280)
              << "x" << std::clamp(opts.dpvoInputHeight, 120, 960) << "\n";
}

void PrintReplayCountSummary(const ReplaySummary &summary)
{
    std::cout << "  frames_out: " << summary.frameCount << "\n";
    std::cout << "  pose_valid_frames: " << summary.poseValidCount << "\n";
    std::cout << "  tracking_ok_frames: " << summary.trackingOkCount << "\n";
    std::cout << "  tracking_lost_frames: " << summary.trackingLostCount << "\n";
    std::cout << "  identity_pose_frames: " << summary.identityPoseCount << "\n";
}

void PrintReplayCoreTiming(const ReplaySummary &summary)
{
    const size_t frames = summary.frameCount;
    PrintReplayMetric("replay_acquire_ms", summary.replayAcquireMs, frames);
    PrintReplayMetric("replay_imu_ms", summary.replayImuMs, frames);
    PrintReplayMetric("slam_total_ms", summary.slamTotalMs, frames);
    PrintReplayMetric("input_prepare_ms", summary.inputPrepareMs, frames);
    PrintReplayMetric("frontend_ms", summary.frontendMs, frames);
    PrintReplayMetric("stereo_pair_ms", summary.stereoPairMs, frames);
    PrintReplayMetric("feature_pack_ms", summary.featurePackMs, frames);
    PrintReplayMetric("external_pack_ms", summary.featurePackMs, frames);
}

void PrintReplayLkTiming(const ReplaySummary &summary)
{
    const size_t frames = summary.frameCount;
    PrintReplayMetric("lk_disparity_ms", summary.lkDisparityMs, frames);
    PrintReplayMetric("lk_gftt_ms", summary.lkGfttMs, frames);
    PrintReplayMetric("lk_flow_ms", summary.lkFlowMs, frames);
    PrintReplayMetric("lk_candidate_ms", summary.lkCandidateMs, frames);
    PrintReplayMetric("lk_pnp_ms", summary.lkPnpMs, frames);
}

void PrintReplayOrbTiming(const ReplaySummary &summary)
{
    std::cout << "  orb_track_ms_mean/max: " << summary.orbTrackMsMean << "/"
              << summary.orbTrackMsMax << "\n";
    std::cout << "  orb_extract_ms_mean/max: " << summary.orbExtractMsMean << "/"
              << summary.orbExtractMsMax << "\n";
    std::cout << "  orb_stereo_ms_mean/max: " << summary.orbStereoMsMean << "/"
              << summary.orbStereoMsMax << "\n";
}

void PrintReplaySummary(const OfflineReplayOptions &opts,
                        const ReplayOutputAdjustment &adjustment,
                        const ReplaySummary &summary)
{
    PrintReplayConfigSummary(opts, adjustment);
    PrintReplayDpvoSummary(opts);
    PrintReplayCountSummary(summary);
    PrintReplayCoreTiming(summary);
    PrintReplayLkTiming(summary);
    PrintReplayOrbTiming(summary);
    std::cout << "  output_csv: " << opts.outputCsv << "\n";
    if (!opts.finalEurocTrajectory.empty()) {
        std::cout << "  final_euroc_trajectory: " << opts.finalEurocTrajectory
                  << "\n";
    }
}

void WriteJsonString(std::ostream &json, const char *name,
                     const std::string &value, bool comma = true)
{
    json << "  \"" << name << "\": \"" << value << "\"";
    json << (comma ? ",\n" : "\n");
}

template <typename T>
void WriteJsonNumber(std::ostream &json, const char *name, T value,
                     bool comma = true)
{
    json << "  \"" << name << "\": " << value;
    json << (comma ? ",\n" : "\n");
}

void WriteReplaySummaryJsonConfig(
    std::ostream &json, const OfflineReplayOptions &opts,
    const ReplayOutputAdjustment &adjustment)
{
    WriteJsonString(json, "dataset", opts.datasetRoot.string());
    WriteJsonString(json, "settings", opts.settings);
    WriteJsonString(json, "vocab",
                    opts.slamBackend == SlamBackend::OrbSlam3 ? opts.vocab : "");
    WriteJsonString(json, "sensor_mode", ToSensorModeText(opts.sensorMode));
    WriteJsonString(json, "slam_backend", ToSlamBackendText(opts.slamBackend));
    WriteJsonString(json, "feature_frontend",
                    ToFeatureFrontendText(opts.featureFrontend));
    WriteJsonNumber(json, "euroc_output_timestamp_offset_ms",
                    static_cast<double>(adjustment.timestampOffsetNs) /
                        1000000.0);
    WriteJsonNumber(json, "euroc_output_position_scale",
                    adjustment.positionScale);
    WriteJsonString(json, "orb_accel", opts.orbAcceleration);
    WriteJsonString(json, "dpvo_repo", opts.dpvoRepo);
    WriteJsonString(json, "dpvo_patch_engine", opts.dpvoPatchEngine);
    WriteJsonString(json, "dpvo_update_engine", opts.dpvoUpdateEngine);
    WriteJsonNumber(json, "dpvo_input_width",
                    std::clamp(opts.dpvoInputWidth, 160, 1280));
    WriteJsonNumber(json, "dpvo_input_height",
                    std::clamp(opts.dpvoInputHeight, 120, 960));
    WriteJsonNumber(json, "dpvo_patches_per_frame",
                    std::clamp(opts.dpvoPatchesPerFrame, 16, 256));
    WriteJsonNumber(json, "dpvo_optimization_window",
                    std::clamp(opts.dpvoOptimizationWindow, 4, 32));
    WriteJsonString(json, "lk_per_frame_accel", opts.lkPerFrameAcceleration);
}

void WriteReplaySummaryJsonCounts(std::ostream &json,
                                  const ReplaySummary &summary)
{
    WriteJsonNumber(json, "frames_out", summary.frameCount);
    WriteJsonNumber(json, "pose_valid_frames", summary.poseValidCount);
    WriteJsonNumber(json, "tracking_ok_frames", summary.trackingOkCount);
    WriteJsonNumber(json, "tracking_lost_frames", summary.trackingLostCount);
    WriteJsonNumber(json, "identity_pose_frames", summary.identityPoseCount);
}

void WriteReplayMetricJson(std::ostream &json, const char *name,
                           const MetricAccumulator &metric, size_t frameCount)
{
    WriteJsonNumber(json, (std::string(name) + "_mean").c_str(),
                    metric.Mean(frameCount));
    WriteJsonNumber(json, (std::string(name) + "_max").c_str(), metric.max);
}

void WriteReplaySummaryJsonTiming(std::ostream &json,
                                  const ReplaySummary &summary)
{
    const size_t frames = summary.frameCount;
    WriteReplayMetricJson(json, "replay_acquire_ms", summary.replayAcquireMs,
                          frames);
    WriteReplayMetricJson(json, "replay_imu_ms", summary.replayImuMs, frames);
    WriteReplayMetricJson(json, "slam_total_ms", summary.slamTotalMs, frames);
    WriteReplayMetricJson(json, "input_prepare_ms", summary.inputPrepareMs,
                          frames);
    WriteReplayMetricJson(json, "frontend_ms", summary.frontendMs, frames);
    WriteReplayMetricJson(json, "stereo_pair_ms", summary.stereoPairMs, frames);
    WriteReplayMetricJson(json, "feature_pack_ms", summary.featurePackMs,
                          frames);
    WriteReplayMetricJson(json, "external_pack_ms", summary.featurePackMs,
                          frames);
    WriteReplayMetricJson(json, "mono_augment_ms", summary.monoAugmentMs,
                          frames);
    WriteReplayMetricJson(json, "lk_rectify_ms", summary.lkRectifyMs, frames);
    WriteReplayMetricJson(json, "lk_disparity_ms", summary.lkDisparityMs,
                          frames);
    WriteReplayMetricJson(json, "lk_gftt_ms", summary.lkGfttMs, frames);
    WriteReplayMetricJson(json, "lk_flow_ms", summary.lkFlowMs, frames);
    WriteReplayMetricJson(json, "lk_candidate_ms", summary.lkCandidateMs,
                          frames);
    WriteReplayMetricJson(json, "lk_pnp_ms", summary.lkPnpMs, frames);
}

void WriteReplaySummaryJsonVisual(std::ostream &json,
                                  const ReplaySummary &summary)
{
    const size_t frames = summary.frameCount;
    WriteReplayMetricJson(json, "lk_update_ms", summary.lkUpdateMs, frames);
    WriteReplayMetricJson(json, "visual_feature_frontend_ms",
                          summary.visualFeatureFrontendMs, frames);
    WriteReplayMetricJson(json, "visual_feature_match_ms",
                          summary.visualFeatureMatchMs, frames);
    WriteReplayMetricJson(json, "visual_feature_total_ms",
                          summary.visualFeatureTotalMs, frames);
    WriteReplayMetricJson(json, "superpoint_frontend_ms",
                          summary.visualFeatureFrontendMs, frames);
    WriteReplayMetricJson(json, "superpoint_match_ms",
                          summary.visualFeatureMatchMs, frames);
    WriteReplayMetricJson(json, "superpoint_total_ms",
                          summary.visualFeatureTotalMs, frames);
}

void WriteReplaySummaryJsonOrb(std::ostream &json,
                               const ReplaySummary &summary)
{
    WriteJsonNumber(json, "orb_track_ms_mean", summary.orbTrackMsMean);
    WriteJsonNumber(json, "orb_track_ms_max", summary.orbTrackMsMax);
    WriteJsonNumber(json, "orb_extract_ms_mean", summary.orbExtractMsMean);
    WriteJsonNumber(json, "orb_extract_ms_max", summary.orbExtractMsMax);
    WriteJsonNumber(json, "orb_stereo_ms_mean", summary.orbStereoMsMean);
    WriteJsonNumber(json, "orb_stereo_ms_max", summary.orbStereoMsMax);
}

int WriteReplaySummaryJsonIfRequested(
    const OfflineReplayOptions &opts, const ReplaySummary &summary,
    const ReplayOutputAdjustment &adjustment)
{
    if (opts.summaryJson.empty()) {
        return 0;
    }
    std::ofstream json(opts.summaryJson);
    if (!json) {
        std::cerr << "failed to open summary json: " << opts.summaryJson << "\n";
        return 1;
    }
    json << "{\n";
    WriteReplaySummaryJsonConfig(json, opts, adjustment);
    WriteReplaySummaryJsonCounts(json, summary);
    WriteReplaySummaryJsonTiming(json, summary);
    WriteReplaySummaryJsonVisual(json, summary);
    WriteReplaySummaryJsonOrb(json, summary);
    WriteJsonString(json, "output_csv", opts.outputCsv.string());
    WriteJsonString(json, "final_euroc_trajectory",
                    opts.finalEurocTrajectory.string(), false);
    json << "}\n";
    json.flush();
    std::cout << "  summary_json: " << opts.summaryJson << "\n";
    return 0;
}

ReplayOutputAdjustment BuildReplayOutputAdjustment(
    const OfflineReplayOptions &opts)
{
    return {.timestampOffsetNs = EnvTimestampOffsetNs(),
            .positionScale = EnvOutputPositionScale(),
            .bodyFromCameraExtrinsics = LoadReplayBodyExtrinsics(opts)};
}

int RunOfflineReplay(const OfflineReplayOptions &opts)
{
    const SmartDrone::Tests::ReplayDataset dataset =
        SmartDrone::Tests::ReplayDataset::Load(opts.datasetRoot);
    if (dataset.Empty()) {
        std::cerr << "dataset is empty: " << opts.datasetRoot << "\n";
        return 1;
    }
    LogReplayStart(opts);
    if (!EnsureOrbSlamReplayAvailable(opts)) {
        return 2;
    }
    ReplayRuntime runtime(dataset);
    if (const int status = InitializeReplayRuntime(opts, runtime); status != 0) {
        return status;
    }
    PrepareReplayOutputDirectories(opts);
    std::ofstream realtimeCsv(opts.outputCsv);
    if (!realtimeCsv) {
        std::cerr << "failed to open output csv: " << opts.outputCsv << "\n";
        return 1;
    }
    WriteReplayCsvHeader(realtimeCsv);
    const ReplayOutputAdjustment adjustment = BuildReplayOutputAdjustment(opts);
    ReplayPoseSamples outputs =
        RunReplayFrames(opts, runtime, realtimeCsv, adjustment);
    realtimeCsv.flush();
    if (outputs.empty()) {
        return HandleEmptyReplayOutput(runtime);
    }
    WarnRealtimeLoopClosureDisabled(opts);
    const ReplaySummary summary = BuildReplaySummary(outputs);
    if (const int status = SaveFinalTrajectoryOrStop(opts, runtime); status != 0) {
        return status;
    }
    PrintReplaySummary(opts, adjustment, summary);
    if (const int status =
            WriteReplaySummaryJsonIfRequested(opts, summary, adjustment);
        status != 0) {
        return status;
    }
    StopReplayVisualFrontend(runtime);
    std::cout.flush();
    std::_Exit(0);
}
