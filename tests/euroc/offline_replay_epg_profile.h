using ReplayApplicationFactories = SmartDrone::Core::Application::ApplicationRuntimeFactories;
using ReplayCameraOpenConfig = SmartDrone::Core::Ports::CameraOpenConfig;
using ReplayImuProviderConfig =
    SmartDrone::Adapters::Imu::Icm42688ImuProviderConfig;
using ReplayMainRuntimeAliases =
    SmartDrone::Core::Application::MainRuntimeAliases;
using ReplaySlamResourceConfig =
    SmartDrone::Core::Application::SlamSessionEngineResourceConfig;
using ReplaySlamResources =
    SmartDrone::Core::Application::SlamSessionEngineResources;
using ReplayUnifiedConfig =
    SmartDrone::Core::Application::UnifiedConfig;

class ReplayNoopPreviewOutputPort final
    : public SmartDrone::Core::Application::IPreviewOutputPort {
  public:
    void Enqueue(const SmartDrone::Core::Application::PreviewOutputFrame &)
        override
    {
    }

    void StepOnce() override
    {
    }

    void StepAll() override
    {
    }
};

class ReplayNoopPreviewOutputRuntime final
    : public SmartDrone::Core::Application::IPreviewOutputRuntime {
  public:
    bool Open(const SmartDrone::Core::Application::PreviewOutputOpenConfig &,
              DestinationResolver) override
    {
        return true;
    }

    bool OpenStaticPeer(
        const SmartDrone::Core::Application::PreviewOutputOpenConfig &)
        override
    {
        return true;
    }

    void Close() override
    {
    }

    void EnqueueCalibStereoFrame(
        const SmartDrone::Core::Application::CalibStereoFrame &) override
    {
    }

    SmartDrone::Core::Application::IPreviewOutputPort &OutputPort() override
    {
        return m_port;
    }

  private:
    ReplayNoopPreviewOutputPort m_port;
};

class ReplayNoopSlamTelemetry final
    : public SmartDrone::Core::Ports::ISlamSessionTelemetryPort {
  public:
    void SetFrameTimingTracker(SmartDrone::Core::Ports::IFrameTimingTracker *)
        override
    {
    }

    bool GetDownwardRange(SmartDrone::Core::Ports::SlamRangeSensor &,
                          uint64_t) const override
    {
        return false;
    }

    void StopSetpointStream() override
    {
    }
};

class ReplayEpgPoseCsvPublisher final
    : public SmartDrone::Core::Ports::IPosePublisher {
  public:
    ReplayEpgPoseCsvPublisher(
        const SmartDrone::Tests::ReplayDataset &dataset,
        const OfflineReplayOptions &opts, ReplayOutputAdjustment adjustment)
        : m_dataset(dataset), m_opts(opts), m_adjustment(std::move(adjustment))
    {
    }

    bool Open()
    {
        CreateDirectoryForPath(m_opts.outputCsv);
        m_csv.open(m_opts.outputCsv);
        if (!m_csv) {
            std::cerr << "failed to open output csv: " << m_opts.outputCsv
                      << "\n";
            return false;
        }
        WriteReplayCsvHeader(m_csv);
        return true;
    }

    void PublishPose(
        const SmartDrone::Core::Ports::PosePublishRequest &request) override
    {
        auto sample = BuildSample(request);
        sample = AdjustReplayOutputSample(
            sample, m_adjustment.timestampOffsetNs, m_adjustment.positionScale,
            m_adjustment.bodyFromCameraExtrinsics);
        WriteReplayCsvSample(m_csv, sample);
        m_csv.flush();
        m_outputs.push_back(sample);
    }

    size_t OutputCount() const
    {
        return m_outputs.size();
    }

  private:
    int64_t TimestampForFrame(uint64_t frameId) const
    {
        if (frameId == 0 || frameId > m_dataset.LeftFrames().size()) {
            return 0;
        }
        const auto index = static_cast<size_t>(frameId - 1);
        const uint64_t left = m_dataset.LeftFrames()[index].timestampNs;
        const uint64_t right = m_dataset.RightFrames()[index].timestampNs;
        return static_cast<int64_t>(std::min(left, right) +
                                    ((std::max(left, right) -
                                      std::min(left, right)) /
                                     2ULL));
    }

    SmartDrone::Tests::ReplayPoseSample BuildSample(
        const SmartDrone::Core::Ports::PosePublishRequest &request) const
    {
        SmartDrone::Tests::ReplayPoseSample sample{};
        sample.frameId = request.frameId;
        sample.captureTimestampNs = TimestampForFrame(request.frameId);
        sample.trackingState = request.trackingState;
        sample.poseValid = request.pose.valid;
        sample.pose = request.pose;
        return sample;
    }

    const SmartDrone::Tests::ReplayDataset &m_dataset;
    const OfflineReplayOptions &m_opts;
    ReplayOutputAdjustment m_adjustment;
    std::ofstream m_csv;
    ReplayPoseSamples m_outputs;
};

class ReplayVisualFrontendSession final
    : public SmartDrone::Core::Application::ISlamVisualFeatureFrontendSession {
  public:
    explicit ReplayVisualFrontendSession(
        std::unique_ptr<ReplayVisualFrontend> client)
        : m_client(std::move(client))
    {
    }

    void Stop() override
    {
        if (m_client != nullptr) {
            m_client->Stop();
        }
    }

    SmartDrone::Core::Ports::IVisualFeatureFrontend *Frontend()
    {
        return m_client.get();
    }

  private:
    std::unique_ptr<ReplayVisualFrontend> m_client;
};

ReplayUnifiedConfig BuildReplayUnifiedConfig(const OfflineReplayOptions &opts)
{
    ReplayUnifiedConfig config{};
    config.app.vocab = opts.vocab;
    config.app.settings = opts.settings;
    config.app.sensorMode = opts.sensorMode;
    config.app.camera.fps = opts.cameraFps;
    config.app.camera.width = opts.visualFeatureInputMaxWidth;
    config.app.camera.height = opts.visualFeatureInputMaxHeight;
    config.app.udp.enable = false;
    config.app.udp.sendImage = false;
    config.app.udp.sendFeature = false;
    config.app.udp.sendMap = false;
    config.app.runtime = BuildReplayRuntimeConfig(opts);
    config.app.runtime.slamInputFps = opts.slamInputFps;
    return config;
}

SmartDrone::Adapters::Slam::SlamInputMode
ResolveReplaySlamInputMode(const ReplayMainRuntimeAliases &aliases)
{
    return (aliases.sensorMode == SensorMode::Mono ||
            aliases.sensorMode == SensorMode::MonoImu)
               ? SmartDrone::Adapters::Slam::SlamInputMode::MonoRight
               : SmartDrone::Adapters::Slam::SlamInputMode::Stereo;
}

SmartDrone::Adapters::Slam::SlamEngineFactoryConfig
BuildEpgReplayEngineConfig(const ReplaySlamResourceConfig &config)
{
    SmartDrone::Adapters::Slam::SlamEngineFactoryConfig engineConfig{};
    engineConfig.backend = config.aliases.slamBackend;
    engineConfig.vocabularyPath = config.cfg.app.vocab;
    engineConfig.settingsPath = config.settingsPath;
    engineConfig.sensorMode = config.aliases.sensorMode;
    engineConfig.useImu = config.useImu;
    engineConfig.inputMode = ResolveReplaySlamInputMode(config.aliases);
    engineConfig.dpvoRuntime.repoPath = config.cfg.app.runtime.dpvoRepo;
    engineConfig.dpvoRuntime.patchEnginePath =
        config.cfg.app.runtime.dpvoPatchEngine;
    engineConfig.dpvoRuntime.updateEnginePath =
        config.cfg.app.runtime.dpvoUpdateEngine;
    engineConfig.dpvoRuntime.inputWidth =
        config.cfg.app.runtime.dpvoInputWidth;
    engineConfig.dpvoRuntime.inputHeight =
        config.cfg.app.runtime.dpvoInputHeight;
    engineConfig.dpvoRuntime.patchesPerFrame =
        config.cfg.app.runtime.dpvoPatchesPerFrame;
    engineConfig.dpvoRuntime.optimizationWindow =
        config.cfg.app.runtime.dpvoOptimizationWindow;
    return engineConfig;
}

ReplaySlamResources CreateEpgReplaySlamResources(
    const ReplaySlamResourceConfig &config)
{
    SmartDrone::Adapters::Slam::ControlledSlamEngine slamEngine =
        SmartDrone::Adapters::Slam::CreateSlamEngine(
            BuildEpgReplayEngineConfig(config));
    ReplaySlamResources resources{};
    resources.control = std::make_unique<
        SmartDrone::Core::Application::SlamRuntimeControlPort>(
        slamEngine.control);
    resources.backendMaintenance = slamEngine.backendMaintenance;
    resources.engine = std::move(slamEngine.engine);
    return resources;
}

SmartDrone::Adapters::Slam::VisualFeatureFrontendRuntimeConfig
BuildEpgReplayVisualFrontendConfig(const ReplayMainRuntimeAliases &aliases,
                                   const ReplayUnifiedConfig &cfg)
{
    SmartDrone::Adapters::Slam::VisualFeatureFrontendRuntimeConfig config{};
    config.repoPath =
        SmartDrone::Adapters::Slam::ResolveVisualFeatureFrontendRepo(
            aliases.featureFrontend, cfg.app.runtime.visualFeatureRepo);
    config.device = cfg.app.runtime.visualFeatureDevice;
    config.topK = cfg.app.runtime.visualFeatureTopK;
    config.maxPoints = cfg.app.runtime.visualFeatureMaxPoints;
    config.inputMaxWidth = cfg.app.runtime.visualFeatureInputMaxWidth;
    config.inputMaxHeight = cfg.app.runtime.visualFeatureInputMaxHeight;
    SmartDrone::Adapters::Slam::ConfigureVisualFeatureFrontendDefaults(
        aliases.featureFrontend, config);
    return config;
}

SmartDrone::Core::Application::SlamVisualFeatureFrontendStartResult
StartEpgReplayVisualFrontendSession(const ReplayMainRuntimeAliases &aliases,
                                    const ReplayUnifiedConfig &cfg)
{
    SmartDrone::Core::Application::SlamVisualFeatureFrontendStartResult result{};
    if (!IsVisualFeatureLightGlueFrontend(aliases.featureFrontend)) {
        return result;
    }
    result.routeAvailable = true;
    auto featureConfig = BuildEpgReplayVisualFrontendConfig(aliases, cfg);
    result.repoPath = featureConfig.repoPath;
    auto client = SmartDrone::Adapters::Slam::CreateVisualFeatureFrontendClient(
        aliases.featureFrontend);
    if (client == nullptr) {
        result.clientMissing = true;
        return result;
    }
    if (!client->Start(featureConfig, &result.error)) {
        return result;
    }
    auto session =
        std::make_unique<ReplayVisualFrontendSession>(std::move(client));
    result.frontend = session->Frontend();
    result.session = std::move(session);
    result.started = true;
    return result;
}

void ConfigureEpgReplayTuning(
    const OfflineReplayOptions &opts,
    SmartDrone::Core::Application::LiveRuntimeTuning &tuning)
{
    tuning.slamInputFps.store(opts.slamInputFps);
    tuning.slamOperationMode.store(static_cast<uint8_t>(opts.slamMode));
    tuning.featureFrontend.store(static_cast<uint8_t>(opts.featureFrontend));
    tuning.sendImage.store(false);
    tuning.sendFeature.store(false);
    tuning.sendMap.store(false);
}

ReplayCameraOpenConfig BuildEpgReplayCameraOpenConfig(
    const ReplayMainRuntimeAliases &aliases)
{
    ReplayCameraOpenConfig config{};
    config.width = aliases.width;
    config.height = aliases.height;
    config.fps = aliases.fps;
    return config;
}

ReplayImuProviderConfig MakeEpgReplayImuProviderConfig(
    const ReplayMainRuntimeAliases &aliases)
{
    const int64_t imuDtNs = 1000000000LL / std::max(1, aliases.imuHz);
    const int64_t slackBeforeNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    const int64_t slackAfterNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    return {slackBeforeNs, slackAfterNs};
}

ReplayApplicationFactories BuildEpgReplayFactories(
    const SmartDrone::Tests::ReplayDataset &dataset,
    std::shared_ptr<SmartDrone::Tests::ReplayCameraProgress> progress)
{
    ReplayApplicationFactories factories{};
    factories.createCameraProvider = [&dataset, progress]() {
        return std::make_unique<SmartDrone::Tests::ReplayCameraProvider>(
            dataset, progress);
    };
    factories.makeCameraOpenConfig = [](const ReplayMainRuntimeAliases &aliases) {
        return BuildEpgReplayCameraOpenConfig(aliases);
    };
    factories.createSlamEngineResources =
        [](const ReplaySlamResourceConfig &config) {
            return CreateEpgReplaySlamResources(config);
        };
    factories.createImuProvider =
        [](SmartDrone::Core::Application::ImuThreadState &state,
           const SmartDrone::Core::Application::MainRuntimeAliases &aliases) {
            return std::make_unique<SmartDrone::Adapters::Imu::Icm42688ImuProvider>(
                state.imuBuffer, MakeEpgReplayImuProviderConfig(aliases));
        };
    factories.startVisualFeatureFrontendSession =
        [](const ReplayMainRuntimeAliases &aliases,
           const ReplayUnifiedConfig &cfg) {
            return StartEpgReplayVisualFrontendSession(aliases, cfg);
        };
    factories.createPreviewOutputRuntime =
        []() { return std::make_unique<ReplayNoopPreviewOutputRuntime>(); };
    factories.cameraProvider = {"euroc_replay", false};
    return factories;
}

bool EpgReplayFinished(
    const SmartDrone::Tests::ReplayCameraProgress &progress,
    std::chrono::steady_clock::time_point finishedAt,
    int drainMs)
{
    if (!progress.finished.load()) {
        return false;
    }
    if (finishedAt == std::chrono::steady_clock::time_point{}) {
        return false;
    }
    const auto elapsed = std::chrono::steady_clock::now() - finishedAt;
    return elapsed >= std::chrono::milliseconds(std::max(0, drainMs));
}

void StepEpgReplayRuntime(
    SmartDrone::Core::Application::SlamSessionGraphRuntime &runtime,
    const SmartDrone::Tests::ReplayCameraProgress &progress,
    const OfflineReplayOptions &opts)
{
    std::chrono::steady_clock::time_point finishedAt{};
    while (runtime.Ok()) {
        runtime.Step();
        if (progress.finished.load() &&
            finishedAt == std::chrono::steady_clock::time_point{}) {
            finishedAt = std::chrono::steady_clock::now();
        }
        if (EpgReplayFinished(progress, finishedAt, opts.epgDrainMs)) {
            return;
        }
        usleep(1000);
    }
}

void StopEpgReplayRuntime(
    SmartDrone::Core::Application::SlamSessionGraphRuntime &runtime,
    std::atomic<bool> &runningFlag)
{
    runningFlag.store(false);
    runtime.RequestStop();
    while (!runtime.Done()) {
        runtime.Step();
        usleep(1000);
    }
    runtime.Stop();
}

fs::path DefaultEpgOptimizedPath()
{
    return fs::path("output") / "epg" / "optimized_slam_session_graph.json";
}

fs::path DefaultEpgSolverReportPath()
{
    return fs::path("output") / "epg" /
           "optimized_slam_session_graph_report.json";
}

int CopyEpgReplayProfile(const OfflineReplayOptions &opts)
{
    if (opts.epgProfileOut.empty()) {
        return 0;
    }
    const fs::path source{"/tmp/smartdrone_epg_slam_profile.json"};
    if (!fs::exists(source)) {
        std::cerr << "EPG replay did not produce profile: " << source << "\n";
        return 1;
    }
    CreateDirectoryForPath(opts.epgProfileOut);
    fs::copy_file(source, opts.epgProfileOut,
                  fs::copy_options::overwrite_existing);
    return 0;
}

int OptimizeEpgReplayProfileIfRequested(const OfflineReplayOptions &opts)
{
    if (opts.epgOptimizedOut.empty()) {
        return 0;
    }
    const auto result =
        SmartDrone::Core::Application::OptimizeEpgProfileForManifest(
            SmartDrone::Core::Application::EpgManifestForDomain(
                SmartDrone::Core::Application::EpgDomain::SlamSession),
            SmartDrone::Core::Application::EpgDfxNowMs());
    if (!result.optimized) {
        std::cerr << "EPG topology optimize failed: " << result.message
                  << "\n";
        return 1;
    }
    CreateDirectoryForPath(opts.epgOptimizedOut);
    fs::copy_file(DefaultEpgOptimizedPath(), opts.epgOptimizedOut,
                  fs::copy_options::overwrite_existing);
    if (!opts.epgSolverReportOut.empty() &&
        fs::exists(DefaultEpgSolverReportPath())) {
        CreateDirectoryForPath(opts.epgSolverReportOut);
        fs::copy_file(DefaultEpgSolverReportPath(), opts.epgSolverReportOut,
                      fs::copy_options::overwrite_existing);
    }
    return 0;
}

int RunEpgProfileReplay(const OfflineReplayOptions &opts)
{
    auto progress =
        std::make_shared<SmartDrone::Tests::ReplayCameraProgress>();
    const SmartDrone::Tests::ReplayDataset dataset =
        SmartDrone::Tests::ReplayDataset::Load(opts.datasetRoot, opts.maxFrames);
    SmartDrone::Tests::ReplayImuSampleSourceScope replayImuSource(dataset);
    ReplayUnifiedConfig cfg = BuildReplayUnifiedConfig(opts);
    SmartDrone::Core::Application::LiveRuntimeTuning tuning;
    ConfigureEpgReplayTuning(opts, tuning);
    ReplayNoopSlamTelemetry telemetry;
    const ReplayOutputAdjustment outputAdjustment =
        BuildReplayOutputAdjustment(opts);
    ReplayEpgPoseCsvPublisher posePublisher(dataset, opts, outputAdjustment);
    if (!posePublisher.Open()) {
        return 1;
    }
    std::atomic<bool> stop{false};
    std::atomic<bool> runningFlag{true};
    SmartDrone::Core::Application::LivePoseState livePose;
    ReplayApplicationFactories factories =
        BuildEpgReplayFactories(dataset, progress);
    SmartDrone::Core::Application::SlamSessionGraphRuntime runtime({
        cfg, tuning, telemetry, posePublisher, stop, livePose, runningFlag,
        factories});
    if (!runtime.Start()) {
        return 1;
    }
    StepEpgReplayRuntime(runtime, *progress, opts);
    StopEpgReplayRuntime(runtime, runningFlag);
    if (posePublisher.OutputCount() == 0) {
        std::cerr << "EPG replay did not publish poses\n";
        return 3;
    }
    if (const int status = CopyEpgReplayProfile(opts); status != 0) {
        return status;
    }
    return OptimizeEpgReplayProfileIfRequested(opts);
}
