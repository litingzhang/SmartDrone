#include "core/application/session/slam_session_runtime.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

#include "common/logger.h"
#include "common/tlv/tlv_protocol.h"
#include "core/application/session/runtime_session_common.h"

namespace smartdrone::core::application {

namespace {

std::string TrimCopy(const std::string &in)
{
    size_t begin = 0;
    while (begin < in.size() && std::isspace(static_cast<unsigned char>(in[begin])) != 0) {
        ++begin;
    }
    size_t end = in.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(in[end - 1])) != 0) {
        --end;
    }
    return in.substr(begin, end - begin);
}

bool SuperPointLightGlueInjectionEnabled()
{
    const char *value = std::getenv("SMART_DRONE_SUPERPOINT_LIGHTGLUE_INJECT");
    if (value == nullptr || value[0] == '\0') {
        return true;
    }
    const std::string text(value);
    return !(text == "0" || text == "false" || text == "FALSE" || text == "off" || text == "OFF");
}

bool EnvVarIsUnsetOrEmpty(const char *name)
{
    const char *value = std::getenv(name);
    return value == nullptr || value[0] == '\0';
}

void SetEnvIfUnset(const char *name, const char *value)
{
    if (!EnvVarIsUnsetOrEmpty(name)) {
        return;
    }
#if defined(_WIN32)
    _putenv_s(name, value);
#else
    setenv(name, value, 0);
#endif
}

void ConfigureSuperPointLightGlueRuntimeDefaults(const std::string &repo, int inputMaxWidth, int inputMaxHeight)
{
    SetEnvIfUnset("SMART_DRONE_FEATURE_PRECISION", "auto");
    SetEnvIfUnset("SMART_DRONE_LIGHTGLUE_LAYERS", "6");
    SetEnvIfUnset("SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT", "1");
    SetEnvIfUnset("SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_FEATURES", "72");
    SetEnvIfUnset("SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_POINTS", "24");
    SetEnvIfUnset("SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_RATIO", "0.30");
    SetEnvIfUnset("SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE", "0.965");
    SetEnvIfUnset("SMART_DRONE_SP_LG_INIT_TRUST_FRONTEND_PAIRS", "1");
    SetEnvIfUnset("SMART_DRONE_SP_LG_RECOVERY_TRUST_FRONTEND_PAIRS", "1");
    SetEnvIfUnset("SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_FRONTEND_PAIRS", "1");
    SetEnvIfUnset("SMART_DRONE_SP_LG_TRUST_FRONTEND_PAIRS_OK_STREAK", "120");
    SetEnvIfUnset("SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE", "1");
    SetEnvIfUnset("SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE_TIMEOUT_MS", "35");
    SetEnvIfUnset("SMART_DRONE_ORB_LIVE_EUROC_TRAJECTORY_POSE", "0");
    SetEnvIfUnset("SMART_DRONE_REALTIME_POSE_CONTINUITY", "1");
    if (!EnvVarIsUnsetOrEmpty("SMART_DRONE_SUPERPOINT_TRT_ENGINE") || repo.empty()) {
        return;
    }

    std::vector<std::string> engineNames;
    if (inputMaxWidth > 0 && inputMaxHeight > 0) {
        engineNames.push_back("superpoint_dense_" + std::to_string(inputMaxWidth) + "x" +
                              std::to_string(inputMaxHeight) + "_fp16.engine");
        engineNames.push_back("superpoint_dense_" + std::to_string(inputMaxWidth) + "x" +
                              std::to_string(inputMaxHeight) + "_fp32.engine");
    }
    engineNames.push_back("superpoint_dense_640x409_fp16.engine");
    engineNames.push_back("superpoint_dense_640x409_fp32.engine");
    engineNames.push_back("superpoint_dense_640x480_fp16.engine");
    engineNames.push_back("superpoint_dense_640x480_fp32.engine");

    for (const std::string &engineName : engineNames) {
        const fs::path engine = fs::path(repo) / "weights" / engineName;
        std::error_code ec;
        if (fs::exists(engine, ec)) {
            SetEnvIfUnset("SMART_DRONE_SUPERPOINT_TRT_ENGINE", engine.string().c_str());
            return;
        }
    }
}

std::string FirstExistingPathOrFallback(const std::vector<std::string> &candidates, const std::string &fallback)
{
    for (const std::string &candidate : candidates) {
        if (candidate.empty()) {
            continue;
        }
        std::error_code ec;
        if (fs::exists(candidate, ec)) {
            return candidate;
        }
    }
    return fallback;
}

std::string ResolveSuperPointLightGlueRepoForRuntime(const std::string &configuredRepo)
{
    const char *home = std::getenv("HOME");
    std::vector<std::string> candidates;
    if (home != nullptr) {
        candidates.push_back((fs::path(home) / "LightGlue").string());
        candidates.push_back((fs::path(home) / "lightglue").string());
        candidates.push_back((fs::path(home) / "third_party" / "LightGlue").string());
        candidates.push_back((fs::path(home) / "third_party" / "lightglue").string());
    }
    candidates.emplace_back("LightGlue");
    candidates.emplace_back("lightglue");
    candidates.emplace_back("third_party/LightGlue");
    candidates.emplace_back("third_party/lightglue");
    candidates.push_back(configuredRepo);
    return FirstExistingPathOrFallback(candidates, configuredRepo);
}

bool IsYamlKeyLine(const std::string &line, const std::string &key)
{
    const std::string trimmed = TrimCopy(line);
    if (trimmed.rfind(key, 0) != 0) {
        return false;
    }
    if (trimmed.size() <= key.size()) {
        return false;
    }
    return trimmed[key.size()] == ':';
}

void ReplaceOrInsertYamlScalar(std::string &text, const std::string &key, const std::string &value)
{
    std::vector<std::string> lines;
    {
        std::istringstream input(text);
        std::string line;
        while (std::getline(input, line)) {
            lines.push_back(line);
        }
    }

    const std::string replacement = key + ": " + value;
    bool replaced = false;
    for (std::string &line : lines) {
        if (IsYamlKeyLine(line, key)) {
            line = replacement;
            replaced = true;
            break;
        }
    }

    if (!replaced) {
        size_t insertAt = lines.size();
        for (size_t i = 0; i < lines.size(); ++i) {
            if (TrimCopy(lines[i]) == "...") {
                insertAt = i;
                break;
            }
        }
        lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(insertAt), replacement);
    }

    std::ostringstream output;
    for (const std::string &line : lines) {
        output << line << "\n";
    }
    text = output.str();
}

std::string BuildEffectiveSlamSettingsPath(const UnifiedConfig &cfg)
{
    if (cfg.app.runtime.orbNFeatures <= 0 || !(cfg.app.runtime.orbScaleFactor > 0.0f) ||
        cfg.app.runtime.orbNLevels <= 0 || cfg.app.runtime.orbIniThFAST <= 0 || cfg.app.runtime.orbMinThFAST <= 0) {
        return cfg.app.settings;
    }

    std::ifstream in(cfg.app.settings, std::ios::in);
    if (!in.is_open()) {
        std::cerr << "[slam] warning: failed to open settings for ORB override: " << cfg.app.settings << "\n";
        return cfg.app.settings;
    }

    std::ostringstream buffer;
    buffer << in.rdbuf();
    std::string content = buffer.str();
    if (content.empty()) {
        std::cerr << "[slam] warning: empty settings file for ORB override: " << cfg.app.settings << "\n";
        return cfg.app.settings;
    }

    ReplaceOrInsertYamlScalar(content, "ORBextractor.nFeatures", std::to_string(cfg.app.runtime.orbNFeatures));
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(6) << cfg.app.runtime.orbScaleFactor;
        ReplaceOrInsertYamlScalar(content, "ORBextractor.scaleFactor", ss.str());
    }
    ReplaceOrInsertYamlScalar(content, "ORBextractor.nLevels", std::to_string(cfg.app.runtime.orbNLevels));
    ReplaceOrInsertYamlScalar(content, "ORBextractor.iniThFAST", std::to_string(cfg.app.runtime.orbIniThFAST));
    ReplaceOrInsertYamlScalar(content, "ORBextractor.minThFAST", std::to_string(cfg.app.runtime.orbMinThFAST));

    const fs::path sourcePath(cfg.app.settings);
    const fs::path sourceDir = sourcePath.has_parent_path() ? sourcePath.parent_path() : fs::path(".");
    const fs::path sourceStem = sourcePath.stem();
    const fs::path targetPath = sourceDir / (sourceStem.string() + ".runtime_orb.yaml");

    std::ofstream out(targetPath.string(), std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        std::cerr << "[slam] warning: failed to write runtime ORB settings file: " << targetPath.string() << "\n";
        return cfg.app.settings;
    }
    out << content;
    out.close();
    if (!out) {
        std::cerr << "[slam] warning: failed to flush runtime ORB settings file: " << targetPath.string() << "\n";
        return cfg.app.settings;
    }

    std::cerr << "[slam] ORB settings override: nFeatures=" << cfg.app.runtime.orbNFeatures
              << " scaleFactor=" << cfg.app.runtime.orbScaleFactor << " nLevels=" << cfg.app.runtime.orbNLevels
              << " iniThFAST=" << cfg.app.runtime.orbIniThFAST << " minThFAST=" << cfg.app.runtime.orbMinThFAST
              << " file=" << targetPath.string() << "\n";
    return targetPath.string();
}

std::atomic<uint32_t> g_slamSessionResetCounter{0};
std::atomic<uint32_t> g_slamSessionResetMapCount{0};

ORB_SLAM3::System::eSensor ResolveOrbSensor(const MainRuntimeAliases &aliases)
{
    const bool monoMode = aliases.sensorMode == SensorMode::Mono || aliases.sensorMode == SensorMode::MonoImu;
    const bool monoImuMode = aliases.sensorMode == SensorMode::MonoImu;
    if (monoImuMode)
        return ORB_SLAM3::System::IMU_MONOCULAR;
    if (monoMode)
        return ORB_SLAM3::System::MONOCULAR;
    if (aliases.sensorMode == SensorMode::StereoImu)
        return ORB_SLAM3::System::IMU_STEREO;
    return ORB_SLAM3::System::STEREO;
}

smartdrone::adapters::imu::Icm42688ImuProviderConfig MakeImuProviderConfig(const MainRuntimeAliases &aliases)
{
    const int64_t imuDtNs = 1000000000LL / std::max(1, aliases.imuHz);
    const int64_t slackBeforeNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    const int64_t slackAfterNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    return {slackBeforeNs, slackAfterNs};
}

SlamFrameProcessor::State MakeInitialFrameProcessorState(const MainRuntimeAliases &aliases)
{
    const auto requestedSlamMode = aliases.slamOperationMode;
    const auto effectiveSlamMode = requestedSlamMode == smartdrone::core::domain::SlamOperationMode::Auto
                                       ? smartdrone::core::domain::SlamOperationMode::Mapping
                                       : requestedSlamMode;
    const uint32_t sessionResetCounter = g_slamSessionResetCounter.fetch_add(1, std::memory_order_relaxed) + 1;
    const uint32_t sessionResetMapCount = g_slamSessionResetMapCount.fetch_add(1, std::memory_order_relaxed) + 1;
    SlamFrameProcessor::State state;
    state.imuWarmupSamples = static_cast<uint64_t>(std::max(20, aliases.imuHz / 2));
    state.sessionResetCounterBase = static_cast<uint8_t>(sessionResetCounter & 0xFFu);
    state.sessionResetMapCountBase = static_cast<uint16_t>(sessionResetMapCount & 0xFFFFu);
    state.requestedSlamMode = requestedSlamMode;
    state.effectiveSlamMode = effectiveSlamMode;
    return state;
}

void ApplyOrbAccelerationEnvironment(const std::string &acceleration)
{
    std::string normalized = acceleration;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized == "cuda" || normalized == "gpu" || normalized == "opencv_cuda" || normalized == "opencv-cuda") {
#if defined(_WIN32)
        _putenv_s("SMART_DRONE_ORB_ACCEL", "cuda");
        _putenv_s("SMART_DRONE_ORB_VPI_REMAP", "");
        _putenv_s("SMART_DRONE_ORB_CUDA_PYRAMID", "");
#else
        setenv("SMART_DRONE_ORB_ACCEL", "cuda", 1);
        unsetenv("SMART_DRONE_ORB_VPI_REMAP");
        unsetenv("SMART_DRONE_ORB_CUDA_PYRAMID");
#endif
    } else if (normalized == "vpi" || normalized == "vpi_remap" || normalized == "vpi-remap" ||
               normalized == "vpi_cuda_remap" || normalized == "vpi-cuda-remap") {
#if defined(_WIN32)
        _putenv_s("SMART_DRONE_ORB_ACCEL", "");
        _putenv_s("SMART_DRONE_ORB_VPI_REMAP", "1");
        _putenv_s("SMART_DRONE_ORB_CUDA_PYRAMID", "");
#else
        unsetenv("SMART_DRONE_ORB_ACCEL");
        setenv("SMART_DRONE_ORB_VPI_REMAP", "1", 1);
        unsetenv("SMART_DRONE_ORB_CUDA_PYRAMID");
#endif
    } else {
#if defined(_WIN32)
        _putenv_s("SMART_DRONE_ORB_ACCEL", "");
        _putenv_s("SMART_DRONE_ORB_VPI_REMAP", "");
        _putenv_s("SMART_DRONE_ORB_CUDA_PYRAMID", "");
#else
        unsetenv("SMART_DRONE_ORB_ACCEL");
        unsetenv("SMART_DRONE_ORB_VPI_REMAP");
        unsetenv("SMART_DRONE_ORB_CUDA_PYRAMID");
#endif
    }
}

} // namespace

SlamSessionRuntime::SlamSessionRuntime(const UnifiedConfig &cfg, LiveRuntimeTuning &tuning, Px4MavlinkGateway &mav,
                                       LivePoseState &livePose, std::atomic<bool> &stop, std::atomic<bool> &runningFlag)
    : m_cfg(cfg), m_tuning(tuning), m_mav(mav), m_livePose(livePose), m_stop(stop), m_runningFlag(runningFlag),
      m_aliases(BuildRuntimeAliases(cfg.app)),
      m_monoMode(m_aliases.sensorMode == SensorMode::Mono || m_aliases.sensorMode == SensorMode::MonoImu),
      m_useImu(m_aliases.sensorMode == SensorMode::StereoImu || m_aliases.sensorMode == SensorMode::MonoImu),
      m_orbSensor(ResolveOrbSensor(m_aliases)),
      m_slamInputMode(m_monoMode ? smartdrone::adapters::slam::SlamInputMode::MonoRight
                                  : smartdrone::adapters::slam::SlamInputMode::Stereo),
      m_effectiveSettingsPath(BuildEffectiveSlamSettingsPath(cfg)),
      m_cameraProvider(CreateCameraProvider()),
      m_imuProvider(m_imuState.imuBuffer, MakeImuProviderConfig(m_aliases)), m_posePublisher(mav),
      m_perceptionPipeline(PerceptionPipelineConfig{m_aliases.fps, true}),
      m_frameProcessorState(MakeInitialFrameProcessorState(m_aliases))
{
    if (m_aliases.slamBackend == SlamBackend::DpvoTensorRt) {
        m_slamEngine = std::make_unique<smartdrone::adapters::slam::DpvoTensorRtEngine>(
            smartdrone::adapters::slam::MakeDpvoTensorRtConfig(cfg.app.runtime));
    } else {
        ApplyOrbAccelerationEnvironment(cfg.app.runtime.orbAcceleration);
        m_slamSystem = std::make_unique<ORB_SLAM3::System>(cfg.app.vocab, m_effectiveSettingsPath, m_orbSensor, false);
        auto orbEngine = std::make_unique<smartdrone::adapters::slam::SlamEngineAdapter>(
            std::move(m_slamSystem), m_slamInputMode, m_useImu, m_effectiveSettingsPath);
        m_orbSlamEngine = orbEngine.get();
        m_slamEngine = std::move(orbEngine);
    }
}

bool SlamSessionRuntime::Start()
{
    PrintStartupConfig(m_cfg.app, m_aliases, ControllerMode::Slam);
    m_livePose.SetRuntimeMode(RUNTIME_MODE_SLAM);
    Logger::Init("./stereo_vslam.log", 32 * 1024 * 1024, Logger::INFO, true);
    if (!m_slamEngine || !m_slamEngine->Start()) {
        m_stop.store(true);
        CleanupAfterStartFailure();
        return false;
    }
    m_slamStarted = true;

    if (m_orbSlamEngine != nullptr) {
        m_orbSlamEngine->SetOperationMode(m_frameProcessorState.effectiveSlamMode);
        m_orbSlamEngine->SetFeatureFrontend(m_aliases.featureFrontend);
        m_orbSlamEngine->SetExternalFeatureFrontendClient(&m_superpointFrontendClient);
        m_orbSlamEngine->SetExternalFeatureInputSizeLimit(m_cfg.app.runtime.superpointInputMaxWidth,
                                                          m_cfg.app.runtime.superpointInputMaxHeight);
        m_orbSlamEngine->SetStereoVoLoopClosure(m_cfg.app.runtime.lkLoopClosure, m_cfg.app.runtime.lkLoopScale,
                                                m_cfg.app.runtime.lkLoopRelaxation);
        m_orbSlamEngine->SetStereoVoPerFrameAcceleration(m_cfg.app.runtime.lkPerFrameAcceleration);
    }
    if (m_frameProcessorState.requestedSlamMode == smartdrone::core::domain::SlamOperationMode::Auto) {
        std::cerr << "[slam] operation_mode=auto effective_mode=mapping\n";
    }
    m_livePose.SetSlamMode(ToRuntimeSlamModeValue(m_frameProcessorState.effectiveSlamMode));
    if (m_frameProcessorState.requestedSlamMode == smartdrone::core::domain::SlamOperationMode::Relocalization ||
        m_frameProcessorState.requestedSlamMode == smartdrone::core::domain::SlamOperationMode::TrackingOnly) {
        std::cerr << "[slam] note: slam_mode=" << smartdrone::core::domain::ToString(m_aliases.slamOperationMode)
                  << " currently maps to ORB-SLAM3 localization-only mode\n";
    }

    if (m_aliases.sensorMode == SensorMode::Stereo) {
        m_stereoBodyExtrinsics = LoadStereoBodyExtrinsics(m_effectiveSettingsPath);
    }

    const bool needsSuperPointTensorRt =
        m_aliases.featureFrontend == FeatureFrontend::SuperPointLightGlue && SuperPointLightGlueInjectionEnabled();
    std::string featureRepo = m_cfg.app.runtime.superpointRepo;
    if (m_aliases.featureFrontend == FeatureFrontend::SuperPointLightGlue) {
        featureRepo = ResolveSuperPointLightGlueRepoForRuntime(featureRepo);
        ConfigureSuperPointLightGlueRuntimeDefaults(featureRepo,
                                                    m_cfg.app.runtime.superpointInputMaxWidth,
                                                    m_cfg.app.runtime.superpointInputMaxHeight);
    }
    if (needsSuperPointTensorRt) {
        std::string featureErr;
        if (m_superpointFrontendClient.Start(featureRepo, m_cfg.app.runtime.superpointDevice, m_cfg.app.runtime.superpointTopK,
                                        m_cfg.app.runtime.superpointMaxPoints, &featureErr)) {
            std::cerr << "[slam] superpoint TensorRT ready repo=" << featureRepo
                      << " device=" << m_cfg.app.runtime.superpointDevice
                      << " top_k=" << m_cfg.app.runtime.superpointTopK
                      << " max_points=" << m_cfg.app.runtime.superpointMaxPoints << "\n";
        } else {
            std::cerr << "[slam] warning: SuperPoint TensorRT start failed: " << featureErr << "\n";
        }
    }
    if (m_aliases.featureFrontend == FeatureFrontend::LK) {
        std::cerr << "[slam] feature_frontend=lk selected; lk_grid=48\n";
    }
    if (m_aliases.udpEnable) {
        auto destinationResolver = [this](sockaddr_in &dst) {
            LivePoseState::Snapshot snapshot{};
            if (!m_livePose.ReadSnapshot(snapshot) || !snapshot.hasPeer || !snapshot.peer.valid) {
                return false;
            }
            dst = snapshot.peer.addr;
            return true;
        };
        if (!m_udp.Open(m_aliases.udpIp, m_aliases.udpPort, m_aliases.udpJpegQ, m_aliases.udpPayload,
                        m_aliases.udpQueue, std::move(destinationResolver))) {
            std::cerr << "[session] slam udp open failed\n";
            m_stop.store(true);
            CleanupAfterStartFailure();
            return false;
        }
        m_udpOpen = true;
    }
    if (m_useImu) {
        m_imuThread = StartImuThread(m_aliases, m_imuState, m_stop, m_runningFlag);
    }
    const bool cameraOpened = m_cameraProvider && m_cameraProvider->Open(m_aliases);
    if (!cameraOpened) {
        m_stop.store(true);
        CleanupAfterStartFailure();
        return false;
    }
    m_cameraOpen = true;

    m_mav.SetFrameTimingTracker(&m_frameTimingTracker);
    return true;
}

void SlamSessionRuntime::Stop()
{
    m_mav.SetFrameTimingTracker(nullptr);
    if (m_cameraOpen) {
        m_cameraProvider->Close();
        m_cameraOpen = false;
        std::cerr << "[session] slam camera closed\n";
    }
    if (m_imuThread.joinable())
        m_imuThread.join();
    std::cerr << "[session] slam imu joined\n";
    if (m_udpOpen) {
        m_udp.Close();
        m_udpOpen = false;
        std::cerr << "[session] slam udp closed\n";
    }
    m_mav.StopSetpointStream();
    std::cerr << "[session] slam setpoint stopped\n";
    if (m_slamStarted) {
        m_slamEngine->Stop();
        m_slamStarted = false;
    }
    m_superpointFrontendClient.Stop();
    std::cerr << "[session] slam shutdown complete\n";
    m_livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
    std::cerr << "[session] slam exit\n";
}

bool SlamSessionRuntime::WaitForImuReady() const
{
    if (!m_useImu)
        return true;
    const uint64_t imuCnt = m_imuState.imuCnt.load(std::memory_order_relaxed);
    const bool imuReady =
        m_imuState.imuOk.load(std::memory_order_relaxed) && imuCnt >= m_frameProcessorState.imuWarmupSamples;
    if (imuReady)
        return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return false;
}

SlamFrameProcessor &SlamSessionRuntime::FrameProcessor()
{
    if (!m_frameProcessorContext) {
        m_frameProcessorContext = std::make_unique<SlamFrameProcessor::Context>(SlamFrameProcessor::Context{
            m_aliases, m_monoMode, m_useImu, m_tuning, m_livePose, m_mav, *m_slamEngine, m_orbSlamEngine,
            &m_superpointFrontendClient, *m_cameraProvider, m_imuProvider, m_posePublisher, m_udp,
            m_frameTimingTracker, m_perceptionPipeline, m_posePostprocessor, m_autoSlamModeController,
            m_stereoBodyExtrinsics});
    }
    if (!m_frameProcessor) {
        m_frameProcessor = std::make_unique<SlamFrameProcessor>(*m_frameProcessorContext, m_frameProcessorState);
    }
    return *m_frameProcessor;
}

void SlamSessionRuntime::CleanupAfterStartFailure()
{
    m_mav.SetFrameTimingTracker(nullptr);
    if (m_cameraOpen) {
        m_cameraProvider->Close();
        m_cameraOpen = false;
    }
    if (m_imuThread.joinable()) {
        m_imuThread.join();
    }
    if (m_udpOpen) {
        m_udp.Close();
        m_udpOpen = false;
    }
    m_superpointFrontendClient.Stop();
    m_mav.StopSetpointStream();
    if (m_slamStarted) {
        m_slamEngine->Stop();
        m_slamStarted = false;
    }
    m_livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
}

} // namespace smartdrone::core::application
