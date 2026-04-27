#include "core/application/session/slam_session_runtime.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstddef>
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

} // namespace

SlamSessionRuntime::SlamSessionRuntime(const UnifiedConfig &cfg, LiveRuntimeTuning &tuning, Px4MavlinkGateway &mav,
                                       LivePoseState &livePose, std::atomic<bool> &stop, std::atomic<bool> &runningFlag)
    : m_cfg(cfg), m_tuning(tuning), m_mav(mav), m_livePose(livePose), m_stop(stop), m_runningFlag(runningFlag),
      m_aliases(BuildRuntimeAliases(cfg.app)),
      m_monoMode(m_aliases.sensorMode == SensorMode::Mono || m_aliases.sensorMode == SensorMode::MonoImu),
      m_useImu(m_aliases.sensorMode == SensorMode::StereoImu || m_aliases.sensorMode == SensorMode::MonoImu),
      m_orbSensor(ResolveOrbSensor(m_aliases)),
      m_orbInputMode(m_monoMode ? smartdrone::adapters::slam::OrbInputMode::MonoRight
                                : smartdrone::adapters::slam::OrbInputMode::Stereo),
      m_effectiveSettingsPath(BuildEffectiveSlamSettingsPath(cfg)),
      m_slamSystem(std::make_unique<ORB_SLAM3::System>(cfg.app.vocab, m_effectiveSettingsPath, m_orbSensor, false)),
      m_slamEngine(std::move(m_slamSystem), m_orbInputMode, m_useImu, m_effectiveSettingsPath),
      m_cameraProvider(CreateCameraProvider()),
      m_imuProvider(m_imuState.imuBuffer, MakeImuProviderConfig(m_aliases)), m_posePublisher(mav),
      m_perceptionPipeline(PerceptionPipelineConfig{m_aliases.fps, true}),
      m_frameProcessorState(MakeInitialFrameProcessorState(m_aliases))
{
}

bool SlamSessionRuntime::Start()
{
    PrintStartupConfig(m_cfg.app, m_aliases, ControllerMode::Slam);
    m_livePose.SetRuntimeMode(RUNTIME_MODE_SLAM);
    Logger::Init("./stereo_vslam.log", 32 * 1024 * 1024, Logger::INFO, true);
    if (!m_slamEngine.Start()) {
        m_stop.store(true);
        CleanupAfterStartFailure();
        return false;
    }
    m_slamStarted = true;

    m_slamEngine.SetOperationMode(m_frameProcessorState.effectiveSlamMode);
    m_slamEngine.SetFeatureFrontend(m_aliases.featureFrontend);
    m_slamEngine.SetXFeatFrontendClient(&m_xfeatFrontendClient);
    m_slamEngine.SetXFeatInputSizeLimit(m_cfg.app.runtime.xfeatInputMaxWidth, m_cfg.app.runtime.xfeatInputMaxHeight);
    m_slamEngine.SetLkLoopClosure(m_cfg.app.runtime.lkLoopClosure, m_cfg.app.runtime.lkLoopScale,
                                  m_cfg.app.runtime.lkLoopRelaxation);
    m_slamEngine.SetLkPerFrameAcceleration(m_cfg.app.runtime.lkPerFrameAcceleration);
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

    const bool needsXFeatWorker = m_aliases.featureFrontend == FeatureFrontend::LK && m_aliases.lkXFeatSeeding;
    if (needsXFeatWorker && !m_cfg.app.runtime.xfeatRepo.empty() &&
        !m_cfg.app.runtime.xfeatWorkerScript.empty()) {
        std::string xfeatErr;
        if (m_xfeatFrontendClient.Start(m_cfg.app.runtime.xfeatPython, m_cfg.app.runtime.xfeatWorkerScript,
                                        m_cfg.app.runtime.xfeatRepo, m_cfg.app.runtime.xfeatDevice,
                                        m_cfg.app.runtime.xfeatTopK,
                                        m_cfg.app.runtime.xfeatMaxPoints, &xfeatErr)) {
            std::cerr << "[slam] xfeat worker ready repo=" << m_cfg.app.runtime.xfeatRepo
                      << " device=" << m_cfg.app.runtime.xfeatDevice
                      << " top_k=" << m_cfg.app.runtime.xfeatTopK
                      << " max_points=" << m_cfg.app.runtime.xfeatMaxPoints << "\n";
        } else if (needsXFeatWorker) {
            std::cerr << "[slam] warning: xfeat worker start failed: " << xfeatErr << "\n";
        }
    }
    if (m_aliases.featureFrontend == FeatureFrontend::LK) {
        std::cerr << "[slam] feature_frontend=lk selected; xfeat_seeding="
                  << (m_aliases.lkXFeatSeeding ? "enabled" : "disabled")
                  << " lk_grid=48\n";
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
        m_slamEngine.Stop();
        m_slamStarted = false;
    }
    m_xfeatFrontendClient.Stop();
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
            m_aliases, m_monoMode, m_useImu, m_tuning, m_livePose, m_mav, m_slamEngine, &m_xfeatFrontendClient,
            *m_cameraProvider, m_imuProvider, m_posePublisher, m_udp, m_frameTimingTracker, m_perceptionPipeline,
            m_posePostprocessor,
            m_autoSlamModeController, m_stereoBodyExtrinsics});
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
    m_xfeatFrontendClient.Stop();
    m_mav.StopSetpointStream();
    if (m_slamStarted) {
        m_slamEngine.Stop();
        m_slamStarted = false;
    }
    m_livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
}

} // namespace smartdrone::core::application
