#include "core/application/session/slam_session_runtime.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

#include "common/logger.h"
#include "common/tlv/tlv_protocol.h"
#include "core/application/session/runtime_session_common.h"

namespace smartdrone::core::application {

namespace {

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
      m_slamSystem(std::make_unique<ORB_SLAM3::System>(cfg.app.vocab, cfg.app.settings, m_orbSensor, false)),
      m_slamEngine(std::move(m_slamSystem), m_orbInputMode, m_useImu), m_cameraProvider(m_cam),
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
        return false;
    }

    m_slamEngine.SetOperationMode(m_frameProcessorState.effectiveSlamMode);
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
        m_stereoBodyExtrinsics = LoadStereoBodyExtrinsics(m_cfg.app.settings);
    }
    if (m_aliases.udpEnable) {
        m_udp.Open(m_aliases.udpIp, m_aliases.udpPort, m_aliases.udpJpegQ, m_aliases.udpPayload, m_aliases.udpQueue);
    }
    if (m_useImu) {
        m_imuThread = StartImuThread(m_aliases, m_imuState, m_stop, m_runningFlag);
    }
    if (!OpenCamera(m_cam, m_aliases)) {
        m_stop.store(true);
        if (m_imuThread.joinable())
            m_imuThread.join();
        if (m_aliases.udpEnable)
            m_udp.Close();
        m_slamEngine.Stop();
        return false;
    }

    m_mav.SetFrameTimingTracker(&m_frameTimingTracker);
    return true;
}

void SlamSessionRuntime::Stop()
{
    m_cam.Close();
    std::cerr << "[session] slam camera closed\n";
    if (m_imuThread.joinable())
        m_imuThread.join();
    std::cerr << "[session] slam imu joined\n";
    if (m_aliases.udpEnable) {
        m_udp.Close();
        std::cerr << "[session] slam udp closed\n";
    }
    m_mav.StopSetpointStream();
    std::cerr << "[session] slam setpoint stopped\n";
    m_slamEngine.Stop();
    m_mav.SetFrameTimingTracker(nullptr);
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
            m_aliases, m_monoMode, m_useImu, m_tuning, m_livePose, m_mav, m_slamEngine, m_cameraProvider, m_imuProvider,
            m_posePublisher, m_udp, m_frameTimingTracker, m_perceptionPipeline, m_posePostprocessor,
            m_autoSlamModeController, m_stereoBodyExtrinsics});
    }
    if (!m_frameProcessor) {
        m_frameProcessor = std::make_unique<SlamFrameProcessor>(*m_frameProcessorContext, m_frameProcessorState);
    }
    return *m_frameProcessor;
}

} // namespace smartdrone::core::application
