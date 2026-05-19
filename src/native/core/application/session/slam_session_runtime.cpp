#include "core/application/session/slam_session_runtime.h"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <utility>

#include "common/logger.h"
#include "common/tlv/tlv_protocol.h"
#include "core/application/runtime/runtime_aliases.h"
#include "core/application/session/slam_runtime_environment.h"
#include "core/application/session/slam_settings_loader.h"
#include "core/domain/runtime_mode.h"

namespace smartdrone::core::application {

namespace {

std::atomic<uint32_t> g_slamSessionResetCounter{0};
std::atomic<uint32_t> g_slamSessionResetMapCount{0};

using ControllerMode = smartdrone::core::domain::RuntimeMode;
constexpr uint64_t kRangeSensorMaxAgeUs = 200000ULL;

smartdrone::adapters::imu::Icm42688ImuProviderConfig
MakeImuProviderConfig(const MainRuntimeAliases &aliases)
{
  const int64_t imuDtNs = 1000000000LL / std::max(1, aliases.imuHz);
  const int64_t slackBeforeNs = std::max<int64_t>(2 * imuDtNs, 5000000);
  const int64_t slackAfterNs = std::max<int64_t>(2 * imuDtNs, 5000000);
  return {slackBeforeNs, slackAfterNs};
}

SlamFrameProcessor::State
MakeInitialFrameProcessorState(const MainRuntimeAliases &aliases)
{
  const auto requestedSlamMode = aliases.slamOperationMode;
  const auto effectiveSlamMode =
      requestedSlamMode == smartdrone::core::domain::SlamOperationMode::Auto
          ? smartdrone::core::domain::SlamOperationMode::Mapping
          : requestedSlamMode;
  const uint32_t sessionResetCounter =
      g_slamSessionResetCounter.fetch_add(1, std::memory_order_relaxed) + 1;
  const uint32_t sessionResetMapCount =
      g_slamSessionResetMapCount.fetch_add(1, std::memory_order_relaxed) + 1;
  SlamFrameProcessor::State state;
  state.imuWarmupSamples =
      static_cast<uint64_t>(std::max(20, aliases.imuHz / 2));
  state.sessionResetCounterBase =
      static_cast<uint8_t>(sessionResetCounter & 0xFFu);
  state.sessionResetMapCountBase =
      static_cast<uint16_t>(sessionResetMapCount & 0xFFFFu);
  state.requestedSlamMode = requestedSlamMode;
  state.effectiveSlamMode = effectiveSlamMode;
  return state;
}

PosePostprocessor::ReadRangeSensorFn
BuildRangeSensorReader(
    smartdrone::core::ports::ISlamSessionTelemetryPort &telemetry)
{
  return [&telemetry](PosePostprocessor::RangeSensorSnapshot &snapshot) {
    smartdrone::core::ports::SlamRangeSensor range{};
    if (!telemetry.GetDownwardRange(range, kRangeSensorMaxAgeUs)) {
      return false;
    }
    snapshot.currentDistance = range.currentDistance;
    snapshot.signalQuality = range.signalQuality;
    return true;
  };
}

} // namespace

SlamSessionRuntime::SlamSessionRuntime(SlamSessionRuntimeConfig config)
    : m_cfg(config.cfg), m_tuning(config.tuning), m_telemetry(config.telemetry),
      m_posePublisher(config.posePublisher), m_livePose(config.livePose),
      m_stop(config.stop), m_runningFlag(config.runningFlag),
      m_aliases(BuildRuntimeAliases(config.cfg.app)),
      m_monoMode(m_aliases.sensorMode == SensorMode::Mono ||
                 m_aliases.sensorMode == SensorMode::MonoImu),
      m_useImu(m_aliases.sensorMode == SensorMode::StereoImu ||
               m_aliases.sensorMode == SensorMode::MonoImu),
      m_slamInputMode(m_monoMode
                          ? smartdrone::adapters::slam::SlamInputMode::MonoRight
                          : smartdrone::adapters::slam::SlamInputMode::Stereo),
      m_effectiveSettingsPath(BuildEffectiveSlamSettingsPath(config.cfg)),
      m_imuPoller(m_aliases, m_imuState),
      m_cameraProvider(CreateCameraProvider()),
      m_imuProvider(m_imuState.imuBuffer, MakeImuProviderConfig(m_aliases)),
      m_perceptionPipeline(PerceptionPipelineConfig{m_aliases.fps, true}),
      m_frameProcessorState(MakeInitialFrameProcessorState(m_aliases))
{
  if (m_aliases.slamBackend == SlamBackend::OrbSlam3) {
    ApplyOrbAccelerationEnvironment(config.cfg.app.runtime.orbAcceleration);
  }
  smartdrone::adapters::slam::SlamEngineFactoryConfig engineConfig{};
  engineConfig.backend = m_aliases.slamBackend;
  engineConfig.vocabularyPath = config.cfg.app.vocab;
  engineConfig.settingsPath = m_effectiveSettingsPath;
  engineConfig.sensorMode = m_aliases.sensorMode;
  engineConfig.useViewer = false;
  engineConfig.useImu = m_useImu;
  engineConfig.inputMode = m_slamInputMode;
  engineConfig.runtime = config.cfg.app.runtime;
  smartdrone::adapters::slam::ControlledSlamEngine slamEngine =
      smartdrone::adapters::slam::CreateSlamEngine(engineConfig);
  m_slamControl = slamEngine.control;
  m_slamEngine = std::move(slamEngine.engine);
}

bool SlamSessionRuntime::Start()
{
  PrintStartupConfig(m_cfg.app, m_aliases, ControllerMode::Slam);
  m_livePose.SetRuntimeMode(RUNTIME_MODE_SLAM);
  Logger::Init("./stereo_vslam.log", 32 * 1024 * 1024, Logger::INFO, true);
  if (!StartSlamEngine()) {
    return false;
  }

  ConfigureSlamControl();
  ApplyInitialSlamMode();
  LoadStereoBodyExtrinsicsIfNeeded();
  StartVisualFeatureFrontend();
  LogLkFrontendSelection();
  if (!StartOutputAndSensors()) {
    return false;
  }

  m_telemetry.SetFrameTimingTracker(&m_frameTimingTracker);
  return true;
}

bool SlamSessionRuntime::FailStart()
{
  m_stop.store(true);
  CleanupAfterStartFailure();
  return false;
}

bool SlamSessionRuntime::StartSlamEngine()
{
  if (m_slamEngine != nullptr && m_slamEngine->Start()) {
    m_slamStarted = true;
    return true;
  }
  return FailStart();
}

void SlamSessionRuntime::ConfigureSlamControl()
{
  if (m_slamControl == nullptr) {
    return;
  }
  m_slamControl->SetOperationMode(m_frameProcessorState.effectiveSlamMode);
  m_slamControl->SetFeatureFrontend(m_aliases.featureFrontend);
  m_slamControl->SetVisualFeatureInputSizeLimit(
      m_cfg.app.runtime.visualFeatureInputMaxWidth,
      m_cfg.app.runtime.visualFeatureInputMaxHeight);
  m_slamControl->SetStereoVoLoopClosure(m_cfg.app.runtime.lkLoopClosure,
                                        m_cfg.app.runtime.lkLoopScale,
                                        m_cfg.app.runtime.lkLoopRelaxation);
  m_slamControl->SetStereoVoPerFrameAcceleration(
      m_cfg.app.runtime.lkPerFrameAcceleration);
}

void SlamSessionRuntime::ApplyInitialSlamMode()
{
  if (m_frameProcessorState.requestedSlamMode ==
      smartdrone::core::domain::SlamOperationMode::Auto) {
    std::cerr << "[slam] operation_mode=auto effective_mode=mapping\n";
  }
  m_livePose.SetSlamMode(
      ToRuntimeSlamModeValue(m_frameProcessorState.effectiveSlamMode));
  const auto requestedMode = m_frameProcessorState.requestedSlamMode;
  if (requestedMode == smartdrone::core::domain::SlamOperationMode::
                           Relocalization ||
      requestedMode ==
          smartdrone::core::domain::SlamOperationMode::TrackingOnly) {
    std::cerr << "[slam] note: slam_mode="
              << smartdrone::core::domain::ToString(m_aliases.slamOperationMode)
              << " currently maps to backend localization-only mode\n";
  }
}

void SlamSessionRuntime::LoadStereoBodyExtrinsicsIfNeeded()
{
  if (m_aliases.sensorMode == SensorMode::Stereo) {
    m_stereoBodyExtrinsics = LoadStereoBodyExtrinsics(m_effectiveSettingsPath);
  }
}

smartdrone::adapters::slam::VisualFeatureFrontendRuntimeConfig
SlamSessionRuntime::BuildVisualFeatureFrontendConfig() const
{
  smartdrone::adapters::slam::VisualFeatureFrontendRuntimeConfig featureConfig{};
  featureConfig.repoPath =
      smartdrone::adapters::slam::ResolveVisualFeatureFrontendRepo(
          m_aliases.featureFrontend, m_cfg.app.runtime.visualFeatureRepo);
  featureConfig.device = m_cfg.app.runtime.visualFeatureDevice;
  featureConfig.topK = m_cfg.app.runtime.visualFeatureTopK;
  featureConfig.maxPoints = m_cfg.app.runtime.visualFeatureMaxPoints;
  featureConfig.inputMaxWidth = m_cfg.app.runtime.visualFeatureInputMaxWidth;
  featureConfig.inputMaxHeight = m_cfg.app.runtime.visualFeatureInputMaxHeight;
  return featureConfig;
}

void SlamSessionRuntime::StartVisualFeatureFrontend()
{
  if (!IsVisualFeatureLightGlueFrontend(m_aliases.featureFrontend)) {
    return;
  }

  auto featureConfig = BuildVisualFeatureFrontendConfig();
  smartdrone::adapters::slam::ConfigureVisualFeatureFrontendDefaults(
      m_aliases.featureFrontend, featureConfig);
  if (!smartdrone::adapters::slam::VisualFeatureFrontendClientEnabled(
          m_aliases.featureFrontend)) {
    return;
  }

  m_visualFeatureFrontendClient =
      smartdrone::adapters::slam::CreateVisualFeatureFrontendClient(
          m_aliases.featureFrontend);
  if (m_visualFeatureFrontendClient == nullptr) {
    std::cerr << "[slam] warning: "
              << ToFeatureFrontendText(m_aliases.featureFrontend)
              << " frontend route is available, but no native client is "
                 "registered\n";
    return;
  }

  std::string featureErr;
  if (!m_visualFeatureFrontendClient->Start(featureConfig, &featureErr)) {
    HandleVisualFeatureFrontendStartFailure(featureErr);
    return;
  }
  HandleVisualFeatureFrontendReady(featureConfig);
}

void SlamSessionRuntime::HandleVisualFeatureFrontendReady(
    const smartdrone::adapters::slam::VisualFeatureFrontendRuntimeConfig
        &featureConfig)
{
  m_visualFeatureFrontend = m_visualFeatureFrontendClient.get();
  if (m_slamControl != nullptr) {
    m_slamControl->SetVisualFeatureFrontend(m_visualFeatureFrontend);
  }
  std::cerr << "[slam] " << ToFeatureFrontendText(m_aliases.featureFrontend)
            << " frontend ready repo=" << featureConfig.repoPath
            << " device=" << m_cfg.app.runtime.visualFeatureDevice
            << " top_k=" << m_cfg.app.runtime.visualFeatureTopK
            << " max_points=" << m_cfg.app.runtime.visualFeatureMaxPoints
            << "\n";
}

void SlamSessionRuntime::HandleVisualFeatureFrontendStartFailure(
    const std::string &featureErr)
{
  std::cerr << "[slam] warning: "
            << ToFeatureFrontendText(m_aliases.featureFrontend)
            << " frontend start failed: " << featureErr << "\n";
  m_visualFeatureFrontendClient.reset();
  m_visualFeatureFrontend = nullptr;
}

void SlamSessionRuntime::LogLkFrontendSelection() const
{
  if (m_aliases.featureFrontend == FeatureFrontend::LK) {
    std::cerr << "[slam] feature_frontend=lk selected; lk_grid=48\n";
  }
}

bool SlamSessionRuntime::OpenUdp()
{
  if (!m_aliases.udpEnable) {
    return true;
  }

  auto destinationResolver = [this](sockaddr_in &dst) {
    LivePoseState::Snapshot snapshot{};
    if (!m_livePose.ReadSnapshot(snapshot) || !snapshot.hasPeer ||
        !snapshot.peer.valid) {
      return false;
    }
    dst = snapshot.peer.addr;
    return true;
  };
  if (m_udp.Open(m_aliases.udpIp, m_aliases.udpPort, m_aliases.udpJpegQ,
                 m_aliases.udpPayload, m_aliases.udpQueue,
                 std::move(destinationResolver))) {
    m_udpOpen = true;
    return true;
  }

  std::cerr << "[session] slam udp open failed\n";
  return FailStart();
}

bool SlamSessionRuntime::StartImuPoller()
{
  if (!m_useImu) {
    return true;
  }
  if (m_imuPoller.Start()) {
    return true;
  }
  return FailStart();
}

bool SlamSessionRuntime::OpenCamera()
{
  if (m_cameraProvider != nullptr && m_cameraProvider->Open(m_aliases)) {
    m_cameraOpen = true;
    return true;
  }
  return FailStart();
}

bool SlamSessionRuntime::StartOutputAndSensors()
{
  if (!OpenUdp()) {
    return false;
  }
  if (!StartImuPoller()) {
    return false;
  }
  return OpenCamera();
}

void SlamSessionRuntime::ReleaseResources(bool logProgress)
{
  m_telemetry.SetFrameTimingTracker(nullptr);
  if (m_cameraOpen) {
    m_cameraProvider->Close();
    m_cameraOpen = false;
    if (logProgress) {
      std::cerr << "[session] slam camera closed\n";
    }
  }
  if (m_useImu) {
    m_imuPoller.Stop();
    if (logProgress) {
      std::cerr << "[session] slam imu stopped\n";
    }
  }
  if (m_udpOpen) {
    m_udp.Close();
    m_udpOpen = false;
    if (logProgress) {
      std::cerr << "[session] slam udp closed\n";
    }
  }
  m_telemetry.StopSetpointStream();
  if (logProgress) {
    std::cerr << "[session] slam setpoint stopped\n";
  }
  if (m_slamStarted) {
    m_slamEngine->Stop();
    m_slamStarted = false;
  }
  if (m_visualFeatureFrontendClient != nullptr) {
    m_visualFeatureFrontendClient->Stop();
    m_visualFeatureFrontendClient.reset();
  }
  m_visualFeatureFrontend = nullptr;
}

void SlamSessionRuntime::Stop()
{
  ReleaseResources(true);
  std::cerr << "[session] slam shutdown complete\n";
  m_livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
  std::cerr << "[session] slam exit\n";
}

bool SlamSessionRuntime::StepImuPoll()
{
  if (!m_useImu)
    return true;
  m_imuPoller.Step();
  return !m_imuPoller.Failed();
}

bool SlamSessionRuntime::ImuReady() const
{
  if (!m_useImu)
    return true;
  const uint64_t imuCnt = m_imuState.imuCnt.load(std::memory_order_relaxed);
  const bool imuReady = m_imuState.imuOk.load(std::memory_order_relaxed) &&
                        imuCnt >= m_frameProcessorState.imuWarmupSamples;
  if (imuReady)
    return true;
  return false;
}

SlamFrameProcessor &SlamSessionRuntime::FrameProcessor()
{
  if (!m_frameProcessorContext) {
    m_frameProcessorContext = std::make_unique<SlamFrameProcessor::Context>(
        SlamFrameProcessor::Context{
            m_aliases, m_monoMode, m_useImu, m_tuning, m_livePose,
            BuildRangeSensorReader(m_telemetry),
            *m_slamEngine, m_slamControl, m_visualFeatureFrontend,
            *m_cameraProvider, m_imuProvider, m_posePublisher, m_udp,
            m_frameTimingTracker, m_perceptionPipeline, m_posePostprocessor,
            m_autoSlamModeController, m_stereoBodyExtrinsics});
  }
  if (!m_frameProcessor) {
    m_frameProcessor = std::make_unique<SlamFrameProcessor>(
        *m_frameProcessorContext, m_frameProcessorState);
  }
  return *m_frameProcessor;
}

void SlamSessionRuntime::CleanupAfterStartFailure()
{
  ReleaseResources(false);
  m_livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
}

} // namespace smartdrone::core::application
