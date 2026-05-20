#include "core/application/session/slam/slam_session_runtime.h"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <string>
#include <utility>

#include "common/logger.h"
#include "common/tlv/tlv_protocol.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/runtime_aliases.h"
#include "core/application/sensors/imu_sensor_poller.h"
#include "core/application/sensors/imu_runtime_state.h"
#include "core/application/session/slam/slam_frame_stage_data.h"
#include "core/application/session/slam/slam_frame_port_set.h"
#include "core/application/session/slam/slam_preview_output_port.h"
#include "core/application/session/slam/slam_processing_support.h"
#include "core/application/session/slam/slam_runtime_control_port.h"
#include "core/application/session/slam/slam_runtime_environment.h"
#include "core/application/session/slam/slam_session_resource_factory.h"
#include "core/application/session/slam/slam_settings_loader.h"
#include "core/application/state/frame_timing_tracker.h"
#include "core/application/state/live_pose_state.h"
#include "core/application/state/perception_pipeline.h"
#include "core/application/state/pose_postprocessor.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/camera_provider.h"
#include "core/ports/imu_provider.h"
#include "core/ports/pose_publisher.h"
#include "core/ports/slam_engine.h"
#include "core/ports/slam_session_telemetry.h"

namespace smartdrone::core::application {

namespace {

std::atomic<uint32_t> g_slamSessionResetCounter{0};
std::atomic<uint32_t> g_slamSessionResetMapCount{0};

using ControllerMode = smartdrone::core::domain::RuntimeMode;

SlamFrameSharedState MakeInitialFrameSharedState(
    const MainRuntimeAliases &aliases)
{
  const auto requestedSlamMode = aliases.slamOperationMode;
  const auto effectiveSlamMode =
      requestedSlamMode == smartdrone::core::domain::SlamOperationMode::Auto
          ? smartdrone::core::domain::SlamOperationMode::Mapping
          : requestedSlamMode;
  return SlamFrameSharedState{requestedSlamMode, effectiveSlamMode};
}

SlamFrameInputState MakeInitialFrameInputState(
    const MainRuntimeAliases &aliases)
{
  SlamFrameInputState state;
  state.imuWarmupSamples =
      static_cast<uint64_t>(std::max(20, aliases.imuHz / 2));
  return state;
}

SlamFramePosePostprocessState MakeInitialFramePosePostprocessState()
{
  const uint32_t sessionResetCounter =
      g_slamSessionResetCounter.fetch_add(1, std::memory_order_relaxed) + 1;
  const uint32_t sessionResetMapCount =
      g_slamSessionResetMapCount.fetch_add(1, std::memory_order_relaxed) + 1;
  SlamFramePosePostprocessState state;
  state.sessionResetCounterBase =
      static_cast<uint8_t>(sessionResetCounter & 0xFFu);
  state.sessionResetMapCountBase =
      static_cast<uint16_t>(sessionResetMapCount & 0xFFFFu);
  return state;
}

} // namespace

class SlamSessionRuntime::Impl final {
public:
  explicit Impl(SlamSessionRuntimeConfig config);

  bool Start();
  void PrepareFramePorts();
  void Stop();
  bool StepImuPoll();
  bool ImuReady() const;
  SlamFrameStageResult StepBackend();
  SlamFrameStageResult AcquireAndPrepareFrame(SlamPreparedFrameData &frame);
  SlamFrameStageResult TrackPreparedFrame(
      std::shared_ptr<SlamPreparedFrameData> frame,
      SlamTrackedFrameData &tracked);
  SlamFrameStageResult PostprocessTrackedFrame(
      std::shared_ptr<SlamTrackedFrameData> tracked,
      SlamPublishedFrameData &published);
  SlamFrameStageResult EmitPointCloud(SlamPublishedFrameData &published);
  SlamFrameStageResult EmitDfx(SlamPublishedFrameData &published);
  SlamFrameStageResult EmitUdp(SlamPublishedFrameData &published);
  SlamFrameStageResult EmitMavlink(SlamPublishedFrameData &published);
  SlamFrameStageResult EmitLivePose(SlamPublishedFrameData &published);

private:
  const UnifiedConfig &m_cfg;
  LiveRuntimeTuning &m_tuning;
  smartdrone::core::ports::ISlamSessionTelemetryPort &m_telemetry;
  smartdrone::core::ports::IPosePublisher &m_posePublisher;
  LivePoseState &m_livePose;
  std::atomic<bool> &m_stop;
  std::atomic<bool> &m_runningFlag;

  MainRuntimeAliases m_aliases;
  bool m_monoMode{false};
  bool m_useImu{false};
  std::string m_effectiveSettingsPath;

  std::unique_ptr<smartdrone::core::ports::ISlamEngine> m_slamEngine;
  std::unique_ptr<SlamRuntimeControlPort> m_slamRuntimeControl;
  std::unique_ptr<ISlamVisualFeatureFrontendSession>
      m_visualFeatureFrontendSession;
  AutoSlamModeController m_autoSlamModeController{};
  StereoBodyExtrinsics m_stereoBodyExtrinsics{};
  SlamPreviewOutputRuntime m_previewOutputRuntime;
  ImuThreadState m_imuState{};
  ImuSensorPoller m_imuPoller;
  std::unique_ptr<smartdrone::core::ports::ICameraProvider> m_cameraProvider;
  std::unique_ptr<smartdrone::core::ports::IImuProvider> m_imuProvider;
  FrameTimingTracker m_frameTimingTracker{};
  PerceptionPipeline m_perceptionPipeline;
  PosePostprocessor m_posePostprocessor{};
  SlamFrameSharedState m_frameSharedState;
  SlamFrameInputState m_frameInputState;
  SlamFramePosePostprocessState m_framePosePostprocessState;
  SlamFrameOutputState m_frameOutputState;
  std::unique_ptr<SlamFramePortSet> m_framePorts;
  bool m_udpOpen{false};
  bool m_cameraOpen{false};
  bool m_slamStarted{false};
  bool m_sessionOk{true};

  bool FailStart();
  bool StartSlamEngine();
  void ConfigureSlamControl();
  void ApplyInitialSlamMode();
  void LoadStereoBodyExtrinsicsIfNeeded();
  void StartVisualFeatureFrontend();
  void HandleVisualFeatureFrontendReady(
      const SlamVisualFeatureFrontendStartResult &startResult);
  void HandleVisualFeatureFrontendStartFailure(const std::string &featureErr);
  void LogLkFrontendSelection() const;
  bool OpenUdp();
  bool StartImuPoller();
  bool OpenCamera();
  bool StartOutputAndSensors();
  SlamFramePortSet &FramePorts();
  SlamFrameStageResult MakeStageResult(SlamFrameStepResult stepResult) const;
  void ReleaseResources(bool logProgress);
  void CleanupAfterStartFailure();
};

SlamSessionRuntime::Impl::Impl(SlamSessionRuntimeConfig config)
    : m_cfg(config.cfg), m_tuning(config.tuning), m_telemetry(config.telemetry),
      m_posePublisher(config.posePublisher), m_livePose(config.livePose),
      m_stop(config.stop), m_runningFlag(config.runningFlag),
      m_aliases(BuildRuntimeAliases(config.cfg.app)),
      m_monoMode(m_aliases.sensorMode == SensorMode::Mono ||
                 m_aliases.sensorMode == SensorMode::MonoImu),
      m_useImu(m_aliases.sensorMode == SensorMode::StereoImu ||
               m_aliases.sensorMode == SensorMode::MonoImu),
      m_effectiveSettingsPath(BuildEffectiveSlamSettingsPath(config.cfg)),
      m_imuPoller(m_aliases, m_imuState),
      m_perceptionPipeline(PerceptionPipelineConfig{m_aliases.fps, true}),
      m_frameSharedState(MakeInitialFrameSharedState(m_aliases)),
      m_frameInputState(MakeInitialFrameInputState(m_aliases)),
      m_framePosePostprocessState(MakeInitialFramePosePostprocessState())
{
  if (m_aliases.slamBackend == SlamBackend::OrbSlam3) {
    ApplyOrbAccelerationEnvironment(config.cfg.app.runtime.orbAcceleration);
  }

  auto engineResources = CreateSlamSessionEngineResources(
      {config.cfg, m_aliases, m_effectiveSettingsPath, m_useImu});
  m_slamEngine = std::move(engineResources.engine);
  m_slamRuntimeControl = std::move(engineResources.control);
  m_cameraProvider = CreateSlamSessionCameraProvider();
  m_imuProvider = CreateSlamSessionImuProvider(m_imuState, m_aliases);
}

bool SlamSessionRuntime::Impl::Start()
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

bool SlamSessionRuntime::Impl::FailStart()
{
  m_stop.store(true);
  CleanupAfterStartFailure();
  return false;
}

bool SlamSessionRuntime::Impl::StartSlamEngine()
{
  if (m_slamEngine != nullptr && m_slamEngine->Start()) {
    m_slamStarted = true;
    return true;
  }
  return FailStart();
}

void SlamSessionRuntime::Impl::ConfigureSlamControl()
{
  if (!m_slamRuntimeControl || !m_slamRuntimeControl->Available()) {
    return;
  }
  m_slamRuntimeControl->SetOperationMode(
      m_frameSharedState.effectiveSlamMode.load());
  m_slamRuntimeControl->SetFeatureFrontend(m_aliases.featureFrontend);
  m_slamRuntimeControl->SetVisualFeatureInputSizeLimit(
      m_cfg.app.runtime.visualFeatureInputMaxWidth,
      m_cfg.app.runtime.visualFeatureInputMaxHeight);
  m_slamRuntimeControl->SetStereoVoLoopClosure(
      m_cfg.app.runtime.lkLoopClosure, m_cfg.app.runtime.lkLoopScale,
      m_cfg.app.runtime.lkLoopRelaxation);
  m_slamRuntimeControl->SetStereoVoPerFrameAcceleration(
      m_cfg.app.runtime.lkPerFrameAcceleration);
}

void SlamSessionRuntime::Impl::ApplyInitialSlamMode()
{
  if (m_frameSharedState.requestedSlamMode.load() ==
      smartdrone::core::domain::SlamOperationMode::Auto) {
    std::cerr << "[slam] operation_mode=auto effective_mode=mapping\n";
  }
  m_livePose.SetSlamMode(
      ToRuntimeSlamModeValue(m_frameSharedState.effectiveSlamMode.load()));
  const auto requestedMode = m_frameSharedState.requestedSlamMode.load();
  if (requestedMode == smartdrone::core::domain::SlamOperationMode::
                           Relocalization ||
      requestedMode ==
          smartdrone::core::domain::SlamOperationMode::TrackingOnly) {
    std::cerr << "[slam] note: slam_mode="
              << smartdrone::core::domain::ToString(m_aliases.slamOperationMode)
              << " currently maps to backend localization-only mode\n";
  }
}

void SlamSessionRuntime::Impl::LoadStereoBodyExtrinsicsIfNeeded()
{
  if (m_aliases.sensorMode == SensorMode::Stereo) {
    m_stereoBodyExtrinsics = LoadStereoBodyExtrinsics(m_effectiveSettingsPath);
  }
}

void SlamSessionRuntime::Impl::StartVisualFeatureFrontend()
{
  auto startResult = StartSlamVisualFeatureFrontendSession(m_aliases, m_cfg);
  if (!startResult.routeAvailable) {
    return;
  }
  if (startResult.clientMissing) {
    std::cerr << "[slam] warning: "
              << ToFeatureFrontendText(m_aliases.featureFrontend)
              << " frontend route is available, but no native client is "
                 "registered\n";
    return;
  }
  if (!startResult.started) {
    HandleVisualFeatureFrontendStartFailure(startResult.error);
    return;
  }
  HandleVisualFeatureFrontendReady(startResult);
  m_visualFeatureFrontendSession = std::move(startResult.session);
}

void SlamSessionRuntime::Impl::HandleVisualFeatureFrontendReady(
    const SlamVisualFeatureFrontendStartResult &startResult)
{
  if (m_slamRuntimeControl != nullptr) {
    m_slamRuntimeControl->SetVisualFeatureFrontend(startResult.frontend);
  }
  std::cerr << "[slam] " << ToFeatureFrontendText(m_aliases.featureFrontend)
            << " frontend ready repo=" << startResult.repoPath
            << " device=" << m_cfg.app.runtime.visualFeatureDevice
            << " top_k=" << m_cfg.app.runtime.visualFeatureTopK
            << " max_points=" << m_cfg.app.runtime.visualFeatureMaxPoints
            << "\n";
}

void SlamSessionRuntime::Impl::HandleVisualFeatureFrontendStartFailure(
    const std::string &featureErr)
{
  std::cerr << "[slam] warning: "
            << ToFeatureFrontendText(m_aliases.featureFrontend)
            << " frontend start failed: " << featureErr << "\n";
  m_visualFeatureFrontendSession.reset();
}

void SlamSessionRuntime::Impl::LogLkFrontendSelection() const
{
  if (m_aliases.featureFrontend == FeatureFrontend::LK) {
    std::cerr << "[slam] feature_frontend=lk selected; lk_grid=48\n";
  }
}

bool SlamSessionRuntime::Impl::OpenUdp()
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
  if (m_previewOutputRuntime.Open(m_aliases, std::move(destinationResolver))) {
    m_udpOpen = true;
    return true;
  }

  std::cerr << "[session] slam udp open failed\n";
  return FailStart();
}

bool SlamSessionRuntime::Impl::StartImuPoller()
{
  if (!m_useImu) {
    return true;
  }
  if (m_imuPoller.Start()) {
    return true;
  }
  return FailStart();
}

bool SlamSessionRuntime::Impl::OpenCamera()
{
  if (m_cameraProvider != nullptr && m_cameraProvider->Open(m_aliases)) {
    m_cameraOpen = true;
    return true;
  }
  return FailStart();
}

bool SlamSessionRuntime::Impl::StartOutputAndSensors()
{
  if (!OpenUdp()) {
    return false;
  }
  if (!StartImuPoller()) {
    return false;
  }
  return OpenCamera();
}

void SlamSessionRuntime::Impl::ReleaseResources(bool logProgress)
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
    m_previewOutputRuntime.Close();
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
  if (m_visualFeatureFrontendSession != nullptr) {
    m_visualFeatureFrontendSession->Stop();
    m_visualFeatureFrontendSession.reset();
  }
}

void SlamSessionRuntime::Impl::Stop()
{
  ReleaseResources(true);
  std::cerr << "[session] slam shutdown complete\n";
  m_livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
  std::cerr << "[session] slam exit\n";
}

bool SlamSessionRuntime::Impl::StepImuPoll()
{
  if (!m_useImu)
    return true;
  m_imuPoller.Step();
  return !m_imuPoller.Failed();
}

bool SlamSessionRuntime::Impl::ImuReady() const
{
  if (!m_useImu)
    return true;
  const uint64_t imuCnt = m_imuState.imuCnt.load(std::memory_order_relaxed);
  const bool imuReady = m_imuState.imuOk.load(std::memory_order_relaxed) &&
                        imuCnt >= m_frameInputState.imuWarmupSamples;
  if (imuReady)
    return true;
  return false;
}

SlamFrameStageResult SlamSessionRuntime::Impl::StepBackend()
{
  return MakeStageResult(FramePorts().TrackingPort().StepBackend());
}

void SlamSessionRuntime::Impl::PrepareFramePorts()
{
  FramePorts().Prepare();
}

SlamFramePortSet &SlamSessionRuntime::Impl::FramePorts()
{
  if (!m_framePorts) {
    m_framePorts = std::make_unique<SlamFramePortSet>(
        SlamFramePortSetConfig{
            m_aliases,
            m_monoMode,
            m_useImu,
            m_tuning,
            m_livePose,
            m_telemetry,
            *m_slamEngine,
            *m_slamRuntimeControl,
            *m_cameraProvider,
            *m_imuProvider,
            m_posePublisher,
            m_previewOutputRuntime.OutputPort(),
            m_frameTimingTracker,
            m_perceptionPipeline,
            m_posePostprocessor,
            m_autoSlamModeController,
            m_stereoBodyExtrinsics,
            m_frameSharedState,
            m_frameInputState,
            m_framePosePostprocessState,
            m_frameOutputState});
  }
  return *m_framePorts;
}

SlamFrameStageResult SlamSessionRuntime::Impl::MakeStageResult(
    SlamFrameStepResult stepResult) const
{
  return {stepResult, m_sessionOk};
}

SlamFrameStageResult SlamSessionRuntime::Impl::AcquireAndPrepareFrame(
    SlamPreparedFrameData &frame)
{
  const SlamFrameStageResult result =
      FramePorts().InputPort().AcquireAndPrepareFrame(frame);
  m_sessionOk = result.sessionOk;
  return result;
}

SlamFrameStageResult SlamSessionRuntime::Impl::TrackPreparedFrame(
    std::shared_ptr<SlamPreparedFrameData> frame,
    SlamTrackedFrameData &tracked)
{
  return MakeStageResult(
      FramePorts().TrackingPort().TrackPreparedFrame(std::move(frame),
                                                     tracked));
}

SlamFrameStageResult SlamSessionRuntime::Impl::PostprocessTrackedFrame(
    std::shared_ptr<SlamTrackedFrameData> tracked,
    SlamPublishedFrameData &published)
{
  return MakeStageResult(
      FramePorts().PosePostprocessPort().PostprocessTrackedFrame(
          std::move(tracked), published));
}

SlamFrameStageResult SlamSessionRuntime::Impl::EmitPointCloud(
    SlamPublishedFrameData &published)
{
  return MakeStageResult(FramePorts().OutputPort().EmitPointCloud(published));
}

SlamFrameStageResult SlamSessionRuntime::Impl::EmitDfx(
    SlamPublishedFrameData &published)
{
  return MakeStageResult(FramePorts().OutputPort().EmitDfx(published));
}

SlamFrameStageResult SlamSessionRuntime::Impl::EmitUdp(
    SlamPublishedFrameData &published)
{
  return MakeStageResult(FramePorts().OutputPort().EmitUdp(published));
}

SlamFrameStageResult SlamSessionRuntime::Impl::EmitMavlink(
    SlamPublishedFrameData &published)
{
  return MakeStageResult(FramePorts().OutputPort().EmitMavlink(published));
}

SlamFrameStageResult SlamSessionRuntime::Impl::EmitLivePose(
    SlamPublishedFrameData &published)
{
  return MakeStageResult(FramePorts().OutputPort().EmitLivePose(published));
}

void SlamSessionRuntime::Impl::CleanupAfterStartFailure()
{
  ReleaseResources(false);
  m_livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
}

SlamSessionRuntime::SlamSessionRuntime(SlamSessionRuntimeConfig config)
    : m_impl(std::make_unique<Impl>(config))
{
}

SlamSessionRuntime::~SlamSessionRuntime() = default;

bool SlamSessionRuntime::Start()
{
  return m_impl->Start();
}

void SlamSessionRuntime::PrepareFramePorts()
{
  m_impl->PrepareFramePorts();
}

void SlamSessionRuntime::Stop()
{
  m_impl->Stop();
}

bool SlamSessionRuntime::StepImuPoll()
{
  return m_impl->StepImuPoll();
}

bool SlamSessionRuntime::ImuReady() const
{
  return m_impl->ImuReady();
}

SlamFrameStageResult SlamSessionRuntime::StepBackend()
{
  return m_impl->StepBackend();
}

SlamFrameStageResult SlamSessionRuntime::AcquireAndPrepareFrame(
    SlamPreparedFrameData &frame)
{
  return m_impl->AcquireAndPrepareFrame(frame);
}

SlamFrameStageResult SlamSessionRuntime::TrackPreparedFrame(
    std::shared_ptr<SlamPreparedFrameData> frame,
    SlamTrackedFrameData &tracked)
{
  return m_impl->TrackPreparedFrame(std::move(frame), tracked);
}

SlamFrameStageResult SlamSessionRuntime::PostprocessTrackedFrame(
    std::shared_ptr<SlamTrackedFrameData> tracked,
    SlamPublishedFrameData &published)
{
  return m_impl->PostprocessTrackedFrame(std::move(tracked), published);
}

SlamFrameStageResult SlamSessionRuntime::EmitPointCloud(
    SlamPublishedFrameData &published)
{
  return m_impl->EmitPointCloud(published);
}

SlamFrameStageResult SlamSessionRuntime::EmitDfx(
    SlamPublishedFrameData &published)
{
  return m_impl->EmitDfx(published);
}

SlamFrameStageResult SlamSessionRuntime::EmitUdp(
    SlamPublishedFrameData &published)
{
  return m_impl->EmitUdp(published);
}

SlamFrameStageResult SlamSessionRuntime::EmitMavlink(
    SlamPublishedFrameData &published)
{
  return m_impl->EmitMavlink(published);
}

SlamFrameStageResult SlamSessionRuntime::EmitLivePose(
    SlamPublishedFrameData &published)
{
  return m_impl->EmitLivePose(published);
}

} // namespace smartdrone::core::application
