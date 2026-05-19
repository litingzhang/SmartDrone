#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "adapters/imu/icm42688_imu_provider.h"
#include "adapters/slam/slam_engine_control.h"
#include "adapters/slam/slam_engine_factory.h"
#include "adapters/slam/visual_feature_frontend_client.h"
#include "adapters/stream/udp_image_sender.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/session/imu_runtime_state.h"
#include "core/application/session/sensor_runtime_helpers.h"
#include "core/application/session/slam_frame_processor.h"
#include "core/application/session/slam_processing_support.h"
#include "core/application/session/slam_settings_loader.h"
#include "core/application/state/frame_timing_tracker.h"
#include "core/application/state/live_pose_state.h"
#include "core/application/state/perception_pipeline.h"
#include "core/application/state/pose_postprocessor.h"
#include "core/ports/camera_provider.h"
#include "core/ports/pose_publisher.h"
#include "core/ports/slam_session_telemetry.h"
#include "core/ports/slam_engine.h"
#include "core/ports/visual_feature_frontend.h"

namespace smartdrone::core::application {

struct SlamSessionRuntimeConfig {
  const UnifiedConfig &cfg;
  LiveRuntimeTuning &tuning;
  smartdrone::core::ports::ISlamSessionTelemetryPort &telemetry;
  smartdrone::core::ports::IPosePublisher &posePublisher;
  LivePoseState &livePose;
  std::atomic<bool> &stop;
  std::atomic<bool> &runningFlag;
};

class SlamSessionRuntime {
public:
  explicit SlamSessionRuntime(SlamSessionRuntimeConfig config);

  bool Start();
  void Stop();
  bool StepImuPoll();
  bool ImuReady() const;
  SlamFrameProcessor &FrameProcessor();

  bool sessionOk{true};

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
  smartdrone::adapters::slam::SlamInputMode m_slamInputMode{
      smartdrone::adapters::slam::SlamInputMode::Stereo};
  std::string m_effectiveSettingsPath;

  std::unique_ptr<smartdrone::core::ports::ISlamEngine> m_slamEngine;
  smartdrone::adapters::slam::ISlamRuntimeControl *m_slamControl{nullptr};
  smartdrone::core::ports::IVisualFeatureFrontend *m_visualFeatureFrontend{
      nullptr};
  std::unique_ptr<smartdrone::adapters::slam::IManagedVisualFeatureFrontend>
      m_visualFeatureFrontendClient;
  AutoSlamModeController m_autoSlamModeController{};
  StereoBodyExtrinsics m_stereoBodyExtrinsics{};
  UdpImageSender m_udp;
  ImuThreadState m_imuState{};
  ImuSensorPoller m_imuPoller;
  std::unique_ptr<smartdrone::core::ports::ICameraProvider> m_cameraProvider;
  smartdrone::adapters::imu::Icm42688ImuProvider m_imuProvider;
  FrameTimingTracker m_frameTimingTracker{};
  PerceptionPipeline m_perceptionPipeline;
  PosePostprocessor m_posePostprocessor{};
  SlamFrameProcessor::State m_frameProcessorState;
  std::unique_ptr<SlamFrameProcessor::Context> m_frameProcessorContext;
  std::unique_ptr<SlamFrameProcessor> m_frameProcessor;
  bool m_udpOpen{false};
  bool m_cameraOpen{false};
  bool m_slamStarted{false};

  bool FailStart();
  bool StartSlamEngine();
  void ConfigureSlamControl();
  void ApplyInitialSlamMode();
  void LoadStereoBodyExtrinsicsIfNeeded();
  smartdrone::adapters::slam::VisualFeatureFrontendRuntimeConfig
  BuildVisualFeatureFrontendConfig() const;
  void StartVisualFeatureFrontend();
  void HandleVisualFeatureFrontendReady(
      const smartdrone::adapters::slam::VisualFeatureFrontendRuntimeConfig
          &featureConfig);
  void HandleVisualFeatureFrontendStartFailure(const std::string &featureErr);
  void LogLkFrontendSelection() const;
  bool OpenUdp();
  bool StartImuPoller();
  bool OpenCamera();
  void CleanupAfterStartFailure();
};

} // namespace smartdrone::core::application
