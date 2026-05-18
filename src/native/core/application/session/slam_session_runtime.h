#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "adapters/imu/icm42688_imu_provider.h"
#include "adapters/slam/slam_engine_control.h"
#include "adapters/slam/slam_engine_factory.h"
#include "adapters/slam/visual_feature_frontend_client.h"
#include "adapters/stream/udp_image_sender.h"
#include "adapters/telemetry/mavlink_pose_publisher.h"
#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/session/sensor_runtime_helpers.h"
#include "core/application/session/slam_frame_processor.h"
#include "core/application/session/slam_processing_support.h"
#include "core/application/state/frame_timing_tracker.h"
#include "core/application/state/live_pose_state.h"
#include "core/application/state/perception_pipeline.h"
#include "core/application/state/pose_postprocessor.h"
#include "core/ports/camera_provider.h"
#include "core/ports/slam_engine.h"
#include "core/ports/visual_feature_frontend.h"

namespace smartdrone::core::application {

class SlamSessionRuntime {
public:
  SlamSessionRuntime(const UnifiedConfig &cfg, LiveRuntimeTuning &tuning,
                     Px4MavlinkGateway &mav, LivePoseState &livePose,
                     std::atomic<bool> &stop, std::atomic<bool> &runningFlag);

  bool Start();
  void Stop();
  bool WaitForImuReady() const;
  SlamFrameProcessor &FrameProcessor();

  bool sessionOk{true};

private:
  const UnifiedConfig &m_cfg;
  LiveRuntimeTuning &m_tuning;
  Px4MavlinkGateway &m_mav;
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
  std::thread m_imuThread;
  std::unique_ptr<smartdrone::core::ports::ICameraProvider> m_cameraProvider;
  smartdrone::adapters::imu::Icm42688ImuProvider m_imuProvider;
  FrameTimingTracker m_frameTimingTracker{};
  smartdrone::adapters::telemetry::MavlinkPosePublisher m_posePublisher;
  PerceptionPipeline m_perceptionPipeline;
  PosePostprocessor m_posePostprocessor{};
  SlamFrameProcessor::State m_frameProcessorState;
  std::unique_ptr<SlamFrameProcessor::Context> m_frameProcessorContext;
  std::unique_ptr<SlamFrameProcessor> m_frameProcessor;
  bool m_udpOpen{false};
  bool m_cameraOpen{false};
  bool m_slamStarted{false};

  void CleanupAfterStartFailure();
};

} // namespace smartdrone::core::application
