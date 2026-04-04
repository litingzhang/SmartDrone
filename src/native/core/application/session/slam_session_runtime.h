#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include "System.h"
#include "adapters/camera/libcamera_stereo_camera.h"
#include "adapters/imu/icm42688_imu_provider.h"
#include "adapters/slam/orbslam3_engine.h"
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

namespace smartdrone::core::application {

class SlamSessionRuntime {
  public:
    SlamSessionRuntime(const UnifiedConfig &cfg, LiveRuntimeTuning &tuning, Px4MavlinkGateway &mav,
                       LivePoseState &livePose, std::atomic<bool> &stop, std::atomic<bool> &runningFlag);

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
    ORB_SLAM3::System::eSensor m_orbSensor{ORB_SLAM3::System::STEREO};
    smartdrone::adapters::slam::OrbInputMode m_orbInputMode{smartdrone::adapters::slam::OrbInputMode::Stereo};

    std::unique_ptr<ORB_SLAM3::System> m_slamSystem;
    smartdrone::adapters::slam::OrbSlam3Engine m_slamEngine;
    AutoSlamModeController m_autoSlamModeController{};
    StereoBodyExtrinsics m_stereoBodyExtrinsics{};
    UdpImageSender m_udp;
    ImuThreadState m_imuState{};
    std::thread m_imuThread;
    LibcameraStereoOV9281_TsPair m_cam;
    smartdrone::adapters::camera::LibcameraStereoCamera m_cameraProvider;
    smartdrone::adapters::imu::Icm42688ImuProvider m_imuProvider;
    FrameTimingTracker m_frameTimingTracker{};
    smartdrone::adapters::telemetry::MavlinkPosePublisher m_posePublisher;
    PerceptionPipeline m_perceptionPipeline;
    PosePostprocessor m_posePostprocessor{};
    SlamFrameProcessor::State m_frameProcessorState;
    std::unique_ptr<SlamFrameProcessor::Context> m_frameProcessorContext;
    std::unique_ptr<SlamFrameProcessor> m_frameProcessor;
};

} // namespace smartdrone::core::application
