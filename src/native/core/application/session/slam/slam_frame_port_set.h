#pragma once

#include <memory>

#include "core/application/session/slam/slam_frame_input_port.h"
#include "core/application/session/slam/slam_frame_output_port.h"
#include "core/application/session/slam/slam_frame_pose_postprocess_port.h"
#include "core/application/session/slam/slam_frame_stage_context.h"
#include "core/application/session/slam/slam_frame_stage_state.h"
#include "core/application/session/slam/slam_frame_tracking_port.h"
#include "core/application/session/slam/slam_backend_maintenance_port.h"

namespace SmartDrone::core::ports {
class ISlamSessionTelemetryPort;
} // namespace SmartDrone::core::ports

namespace SmartDrone::core::application {

class IPreviewOutputPort;
struct StereoBodyExtrinsics;

struct SlamFramePortSetConfig {
    const MainRuntimeAliases &aliases;
    bool monoMode{false};
    bool useImu{false};
    LiveRuntimeTuning &tuning;
    LivePoseState &livePose;
    SmartDrone::core::ports::ISlamSessionTelemetryPort &telemetry;
    SmartDrone::core::ports::ISlamEngine &slamEngine;
    SlamRuntimeControlPort &slamControl;
    SmartDrone::core::ports::ICameraProvider &cameraProvider;
    SmartDrone::core::ports::IImuProvider &imuProvider;
    SmartDrone::core::ports::IPosePublisher &posePublisher;
    IPreviewOutputPort &previewOutput;
    FrameTimingTracker &frameTimingTracker;
    PerceptionPipeline &perceptionPipeline;
    PosePostprocessor &posePostprocessor;
    AutoSlamModeController &autoSlamModeController;
    const StereoBodyExtrinsics &stereoBodyExtrinsics;
    SlamFrameSharedState &sharedState;
    SlamFrameInputState &inputState;
    SlamFramePosePostprocessState &posePostprocessState;
    SlamFrameOutputState &outputState;
};

class SlamFramePortSet final {
  public:
    explicit SlamFramePortSet(SlamFramePortSetConfig config);

    void Prepare();
    SlamFrameInputPort &InputPort();
    SlamBackendMaintenancePort &BackendMaintenancePort();
    SlamFrameTrackingPort &TrackingPort();
    SlamFramePosePostprocessPort &PosePostprocessPort();
    SlamFrameOutputPort &OutputPort();

  private:
    SlamFrameProcessingContext &InputContext();
    SlamFrameTrackingContext &TrackingContext();
    SlamFramePosePostprocessContext &PosePostprocessContext();
    SlamFrameOutputContext &OutputContext();

    SlamFramePortSetConfig m_cfg;
    std::unique_ptr<SlamFrameProcessingContext> m_inputContext;
    std::unique_ptr<SlamFrameTrackingContext> m_trackingContext;
    std::unique_ptr<SlamFramePosePostprocessContext> m_posePostprocessContext;
    std::unique_ptr<SlamFrameOutputContext> m_outputContext;
    std::unique_ptr<SlamFrameInputPort> m_inputPort;
    std::unique_ptr<SlamBackendMaintenancePort> m_backendMaintenancePort;
    std::unique_ptr<SlamFrameTrackingPort> m_trackingPort;
    std::unique_ptr<SlamFramePosePostprocessPort> m_posePostprocessPort;
    std::unique_ptr<SlamFrameOutputPort> m_outputPort;
};

} // namespace SmartDrone::core::application
