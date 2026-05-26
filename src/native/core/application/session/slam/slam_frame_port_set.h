#pragma once

#include <memory>

#include "core/application/session/slam/slam_frame_input_port.h"
#include "core/application/session/slam/slam_frame_output_port.h"
#include "core/application/session/slam/slam_frame_pose_postprocess_port.h"
#include "core/application/session/slam/slam_frame_stage_context.h"
#include "core/application/session/slam/slam_frame_stage_state.h"
#include "core/application/session/slam/slam_frame_tracking_port.h"
#include "core/application/session/slam/slam_backend_maintenance_port.h"

namespace SmartDrone::Core::Ports {
class ISlamBackendMaintenance;
class ISlamSessionTelemetryPort;
} // namespace SmartDrone::Core::Ports

namespace SmartDrone::Core::Application {

class IPreviewOutputPort;
struct StereoBodyExtrinsics;

struct SlamFramePortSetConfig {
    const MainRuntimeAliases &aliases;
    bool monoMode{false};
    bool useImu{false};
    LiveRuntimeTuning &tuning;
    LivePoseState &livePose;
    SmartDrone::Core::Ports::ISlamSessionTelemetryPort &telemetry;
    SmartDrone::Core::Ports::ISlamEngine &slamEngine;
    SlamRuntimeControlPort &slamControl;
    SmartDrone::Core::Ports::ISlamBackendMaintenance *slamBackendMaintenance{
        nullptr};
    SmartDrone::Core::Ports::ICameraProvider &cameraProvider;
    SmartDrone::Core::Ports::IImuProvider &imuProvider;
    SmartDrone::Core::Ports::IPosePublisher &posePublisher;
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

} // namespace SmartDrone::Core::Application
