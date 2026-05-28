#pragma once

#include <functional>

#include "core/application/session/slam/slam_processing_support.h"
#include "core/application/state/pose_postprocessor.h"

namespace SmartDrone::Core::Ports {
class ICameraProvider;
class IImuProvider;
class IPosePublisher;
class ISlamEngine;
} // namespace SmartDrone::Core::Ports

namespace SmartDrone::Core::Application {

class FrameTimingTracker;
class PerceptionPipeline;
class SlamRuntimeControlPort;
class IPreviewOutputPort;
class LivePoseState;
struct LiveRuntimeTuning;
struct MainRuntimeAliases;
struct StereoBodyExtrinsics;

struct SlamFrameProcessingContext {
    const MainRuntimeAliases &aliases;
    bool monoMode{false};
    bool useImu{false};
    LiveRuntimeTuning &tuning;
    LivePoseState &livePose;
    SlamRuntimeControlPort *slamControl{nullptr};
    SmartDrone::Core::Ports::ICameraProvider &cameraProvider;
    SmartDrone::Core::Ports::IImuProvider &imuProvider;
    FrameTimingTracker &frameTimingTracker;
    PerceptionPipeline &perceptionPipeline;
    AutoSlamModeController &autoSlamModeController;
};

struct SlamFrameTrackingContext {
    SmartDrone::Core::Ports::ISlamEngine &slamEngine;
    SlamRuntimeControlPort *slamControl{nullptr};
    FrameTimingTracker &frameTimingTracker;
};

struct SlamFramePosePostprocessContext {
    bool monoMode{false};
    bool useImu{false};
    LiveRuntimeTuning &tuning;
    LivePoseState &livePose;
    PosePostprocessor::ReadRangeSensorFn readRangeSensor;
    SlamRuntimeControlPort *slamControl{nullptr};
    PosePostprocessor &posePostprocessor;
    AutoSlamModeController &autoSlamModeController;
    const StereoBodyExtrinsics &stereoBodyExtrinsics;
};

struct SlamFrameOutputContext {
    const MainRuntimeAliases &aliases;
    bool monoMode{false};
    LivePoseState &livePose;
    SmartDrone::Core::Ports::IPosePublisher &posePublisher;
    IPreviewOutputPort &previewOutput;
};

} // namespace SmartDrone::Core::Application
