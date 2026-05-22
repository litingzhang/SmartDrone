#pragma once

#include <functional>

#include "core/application/session/slam/slam_processing_support.h"
#include "core/application/state/pose_postprocessor.h"

namespace SmartDrone::core::ports {
class ICameraProvider;
class IImuProvider;
class IPosePublisher;
class ISlamEngine;
} // namespace SmartDrone::core::ports

namespace SmartDrone::core::application {

class FrameTimingTracker;
class PerceptionPipeline;
class SlamRuntimeControlPort;
class IPreviewOutputPort;
struct LivePoseState;
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
    SmartDrone::core::ports::ICameraProvider &cameraProvider;
    SmartDrone::core::ports::IImuProvider &imuProvider;
    FrameTimingTracker &frameTimingTracker;
    PerceptionPipeline &perceptionPipeline;
    AutoSlamModeController &autoSlamModeController;
};

struct SlamFrameTrackingContext {
    SmartDrone::core::ports::ISlamEngine &slamEngine;
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
    SmartDrone::core::ports::IPosePublisher &posePublisher;
    IPreviewOutputPort &previewOutput;
};

} // namespace SmartDrone::core::application
