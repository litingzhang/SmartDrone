#pragma once

#include <functional>

#include "core/application/session/slam/slam_processing_support.h"
#include "core/application/state/pose_postprocessor.h"

namespace smartdrone::core::ports {
class ICameraProvider;
class IImuProvider;
class IPosePublisher;
class ISlamEngine;
} // namespace smartdrone::core::ports

namespace smartdrone::core::application {

class FrameTimingTracker;
class ISlamPreviewOutputPort;
class PerceptionPipeline;
class SlamRuntimeControlPort;
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
    smartdrone::core::ports::ICameraProvider &cameraProvider;
    smartdrone::core::ports::IImuProvider &imuProvider;
    FrameTimingTracker &frameTimingTracker;
    PerceptionPipeline &perceptionPipeline;
    AutoSlamModeController &autoSlamModeController;
};

struct SlamFrameTrackingContext {
    smartdrone::core::ports::ISlamEngine &slamEngine;
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
    smartdrone::core::ports::IPosePublisher &posePublisher;
    ISlamPreviewOutputPort &previewOutput;
};

} // namespace smartdrone::core::application
