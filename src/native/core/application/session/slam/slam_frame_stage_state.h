#pragma once

#include <atomic>
#include <cstdint>

#include <sophus/se3.hpp>

#include "core/application/config/app_args.h"
#include "core/application/session/slam/slam_processing_support.h"
#include "core/application/state/pose_postprocessor.h"
#include "core/ports/slam_tracking_state.h"

namespace SmartDrone::core::application {

struct SlamFrameSharedState {
    SlamFrameSharedState() = default;
    SlamFrameSharedState(
        SmartDrone::core::domain::SlamOperationMode requestedMode,
        SmartDrone::core::domain::SlamOperationMode effectiveMode)
        : requestedSlamMode(requestedMode), effectiveSlamMode(effectiveMode)
    {
    }

    std::atomic<bool> lastTrackingUsable{false};
    std::atomic<int> lastTrackingState{
        SmartDrone::core::ports::kSlamTrackingNoImagesYet};
    std::atomic<SmartDrone::core::domain::SlamOperationMode>
        requestedSlamMode{
            SmartDrone::core::domain::SlamOperationMode::Mapping};
    std::atomic<SmartDrone::core::domain::SlamOperationMode>
        effectiveSlamMode{
            SmartDrone::core::domain::SlamOperationMode::Mapping};
};

struct SlamFrameInputState {
    std::int64_t lastFrameNs{0};
    int lastLoggedConfiguredSlamInputFps{-1};
    int lastLoggedEffectiveSlamInputFps{-1};
    std::uint64_t imuWarmupSamples{0};
    std::int64_t lastFrameGapWarnLogNs{0};
    int adaptiveSlamInputFps{0};
    int visualFeatureLoadSheddingLevel{0};
    int lastLoggedVisualFeatureLoadSheddingLevel{-1};
    FeatureFrontend lastAppliedFeatureFrontend{
        FeatureFrontend::LkGfttPerFrame};
};

struct SlamFramePosePostprocessState {
    Sophus::SE3f stereoReferencePose{Sophus::SE3f()};
    bool stereoReferencePoseSet{false};
    unsigned long lastRawMapId{
        PosePostprocessor::ContinuityMapper::kInvalidMapId};
    Sophus::SE3f lastValidTwcRaw{Sophus::SE3f()};
    bool haveLastValidTwcRaw{false};
    std::uint8_t sessionResetCounterBase{0};
    std::uint16_t sessionResetMapCountBase{0};
};

struct SlamFrameOutputState {
    std::atomic<std::int64_t> lastPointCloudUpdateNs{0};
    std::atomic<std::uint64_t> frameIndex{0};
    std::atomic<std::int64_t> lastPublishedFrameNs{0};
    std::atomic<std::uint64_t> rateLimitedDrops{0};
    std::atomic<double> smoothedAcquireMs{0.0};
    std::atomic<double> smoothedSlamMs{0.0};
    std::atomic<double> smoothedTotalMs{0.0};
};

} // namespace SmartDrone::core::application
