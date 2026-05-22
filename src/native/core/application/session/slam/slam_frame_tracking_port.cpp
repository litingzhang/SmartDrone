#include "core/application/session/slam/slam_frame_tracking_port.h"

#include <chrono>
#include <utility>

#include "core/application/state/frame_timing_tracker.h"
#include "core/application/session/slam/slam_processing_support.h"
#include "core/ports/slam_engine.h"

namespace SmartDrone::core::application {

SlamFrameTrackingPort::SlamFrameTrackingPort(
    SlamFrameTrackingContext &context)
    : m_ctx(context)
{
}

SlamFrameStepResult SlamFrameTrackingPort::TrackPreparedFrame(
    std::shared_ptr<SlamPreparedFrameData> frame,
    SlamTrackedFrameData &tracked)
{
    if (!frame) {
        return SlamFrameStepResult::Continue;
    }
    auto &stereoBatch = frame->stereoBatch;
    auto &slamInput = frame->slamInput;
    auto &right = stereoBatch.stereo.right;
    const bool debugRightOnlyFeatures = frame->debugRightOnlyFeatures;
    const bool extractFeatures = frame->extractFeatures;
    const bool updatePointCloud = frame->updatePointCloud;

    const auto slamStartTp = std::chrono::steady_clock::now();
    const uint64_t slamInputTimestampNs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            slamStartTp.time_since_epoch())
            .count());
    m_ctx.frameTimingTracker.MarkSlamIn(slamInput.frameId,
                                        slamInputTimestampNs);
    SmartDrone::core::ports::SlamOutput slamOutput{};
    slamOutput.frameId = slamInput.frameId;
    slamOutput.captureTimestampNs = slamInput.captureTimestampNs;
    if (debugRightOnlyFeatures) {
        slamOutput.leftFeatures.clear();
        slamOutput.rightFeatures = ComputeOrbDebugFeatures(right.gray);
    } else {
        slamOutput = m_ctx.slamEngine.Process(slamInput, extractFeatures,
                                              updatePointCloud);
    }
    const auto slamEndTp = std::chrono::steady_clock::now();
    const uint64_t slamOutputTimestampNs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            slamEndTp.time_since_epoch())
            .count());
    m_ctx.frameTimingTracker.MarkSlamOut(slamInput.frameId,
                                         slamOutputTimestampNs);

    tracked.frame = std::move(frame);
    tracked.slamOutput = std::move(slamOutput);
    tracked.slamStartTp = slamStartTp;
    tracked.slamEndTp = slamEndTp;
    return SlamFrameStepResult::Continue;
}

} // namespace SmartDrone::core::application
