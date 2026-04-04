#include "core/application/state/perception_pipeline.h"

#include <algorithm>
#include <utility>

namespace smartdrone::core::application {

PerceptionPipeline::PerceptionPipeline(PerceptionPipelineConfig cfg) : m_cfg(cfg) {}

StereoAcquireStatus PerceptionPipeline::AcquireNextStereoBatch(ports::ICameraProvider &camera, int slamInputFps,
                                                               int timeoutMs, StereoBatch &out,
                                                               FrameTimingTracker *timingTracker)
{
    ports::StereoFrame stereo{};
    const int clampedSlamInputFps = ClampTargetFps(slamInputFps);
    const int64_t slamFrameStepNs = 1000000000LL / std::max(1, clampedSlamInputFps);
    uint64_t minTimestampNs = 0;
    if (m_lastAcceptedCaptureTimestampNs != 0) {
        const int64_t toleranceNs = std::max<int64_t>(2000000LL, slamFrameStepNs / 20);
        minTimestampNs =
            static_cast<uint64_t>(std::max<int64_t>(0, m_lastAcceptedCaptureTimestampNs + slamFrameStepNs - toleranceNs));
    }

    if (!camera.GrabStereo(stereo, timeoutMs, m_cfg.preferLatestFrame, minTimestampNs)) {
        return camera.GetHealth().healthy ? StereoAcquireStatus::Timeout : StereoAcquireStatus::CameraUnhealthy;
    }

    const int64_t frameStepNs = 1000000000LL / std::max(1, m_cfg.cameraFps);
    const uint64_t earlierTimestampNs = std::min(stereo.left.timestampNs, stereo.right.timestampNs);
    const uint64_t laterTimestampNs = std::max(stereo.left.timestampNs, stereo.right.timestampNs);
    const int64_t captureTimestampNs =
        static_cast<int64_t>(earlierTimestampNs + ((laterTimestampNs - earlierTimestampNs) / 2ULL));
    int64_t logicalFrameTimestampNs = captureTimestampNs;
    if (m_lastDeliveredLogicalFrameNs != 0 && logicalFrameTimestampNs <= m_lastDeliveredLogicalFrameNs) {
        logicalFrameTimestampNs = m_lastDeliveredLogicalFrameNs + frameStepNs;
    }

    if (m_nextAcceptedLogicalFrameNs == 0) {
        m_nextAcceptedLogicalFrameNs = logicalFrameTimestampNs;
    }

    const int64_t toleranceNs = std::max<int64_t>(2000000LL, slamFrameStepNs / 20);
    if (logicalFrameTimestampNs + toleranceNs < m_nextAcceptedLogicalFrameNs) {
        return StereoAcquireStatus::DroppedByRateLimiter;
    }

    m_lastDeliveredLogicalFrameNs = logicalFrameTimestampNs;
    m_lastAcceptedCaptureTimestampNs = captureTimestampNs;
    m_nextAcceptedLogicalFrameNs = logicalFrameTimestampNs + slamFrameStepNs;

    out.stereo = std::move(stereo);
    out.frameId = m_nextFrameId++;
    out.captureTimestampNs = captureTimestampNs;
    out.logicalFrameTimestampNs = logicalFrameTimestampNs;
    out.monotonicFrameStepNs = frameStepNs;
    if (timingTracker) {
        const uint64_t tCamNs = static_cast<uint64_t>(captureTimestampNs);
        const uint64_t tCbNs =
            static_cast<uint64_t>(std::max<int64_t>(0, (out.stereo.left.arriveNs + out.stereo.right.arriveNs) / 2LL));
        timingTracker->UpsertCapture(out.frameId, tCamNs, tCbNs);
    }
    return StereoAcquireStatus::Ok;
}

int PerceptionPipeline::ClampTargetFps(int requestedFps) const
{
    if (requestedFps <= 0) {
        return std::max(1, m_cfg.cameraFps);
    }
    return std::clamp(requestedFps, 1, std::max(1, m_cfg.cameraFps));
}

} // namespace smartdrone::core::application
