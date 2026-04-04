#include "core/application/state/perception_pipeline.h"

#include <algorithm>
#include <utility>

namespace smartdrone::core::application {

PerceptionPipeline::PerceptionPipeline(PerceptionPipelineConfig cfg) : m_cfg(cfg)
{
}

StereoAcquireStatus PerceptionPipeline::AcquireNextStereoBatch(
    ports::ICameraProvider& camera,
    int slamInputFps,
    int timeoutMs,
    StereoBatch& out,
    FrameTimingTracker* timingTracker)
{
    ports::StereoFrame stereo{};
    const int clampedSlamInputFps = ClampTargetFps(slamInputFps);
    const int64_t slamFrameStepNs = 1000000000LL / std::max(1, clampedSlamInputFps);
    uint64_t minTimestampNs = 0;
    if (m_lastAcceptedFrameNs != 0) {
        const int64_t toleranceNs = std::max<int64_t>(2000000LL, slamFrameStepNs / 20);
        minTimestampNs = static_cast<uint64_t>(std::max<int64_t>(0, m_lastAcceptedFrameNs + slamFrameStepNs - toleranceNs));
    }

    if (!camera.GrabStereo(stereo, timeoutMs, m_cfg.preferLatestFrame, minTimestampNs)) {
        return camera.GetHealth().healthy ? StereoAcquireStatus::Timeout : StereoAcquireStatus::CameraUnhealthy;
    }

    const int64_t frameStepNs = 1000000000LL / std::max(1, m_cfg.cameraFps);
    int64_t frameNs = static_cast<int64_t>((stereo.left.timestampNs + stereo.right.timestampNs) / 2ULL);
    if (m_lastDeliveredFrameNs != 0 && frameNs <= m_lastDeliveredFrameNs) {
        frameNs = m_lastDeliveredFrameNs + frameStepNs;
    }

    if (m_nextAcceptedFrameNs == 0) {
        m_nextAcceptedFrameNs = frameNs;
    }

    const int64_t toleranceNs = std::max<int64_t>(2000000LL, slamFrameStepNs / 20);
    if (frameNs + toleranceNs < m_nextAcceptedFrameNs) {
        return StereoAcquireStatus::DroppedByRateLimiter;
    }

    m_lastDeliveredFrameNs = frameNs;
    m_lastAcceptedFrameNs = frameNs;
    m_nextAcceptedFrameNs = frameNs + slamFrameStepNs;

    out.stereo = std::move(stereo);
    out.frameId = m_nextFrameId++;
    out.frameTimestampNs = frameNs;
    out.monotonicFrameStepNs = frameStepNs;
    if (timingTracker) {
        const uint64_t tCamNs = static_cast<uint64_t>(frameNs);
        const uint64_t tCbNs = static_cast<uint64_t>(std::max<int64_t>(
            0,
            (out.stereo.left.arriveNs + out.stereo.right.arriveNs) / 2LL));
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

}  // namespace smartdrone::core::application
