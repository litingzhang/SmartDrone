#pragma once

#include <algorithm>
#include <cstdint>

#include "core/application/frame_timing_tracker.hpp"
#include "core/ports/camera_provider.hpp"

namespace smartdrone::core::application {

struct PerceptionPipelineConfig {
    int cameraFps{60};
    bool preferLatestFrame{true};
};

struct StereoBatch {
    ports::StereoFrame stereo;
    uint64_t frameId{0};
    int64_t frameTimestampNs{0};
    int64_t monotonicFrameStepNs{0};
};

enum class StereoAcquireStatus : uint8_t {
    Ok,
    Timeout,
    DroppedByRateLimiter,
    CameraUnhealthy,
};

class PerceptionPipeline {
public:
    explicit PerceptionPipeline(PerceptionPipelineConfig cfg)
        : m_cfg(cfg)
    {
    }

    StereoAcquireStatus AcquireNextStereoBatch(ports::ICameraProvider& camera,
                                               int slamInputFps,
                                               int timeoutMs,
                                               StereoBatch& out,
                                               FrameTimingTracker* timingTracker = nullptr)
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
            return camera.GetHealth().healthy ? StereoAcquireStatus::Timeout
                                              : StereoAcquireStatus::CameraUnhealthy;
        }

        const int64_t frameStepNs = 1000000000LL / std::max(1, m_cfg.cameraFps);
        int64_t frameNs = static_cast<int64_t>((stereo.left.timestampNs + stereo.right.timestampNs) / 2ULL);
        if (m_lastDeliveredFrameNs != 0 && frameNs <= m_lastDeliveredFrameNs) {
            frameNs = m_lastDeliveredFrameNs + frameStepNs;
        }

        if (m_nextAcceptedFrameNs == 0) {
            m_nextAcceptedFrameNs = frameNs;
        }

        // Integer nanosecond frame steps can be a few microseconds larger than the
        // actual sensor cadence (e.g. 50.000ms target vs ~49.998ms from 60Hz stereo).
        // A small tolerance avoids accidentally dropping an extra frame and falling
        // to the next lower cadence bucket.
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

    int ClampTargetFps(int requestedFps) const
    {
        if (requestedFps <= 0) {
            return std::max(1, m_cfg.cameraFps);
        }
        return std::clamp(requestedFps, 1, std::max(1, m_cfg.cameraFps));
    }

private:
    PerceptionPipelineConfig m_cfg;
    int64_t m_lastDeliveredFrameNs{0};
    int64_t m_lastAcceptedFrameNs{0};
    int64_t m_nextAcceptedFrameNs{0};
    uint64_t m_nextFrameId{1};
};

}  // namespace smartdrone::core::application
