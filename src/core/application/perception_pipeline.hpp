#pragma once

#include <algorithm>
#include <cstdint>

#include "core/ports/camera_provider.hpp"

namespace smartdrone::core::application {

struct PerceptionPipelineConfig {
    int cameraFps{60};
    bool preferLatestFrame{true};
};

struct StereoBatch {
    ports::StereoFrame stereo;
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
                                               StereoBatch& out)
    {
        ports::StereoFrame stereo{};
        if (!camera.GrabStereo(stereo, timeoutMs, m_cfg.preferLatestFrame)) {
            return camera.GetHealth().healthy ? StereoAcquireStatus::Timeout
                                              : StereoAcquireStatus::CameraUnhealthy;
        }

        const int64_t frameStepNs = 1000000000LL / std::max(1, m_cfg.cameraFps);
        int64_t frameNs = static_cast<int64_t>((stereo.left.timestampNs + stereo.right.timestampNs) / 2ULL);
        if (m_lastDeliveredFrameNs != 0 && frameNs <= m_lastDeliveredFrameNs) {
            frameNs = m_lastDeliveredFrameNs + frameStepNs;
        }

        const int clampedSlamInputFps = ClampTargetFps(slamInputFps);
        const int64_t slamFrameStepNs = 1000000000LL / std::max(1, clampedSlamInputFps);
        if (m_lastAcceptedFrameNs != 0 && (frameNs - m_lastAcceptedFrameNs) < slamFrameStepNs) {
            return StereoAcquireStatus::DroppedByRateLimiter;
        }

        m_lastDeliveredFrameNs = frameNs;
        m_lastAcceptedFrameNs = frameNs;

        out.stereo = std::move(stereo);
        out.frameTimestampNs = frameNs;
        out.monotonicFrameStepNs = frameStepNs;
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
};

}  // namespace smartdrone::core::application
