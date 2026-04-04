#pragma once

#include <cstdint>

#include "core/application/state/frame_timing_tracker.h"
#include "core/ports/camera_provider.h"

namespace smartdrone::core::application {

struct PerceptionPipelineConfig {
    int cameraFps{60};
    bool preferLatestFrame{true};
};

struct StereoBatch {
    ports::StereoFrame stereo;
    uint64_t frameId{0};
    int64_t captureTimestampNs{0};
    int64_t logicalFrameTimestampNs{0};
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
    explicit PerceptionPipeline(PerceptionPipelineConfig cfg);

    StereoAcquireStatus AcquireNextStereoBatch(ports::ICameraProvider &camera, int slamInputFps, int timeoutMs,
                                               StereoBatch &out, FrameTimingTracker *timingTracker = nullptr);

    int ClampTargetFps(int requestedFps) const;

  private:
    PerceptionPipelineConfig m_cfg;
    int64_t m_lastDeliveredLogicalFrameNs{0};
    int64_t m_lastAcceptedCaptureTimestampNs{0};
    int64_t m_nextAcceptedLogicalFrameNs{0};
    uint64_t m_nextFrameId{1};
};

} // namespace smartdrone::core::application
