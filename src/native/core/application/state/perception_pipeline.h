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

struct StereoAcquireRequest {
    ports::ICameraProvider &camera;
    int slamInputFps{};
    int timeoutMs{};
    StereoBatch &out;
    FrameTimingTracker *timingTracker{};
};

struct StereoFrameTiming {
    int64_t captureTimestampNs{};
    int64_t logicalFrameTimestampNs{};
    int64_t frameStepNs{};
};

struct StereoGrabFailureLog {
    ports::CameraDiagnostics diagnostics;
    uint64_t minTimestampNs{};
    int timeoutMs{};
    int clampedSlamInputFps{};
    bool packedStereo{};
    const char *likelyCause{"stereo_frame_wait_timeout"};
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
    StereoAcquireStatus AcquireNextStereoBatch(const StereoAcquireRequest &request);

    int ClampTargetFps(int requestedFps) const;

  private:
    uint64_t ComputeMinCaptureTimestampNs(int slamFrameStepNs) const;
    StereoAcquireStatus HandleGrabFailure(ports::ICameraProvider &camera, int timeoutMs,
                                          int clampedSlamInputFps, uint64_t minTimestampNs) const;
    const char *ClassifyGrabFailureCause(const StereoGrabFailureLog &failure) const;
    void LogGrabFailure(const StereoGrabFailureLog &failure) const;
    StereoFrameTiming BuildStereoFrameTiming(const ports::StereoFrame &stereo,
                                             int64_t frameStepNs) const;
    bool ShouldDropByRateLimiter(const StereoFrameTiming &timing,
                                 int64_t toleranceNs) const;
    void AcceptStereoBatch(ports::StereoFrame stereo, const StereoFrameTiming &timing,
                           int64_t slamFrameStepNs, StereoBatch &out);
    void RecordFrameTiming(const StereoBatch &out, FrameTimingTracker *timingTracker) const;

    PerceptionPipelineConfig m_cfg;
    int64_t m_lastDeliveredLogicalFrameNs{0};
    int64_t m_lastAcceptedCaptureTimestampNs{0};
    int64_t m_nextAcceptedLogicalFrameNs{0};
    uint64_t m_nextFrameId{1};
};

} // namespace smartdrone::core::application
