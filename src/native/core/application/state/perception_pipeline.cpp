#include "core/application/state/perception_pipeline.h"

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <utility>

namespace SmartDrone::Core::Application {

PerceptionPipeline::PerceptionPipeline(PerceptionPipelineConfig cfg)
    : m_cfg(cfg)
{
}

StereoAcquireStatus PerceptionPipeline::AcquireNextStereoBatch(
    Ports::ICameraProvider &camera, int slamInputFps,
    int staleFrameThresholdMs, StereoBatch &out,
    FrameTimingTracker *timingTracker)
{
    return AcquireNextStereoBatch(
        StereoAcquireRequest{camera, slamInputFps, staleFrameThresholdMs,
                             out, timingTracker});
}

StereoAcquireStatus PerceptionPipeline::AcquireNextStereoBatch(
    const StereoAcquireRequest &request)
{
    if (DetectCameraClockReset(request.camera)) {
        return StereoAcquireStatus::CameraClockReset;
    }
    Ports::StereoFrame stereo{};
    const int clampedSlamInputFps = ClampTargetFps(request.slamInputFps);
    const uint64_t minTimestampNs =
        ComputeMinCaptureTimestampNs(clampedSlamInputFps);

    if (!request.camera.GrabStereo(stereo, m_cfg.preferLatestFrame,
                                   minTimestampNs)) {
        if (DetectCameraClockReset(request.camera)) {
            return StereoAcquireStatus::CameraClockReset;
        }
        return HandleGrabFailure(request.camera, request.staleFrameThresholdMs,
                                 clampedSlamInputFps, minTimestampNs);
    }
    if (DetectCameraClockReset(request.camera)) {
        return StereoAcquireStatus::CameraClockReset;
    }

    const int64_t frameStepNs = 1000000000LL / std::max(1, m_cfg.cameraFps);
    const StereoFrameTiming timing = BuildStereoFrameTiming(stereo, frameStepNs);

    AcceptStereoBatch(std::move(stereo), timing, request.out);
    AdvanceCaptureSchedule(timing.captureTimestampNs, clampedSlamInputFps);
    RecordFrameTiming(request.out, request.timingTracker);
    return StereoAcquireStatus::Ok;
}

bool PerceptionPipeline::DetectCameraClockReset(
    Ports::ICameraProvider &camera)
{
    const uint32_t resetCounter = camera.GetDiagnostics().clockResetCounter;
    if (!m_cameraClockInitialized) {
        m_cameraClockResetCounter = resetCounter;
        m_cameraClockInitialized = true;
        return false;
    }
    if (resetCounter == m_cameraClockResetCounter) {
        return false;
    }
    m_cameraClockResetCounter = resetCounter;
    ResetTimeline();
    return true;
}

void PerceptionPipeline::ResetTimeline()
{
    m_lastDeliveredLogicalFrameNs = 0;
    m_lastAcceptedCaptureTimestampNs = 0;
    m_nextCaptureTimestampNs = 0;
    m_captureScheduleFps = 0;
}

uint64_t PerceptionPipeline::ComputeMinCaptureTimestampNs(
    int clampedSlamInputFps)
{
    if (m_lastAcceptedCaptureTimestampNs == 0) {
        return 0;
    }
    if (m_captureScheduleFps != clampedSlamInputFps ||
        m_nextCaptureTimestampNs == 0) {
        const uint64_t intervalNs =
            1000000000ULL / static_cast<uint64_t>(clampedSlamInputFps);
        m_captureScheduleFps = clampedSlamInputFps;
        m_nextCaptureTimestampNs =
            static_cast<uint64_t>(m_lastAcceptedCaptureTimestampNs) + intervalNs;
    }
    const uint64_t toleranceNs = CaptureScheduleToleranceNs();
    return m_nextCaptureTimestampNs > toleranceNs
               ? m_nextCaptureTimestampNs - toleranceNs
               : 0;
}

void PerceptionPipeline::AdvanceCaptureSchedule(
    int64_t captureTimestampNs, int clampedSlamInputFps)
{
    const uint64_t captureNs = static_cast<uint64_t>(captureTimestampNs);
    const uint64_t intervalNs =
        1000000000ULL / static_cast<uint64_t>(clampedSlamInputFps);
    if (m_captureScheduleFps != clampedSlamInputFps ||
        m_nextCaptureTimestampNs == 0) {
        m_captureScheduleFps = clampedSlamInputFps;
        m_nextCaptureTimestampNs = captureNs + intervalNs;
        return;
    }
    const uint64_t adjustedCaptureNs = captureNs + CaptureScheduleToleranceNs();
    if (adjustedCaptureNs >= m_nextCaptureTimestampNs) {
        const uint64_t elapsedIntervals =
            (adjustedCaptureNs - m_nextCaptureTimestampNs) / intervalNs + 1ULL;
        m_nextCaptureTimestampNs += elapsedIntervals * intervalNs;
    }
}

uint64_t PerceptionPipeline::CaptureScheduleToleranceNs() const
{
    const uint64_t cameraIntervalNs =
        1000000000ULL / static_cast<uint64_t>(std::max(1, m_cfg.cameraFps));
    return std::max<uint64_t>(1, cameraIntervalNs / 8ULL);
}

StereoAcquireStatus PerceptionPipeline::HandleGrabFailure(
    Ports::ICameraProvider &camera, int staleFrameThresholdMs,
    int clampedSlamInputFps, uint64_t minTimestampNs) const
{
    StereoGrabFailureLog failure;
    failure.diagnostics = camera.GetDiagnostics();
    failure.minTimestampNs = minTimestampNs;
    failure.staleFrameThresholdMs = staleFrameThresholdMs;
    failure.clampedSlamInputFps = clampedSlamInputFps;
    failure.packedStereo =
        camera.Semantics() == Ports::CameraProviderSemantics::PackedStereoSingleDevice;
    failure.likelyCause = ClassifyGrabFailureCause(failure);

    if (failure.diagnostics.healthy && staleFrameThresholdMs <= 0) {
        return StereoAcquireStatus::Timeout;
    }
    LogGrabFailure(failure);
    return camera.GetHealth().healthy ? StereoAcquireStatus::Timeout : StereoAcquireStatus::CameraUnhealthy;
}

const char *PerceptionPipeline::ClassifyGrabFailureCause(
    const StereoGrabFailureLog &failure) const
{
    const auto &diag = failure.diagnostics;
    const bool packedStereo = failure.packedStereo;
    const int staleFrameThresholdMs = failure.staleFrameThresholdMs;
    if (!diag.acceptFrames) {
        return "camera_not_accepting_frames";
    }
    if (!diag.healthy) {
        return "camera_unhealthy";
    }
    if (packedStereo && diag.lastFrameAgeMsL >= staleFrameThresholdMs &&
        diag.lastFrameAgeMsR >= staleFrameThresholdMs) {
        return "packed_stereo_stream_stalled";
    }
    if (packedStereo && diag.pairedQueue > 0) {
        return "eligible_frame_filter";
    }
    if (!packedStereo && diag.lastFrameAgeMsL >= staleFrameThresholdMs &&
        diag.lastFrameAgeMsR < staleFrameThresholdMs) {
        return "left_stream_stalled";
    }
    if (!packedStereo && diag.lastFrameAgeMsR >= staleFrameThresholdMs &&
        diag.lastFrameAgeMsL < staleFrameThresholdMs) {
        return "right_stream_stalled";
    }
    if (!packedStereo && diag.pendingL > 0 && diag.pendingR == 0) {
        return "waiting_right_frame";
    }
    if (!packedStereo && diag.pendingR > 0 && diag.pendingL == 0) {
        return "waiting_left_frame";
    }
    if (!packedStereo && diag.pendingL > 0 && diag.pendingR > 0 &&
        diag.pairedQueue == 0) {
        return "pairing_threshold_or_min_ts";
    }
    if (!packedStereo && diag.pairedQueue > 0) {
        return "eligible_pair_filter";
    }
    return "no_eligible_stereo_frame";
}

void PerceptionPipeline::LogGrabFailure(const StereoGrabFailureLog &failure) const
{
    const auto &diag = failure.diagnostics;
    char line[512];
    std::snprintf(
        line, sizeof(line),
        "[stereo_acquire_failure] stale_threshold_ms=%d slam_fps=%d min_ts=%llu last_accept_ts=%lld "
        "accept=%d healthy=%d cause=%s "
        "raw_seq=[%u,%u] raw_count=[%llu,%llu] pending=[%zu,%zu] paired=%zu "
        "drop_pair=%llu drop_unpaired=[%llu,%llu] "
        "pair_tol_ms=%.3f last_pair_dt_ms=%lld last_reject_dt_ms=%.3f age_ms=[%lld,%lld] last_pair_age_ms=%lld",
        failure.staleFrameThresholdMs, failure.clampedSlamInputFps,
        static_cast<unsigned long long>(failure.minTimestampNs),
        static_cast<long long>(m_lastAcceptedCaptureTimestampNs),
        diag.acceptFrames ? 1 : 0, diag.healthy ? 1 : 0, failure.likelyCause, static_cast<unsigned>(diag.lastRawSeqL),
        static_cast<unsigned>(diag.lastRawSeqR), static_cast<unsigned long long>(diag.rawCountL),
        static_cast<unsigned long long>(diag.rawCountR), diag.pendingL, diag.pendingR, diag.pairedQueue,
        static_cast<unsigned long long>(diag.droppedPairs), static_cast<unsigned long long>(diag.droppedUnpairedL),
        static_cast<unsigned long long>(diag.droppedUnpairedR), static_cast<double>(diag.pairTolNs) * 1e-6,
        static_cast<long long>(diag.lastPairDtMs), static_cast<double>(diag.lastRejectDtUs) * 1e-3,
        static_cast<long long>(diag.lastFrameAgeMsL), static_cast<long long>(diag.lastFrameAgeMsR),
        static_cast<long long>(diag.lastPairAgeMs));
    std::cerr << line << "\n";
}

StereoFrameTiming PerceptionPipeline::BuildStereoFrameTiming(
    const Ports::StereoFrame &stereo, int64_t frameStepNs) const
{
    const uint64_t earlierTimestampNs = std::min(stereo.left.timestampNs, stereo.right.timestampNs);
    const uint64_t laterTimestampNs = std::max(stereo.left.timestampNs, stereo.right.timestampNs);
    const int64_t captureTimestampNs =
        static_cast<int64_t>(earlierTimestampNs + ((laterTimestampNs - earlierTimestampNs) / 2ULL));
    int64_t logicalFrameTimestampNs = captureTimestampNs;
    if (m_lastDeliveredLogicalFrameNs != 0 && logicalFrameTimestampNs <= m_lastDeliveredLogicalFrameNs) {
        logicalFrameTimestampNs = m_lastDeliveredLogicalFrameNs + frameStepNs;
    }
    return {captureTimestampNs, logicalFrameTimestampNs, frameStepNs};
}

void PerceptionPipeline::AcceptStereoBatch(Ports::StereoFrame stereo,
                                           const StereoFrameTiming &timing,
                                           StereoBatch &out)
{
    m_lastDeliveredLogicalFrameNs = timing.logicalFrameTimestampNs;
    m_lastAcceptedCaptureTimestampNs = timing.captureTimestampNs;

    out.stereo = std::move(stereo);
    out.frameId = m_nextFrameId++;
    out.captureTimestampNs = timing.captureTimestampNs;
    out.logicalFrameTimestampNs = timing.logicalFrameTimestampNs;
    out.monotonicFrameStepNs = timing.frameStepNs;
}

void PerceptionPipeline::RecordFrameTiming(const StereoBatch &out,
                                           FrameTimingTracker *timingTracker) const
{
    if (!timingTracker) {
        return;
    }
    const int64_t earlierCaptureNs = std::max<int64_t>(
        0, std::min(out.stereo.left.captureMonotonicNs,
                    out.stereo.right.captureMonotonicNs));
    const int64_t laterCaptureNs = std::max<int64_t>(
        0, std::max(out.stereo.left.captureMonotonicNs,
                    out.stereo.right.captureMonotonicNs));
    FrameCaptureTiming timing;
    timing.tCamNs = static_cast<uint64_t>(out.captureTimestampNs);
    timing.tCaptureMonotonicNs = static_cast<uint64_t>(
        earlierCaptureNs + (laterCaptureNs - earlierCaptureNs) / 2LL);
    timing.tLeftArrivalNs = static_cast<uint64_t>(
        std::max<int64_t>(0, out.stereo.left.arriveNs));
    timing.tRightArrivalNs = static_cast<uint64_t>(
        std::max<int64_t>(0, out.stereo.right.arriveNs));
    timingTracker->UpsertCapture(out.frameId, timing);
}

int PerceptionPipeline::ClampTargetFps(int requestedFps) const
{
    if (requestedFps <= 0) {
        return std::max(1, m_cfg.cameraFps);
    }
    return std::clamp(requestedFps, 1, std::max(1, m_cfg.cameraFps));
}

} // namespace SmartDrone::Core::Application
