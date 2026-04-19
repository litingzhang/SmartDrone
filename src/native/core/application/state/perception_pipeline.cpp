#include "core/application/state/perception_pipeline.h"

#include <algorithm>
#include <cstdio>
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
        const auto diag = camera.GetDiagnostics();
        const bool packedStereo = camera.Semantics() == ports::CameraProviderSemantics::PackedStereoSingleDevice;
        const char *likelyCause = "stereo_frame_wait_timeout";
        if (!diag.acceptFrames) {
            likelyCause = "camera_not_accepting_frames";
        } else if (!diag.healthy) {
            likelyCause = "camera_unhealthy";
        } else if (packedStereo && diag.lastFrameAgeMsL >= timeoutMs && diag.lastFrameAgeMsR >= timeoutMs) {
            likelyCause = "packed_stereo_stream_stalled";
        } else if (packedStereo && diag.pairedQueue > 0) {
            likelyCause = "eligible_frame_filter";
        } else if (!packedStereo && diag.lastFrameAgeMsL >= timeoutMs && diag.lastFrameAgeMsR < timeoutMs) {
            likelyCause = "left_stream_stalled";
        } else if (!packedStereo && diag.lastFrameAgeMsR >= timeoutMs && diag.lastFrameAgeMsL < timeoutMs) {
            likelyCause = "right_stream_stalled";
        } else if (!packedStereo && diag.pendingL > 0 && diag.pendingR == 0) {
            likelyCause = "waiting_right_frame";
        } else if (!packedStereo && diag.pendingR > 0 && diag.pendingL == 0) {
            likelyCause = "waiting_left_frame";
        } else if (!packedStereo && diag.pendingL > 0 && diag.pendingR > 0 && diag.pairedQueue == 0) {
            likelyCause = "pairing_threshold_or_min_ts";
        } else if (!packedStereo && diag.pairedQueue > 0) {
            likelyCause = "eligible_pair_filter";
        }

        char line[512];
        std::snprintf(
            line, sizeof(line),
            "[stereo_timeout] timeout_ms=%d slam_fps=%d min_ts=%llu last_accept_ts=%lld next_logical_ts=%lld "
            "accept=%d healthy=%d cause=%s "
            "raw_seq=[%u,%u] raw_count=[%llu,%llu] pending=[%zu,%zu] paired=%zu "
            "drop_pair=%llu drop_unpaired=[%llu,%llu] "
            "pair_tol_ms=%.3f last_pair_dt_ms=%lld last_reject_dt_ms=%.3f age_ms=[%lld,%lld] last_pair_age_ms=%lld",
            timeoutMs, clampedSlamInputFps, static_cast<unsigned long long>(minTimestampNs),
            static_cast<long long>(m_lastAcceptedCaptureTimestampNs), static_cast<long long>(m_nextAcceptedLogicalFrameNs),
            diag.acceptFrames ? 1 : 0, diag.healthy ? 1 : 0, likelyCause, static_cast<unsigned>(diag.lastRawSeqL),
            static_cast<unsigned>(diag.lastRawSeqR), static_cast<unsigned long long>(diag.rawCountL),
            static_cast<unsigned long long>(diag.rawCountR), diag.pendingL, diag.pendingR, diag.pairedQueue,
            static_cast<unsigned long long>(diag.droppedPairs), static_cast<unsigned long long>(diag.droppedUnpairedL),
            static_cast<unsigned long long>(diag.droppedUnpairedR), static_cast<double>(diag.pairTolNs) * 1e-6,
            static_cast<long long>(diag.lastPairDtMs), static_cast<double>(diag.lastRejectDtUs) * 1e-3,
            static_cast<long long>(diag.lastFrameAgeMsL), static_cast<long long>(diag.lastFrameAgeMsR),
            static_cast<long long>(diag.lastPairAgeMs));
        std::fprintf(stderr, "%s\n", line);
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
