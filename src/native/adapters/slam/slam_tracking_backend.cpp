#include "adapters/slam/slam_tracking_backend.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <utility>

#include <sophus/se3.hpp>

#include "adapters/slam/orb_slam3_backend.h"
#include "adapters/slam/slam_engine_access.h"
#include "adapters/slam/slam_env.h"
#include "adapters/slam/slam_pose_utils.h"
#include "core/ports/slam_tracking_state.h"

namespace smartdrone::adapters::slam {

namespace {

void NormalizeRealtimePublishState(core::ports::SlamOutput &out)
{
    if (!out.poseValid || !out.pose.valid || IsIdentityPose(out.pose) ||
        !EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_CONTINUITY_MARK_RECENTLY_LOST", true)) {
        return;
    }
    if (out.trackingState != core::ports::kSlamTrackingOk &&
        out.trackingState != core::ports::kSlamTrackingRecentlyLost) {
        out.trackingState = core::ports::kSlamTrackingRecentlyLost;
    }
}

} // namespace

core::ports::SlamOutput RunSlamTrackingBackend(SlamEngineAdapter &engine,
                                                  const core::ports::SlamInputBatch &input,
                                                  bool extractFeatures, bool extractPointCloud,
                                                  const ExternalStereoTrackRequest *externalRequest)
{
    core::ports::SlamOutput out{};
    OrbSlam3Backend *backend = SlamEngineAccess::OrbBackend(engine);
    if (backend == nullptr || !backend->Available()) {
        return out;
    }

    SlamModeSharedState &state = SlamEngineAccess::ModeState(engine);
    const SlamInputMode inputMode = SlamEngineAccess::InputMode(engine);
    const bool useImu = SlamEngineAccess::UseImu(engine);
    const bool monoMode = (inputMode != SlamInputMode::Stereo);
    const cv::Mat &monoImage =
        (inputMode == SlamInputMode::MonoRight) ? input.stereo.right.gray : input.stereo.left.gray;

    const bool trackWithExternalStereo =
        externalRequest != nullptr && externalRequest->enabled && !monoMode && !externalRequest->leftPrepared.empty() &&
        !externalRequest->rightPrepared.empty();
    if (!trackWithExternalStereo && state.m_lastSuperPointImageCount == 0) {
        state.ResetExternalFeatureStats();
    }

    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    out.usedSuperPointFrontend = false;
    state.CopyExternalFeatureStatsToOutput(out);

    Sophus::SE3f tcw;
    if (trackWithExternalStereo) {
        out.inputPrepareMs = externalRequest->inputPrepareMs;
        out.frontendMs = externalRequest->frontendMs;
        out.stereoPairMs = externalRequest->stereoPairMs;
        out.externalPackMs = externalRequest->externalPackMs;
        out.monoAugmentMs = externalRequest->monoAugmentMs;
        state.m_lastSuperPointExternalHash = externalRequest->externalHash;

        const auto trackStartTp = std::chrono::steady_clock::now();
        tcw = backend->TrackPreparedStereoWithFeatures(input, externalRequest->leftPrepared,
                                                       externalRequest->rightPrepared,
                                                       externalRequest->observations, useImu);
        const auto trackEndTp = std::chrono::steady_clock::now();
        backend->LogExternalStereoDfx(input.frameId, externalRequest->observations);
        if (externalRequest->recordTotalMs) {
            state.m_lastSuperPointTotalMs =
                std::chrono::duration<double, std::milli>(trackEndTp - externalRequest->totalStartTp).count();
        }
        out.orbTrackMs = std::chrono::duration<double, std::milli>(trackEndTp - trackStartTp).count();
        out.frontendMs = state.m_lastSuperPointFrontendMs > 0.0 ? state.m_lastSuperPointFrontendMs : out.frontendMs;
        state.CopyExternalFeatureStatsToOutput(out);
        out.usedSuperPointFrontend = true;
        out.leftFeatures = externalRequest->leftFeaturePoints;
        out.rightFeatures = externalRequest->rightFeaturePoints;
    } else {
        const auto orbTrackStartTp = std::chrono::steady_clock::now();
        tcw = backend->TrackRaw(input, inputMode, useImu);
        const auto orbTrackEndTp = std::chrono::steady_clock::now();
        out.orbTrackMs = std::chrono::duration<double, std::milli>(orbTrackEndTp - orbTrackStartTp).count();
    }

    backend->CopyTrackingStatsToOutput(out);

    Sophus::SE3f twc = tcw.inverse();
    bool trajectoryPoseLost = false;
    if (EnvFlagEnabled("SMART_DRONE_ORB_LIVE_EUROC_TRAJECTORY_POSE", false)) {
        Sophus::SE3f trajectoryTwc;
        if (backend->GetLatestFrameTrajectoryPoseEuRoC(trajectoryTwc, nullptr, &trajectoryPoseLost)) {
            twc = trajectoryTwc;
        }
    }
    const Eigen::Vector3f t = twc.translation();
    const Eigen::Quaternionf q(twc.so3().unit_quaternion());
    const bool finitePose = std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()) &&
                            std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) &&
                            std::isfinite(q.z());
    out.poseValid = finitePose && TrackingStateCanPublishPose(out.trackingState) && !trajectoryPoseLost;
    out.pose.valid = out.poseValid;
    out.pose.x = t.x();
    out.pose.y = t.y();
    out.pose.z = t.z();
    out.pose.qw = q.w();
    out.pose.qx = q.x();
    out.pose.qy = q.y();
    out.pose.qz = q.z();

    SlamEngineAccess::GateRealtimePoseQuality(engine, out, input.frameTimeSec);
    out.pose.valid = out.poseValid;
    if (EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_CONTINUITY", true)) {
        SlamEngineAccess::MaintainRealtimePoseContinuity(engine, out.pose, out.poseValid, input.frameTimeSec,
                                                         out.trackingState);
        if (out.poseValid && !TrackingStateCanPublishPose(out.trackingState) &&
            !IsIdentityPose(out.pose) &&
            EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_CONTINUITY_MARK_RECENTLY_LOST", true)) {
            out.trackingState = core::ports::kSlamTrackingRecentlyLost;
        }
        out.pose.valid = out.poseValid;
    }

    if (EnvFlagEnabled("SMART_DRONE_POSE_STABILIZER", false)) {
        SlamEngineAccess::StabilizeOutputPose(engine, out.pose, out.poseValid, input.frameTimeSec,
                                                  out.trackingState);
        NormalizeRealtimePublishState(out);
    }
    NormalizeRealtimePublishState(out);

    const bool needVisualExtraction = extractPointCloud || extractFeatures;
    if (!needVisualExtraction) {
        return out;
    }

    const int leftWidth = monoMode ? monoImage.cols : input.stereo.left.gray.cols;
    const int leftHeight = monoMode ? monoImage.rows : input.stereo.left.gray.rows;
    TrackedVisualSnapshot visual =
        backend->ExtractTrackedVisualSnapshot(leftWidth, leftHeight, monoMode ? 0 : input.stereo.right.gray.cols,
                                              monoMode ? 0 : input.stereo.right.gray.rows, extractPointCloud, 120);
    out.matchesInliers = visual.matchesInliers;
    out.trackedMapPointCount = static_cast<uint32_t>(visual.trackedMapPointCount);
    out.localMapPointCount = static_cast<uint32_t>(visual.localMapPointCount);
    out.localMapPointHash = visual.localMapPointHash;
    out.matchedMapPointHashBeforePoseOptimization = visual.matchedMapPointHashBeforePoseOptimization;
    out.trackedMapPointHash = visual.trackedMapPointHash;
    if (extractFeatures && !out.usedSuperPointFrontend) {
        if (inputMode == SlamInputMode::MonoRight) {
            out.rightFeatures = std::move(visual.leftFeatures);
        } else {
            out.leftFeatures = std::move(visual.leftFeatures);
            out.rightFeatures = std::move(visual.rightFeatures);
        }
    }
    if (extractPointCloud) {
        out.pointCloudXyz = std::move(visual.pointCloudXyz);
    }
    return out;
}

} // namespace smartdrone::adapters::slam
