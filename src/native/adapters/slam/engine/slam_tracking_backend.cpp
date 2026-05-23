#include "adapters/slam/engine/slam_tracking_backend.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <utility>

#include <sophus/se3.hpp>

#include "adapters/slam/engine/slam_engine_access.h"
#include "adapters/slam/engine/slam_env.h"
#include "adapters/slam/engine/slam_output_utils.h"
#include "adapters/slam/engine/slam_pose_utils.h"
#include "core/ports/slam_tracking_state.h"

namespace SmartDrone::Adapters::Slam {

namespace {

struct TrackingBackendContext {
    SlamEngineAdapter *engine{nullptr};
    Core::Ports::ISlamTrackingBackend *backend{nullptr};
    Core::Ports::ITrackedVisualDataProvider *visualProvider{nullptr};
    SlamModeSharedState *state{nullptr};
    SlamInputMode inputMode{SlamInputMode::Stereo};
    bool useImu{false};
    bool monoMode{false};
};

void NormalizeRealtimePublishState(Core::Ports::SlamOutput &out)
{
    if (!out.poseValid || !out.pose.valid || IsIdentityPose(out.pose) ||
        !EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_CONTINUITY_MARK_RECENTLY_LOST",
                        true)) {
        return;
    }
    if (out.trackingState != Core::Ports::kSlamTrackingOk &&
        out.trackingState != Core::Ports::kSlamTrackingRecentlyLost) {
        out.trackingState = Core::Ports::kSlamTrackingRecentlyLost;
    }
}

TrackingBackendContext BuildTrackingBackendContext(SlamEngineAdapter &engine)
{
    TrackingBackendContext context;
    context.engine = &engine;
    context.backend = SlamEngineAccess::TrackingBackend(engine);
    context.visualProvider = SlamEngineAccess::TrackedVisualDataProvider(engine);
    context.state = &SlamEngineAccess::ModeState(engine);
    context.inputMode = SlamEngineAccess::InputMode(engine);
    context.useImu = SlamEngineAccess::UseImu(engine);
    context.monoMode = context.inputMode != SlamInputMode::Stereo;
    return context;
}

bool CanTrack(const TrackingBackendContext &context)
{
    return context.backend != nullptr && context.backend->Available();
}

void CopyStereoFeatureTimingToOutput(const StereoFeatureTrackRequest &request,
                                     Core::Ports::SlamOutput &out)
{
    out.inputPrepareMs = request.inputPrepareMs;
    out.frontendMs = request.frontendMs;
    out.stereoPairMs = request.stereoPairMs;
    out.featurePackMs = request.featurePackMs;
    out.monoAugmentMs = request.monoAugmentMs;
}

Sophus::SE3f TrackPreparedStereoFeatures(
    const TrackingBackendContext &context,
    const Core::Ports::SlamInputBatch &input,
    const StereoFeatureTrackRequest &request, Core::Ports::SlamOutput &out)
{
    CopyStereoFeatureTimingToOutput(request, out);
    context.state->m_lastVisualFeatureObservationHash = request.observationHash;

    const auto trackStartTp = std::chrono::steady_clock::now();
    Core::Ports::PreparedStereoFeatureTrackRequest trackRequest;
    trackRequest.input = &input;
    trackRequest.leftPrepared = &request.leftPrepared;
    trackRequest.rightPrepared = &request.rightPrepared;
    trackRequest.observations = &request.observations;
    trackRequest.useImu = context.useImu;
    Sophus::SE3f tcw =
        context.backend->TrackPreparedStereoWithFeatures(trackRequest);
    const auto trackEndTp = std::chrono::steady_clock::now();
    context.backend->LogStereoFeatureDiagnostics(input.frameId,
                                                 request.observations);
    if (request.recordTotalMs) {
        context.state->m_lastVisualFeatureTotalMs =
            std::chrono::duration<double, std::milli>(
                trackEndTp - request.totalStartTp)
                .count();
    }
    out.orbTrackMs =
        std::chrono::duration<double, std::milli>(trackEndTp - trackStartTp)
            .count();
    out.frontendMs = context.state->m_lastVisualFeatureFrontendMs > 0.0
                         ? context.state->m_lastVisualFeatureFrontendMs
                         : out.frontendMs;
    context.state->CopyVisualFeatureStatsToOutput(out);
    out.usedVisualFeatureFrontend = true;
    out.leftFeatures = request.leftFeaturePoints;
    out.rightFeatures = request.rightFeaturePoints;
    return tcw;
}

Sophus::SE3f TrackRawInput(const TrackingBackendContext &context,
                           const Core::Ports::SlamInputBatch &input,
                           Core::Ports::SlamOutput &out)
{
    const auto orbTrackStartTp = std::chrono::steady_clock::now();
    Core::Ports::SlamTrackRequest trackRequest;
    trackRequest.input = &input;
    trackRequest.inputMode = context.inputMode;
    trackRequest.useImu = context.useImu;
    Sophus::SE3f tcw = context.backend->TrackRaw(trackRequest);
    const auto orbTrackEndTp = std::chrono::steady_clock::now();
    out.orbTrackMs =
        std::chrono::duration<double, std::milli>(orbTrackEndTp -
                                                 orbTrackStartTp)
            .count();
    return tcw;
}

bool CanTrackWithStereoFeatures(
    const StereoFeatureTrackRequest *stereoFeatureRequest, bool monoMode)
{
    return stereoFeatureRequest != nullptr && stereoFeatureRequest->enabled &&
           !monoMode && !stereoFeatureRequest->leftPrepared.empty() &&
           !stereoFeatureRequest->rightPrepared.empty();
}

bool TryGetTrajectoryPose(Core::Ports::ISlamTrackingBackend &backend,
                          Sophus::SE3f &twc, bool &trajectoryPoseLost)
{
    if (!EnvFlagEnabled("SMART_DRONE_ORB_LIVE_EUROC_TRAJECTORY_POSE", false)) {
        return false;
    }
    Sophus::SE3f trajectoryTwc;
    if (!backend.GetLatestFrameTrajectoryPoseEuRoC(trajectoryTwc, nullptr,
                                                   &trajectoryPoseLost)) {
        return false;
    }
    twc = trajectoryTwc;
    return true;
}

void FillOutputPose(const TrackingBackendContext &context,
                    const Core::Ports::SlamInputBatch &input,
                    const Sophus::SE3f &tcw, Core::Ports::SlamOutput &out)
{
    Sophus::SE3f twc = tcw.inverse();
    bool trajectoryPoseLost = false;
    TryGetTrajectoryPose(*context.backend, twc, trajectoryPoseLost);
    const Eigen::Vector3f t = twc.translation();
    const Eigen::Quaternionf q(twc.so3().unit_quaternion());
    const bool finitePose = std::isfinite(t.x()) && std::isfinite(t.y()) &&
                            std::isfinite(t.z()) && std::isfinite(q.w()) &&
                            std::isfinite(q.x()) && std::isfinite(q.y()) &&
                            std::isfinite(q.z());
    out.poseValid = finitePose &&
                    TrackingStateCanPublishPose(out.trackingState) &&
                    !trajectoryPoseLost;
    out.pose.valid = out.poseValid;
    out.pose.x = t.x();
    out.pose.y = t.y();
    out.pose.z = t.z();
    out.pose.qw = q.w();
    out.pose.qx = q.x();
    out.pose.qy = q.y();
    out.pose.qz = q.z();

    SlamEngineAccess::GateRealtimePoseQuality(*context.engine, out,
                                              input.frameTimeSec);
}

void ApplyRealtimePoseContinuity(const TrackingBackendContext &context,
                                 double frameTimeSec,
                                 Core::Ports::SlamOutput &out)
{
    out.pose.valid = out.poseValid;
    if (!EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_CONTINUITY", true)) {
        return;
    }
    SlamEngineAccess::MaintainRealtimePoseContinuity(
        *context.engine, out.pose, out.poseValid, frameTimeSec,
        out.trackingState);
    if (out.poseValid && !TrackingStateCanPublishPose(out.trackingState) &&
        !IsIdentityPose(out.pose) &&
        EnvFlagEnabled(
            "SMART_DRONE_REALTIME_POSE_CONTINUITY_MARK_RECENTLY_LOST", true)) {
        out.trackingState = Core::Ports::kSlamTrackingRecentlyLost;
    }
    out.pose.valid = out.poseValid;
}

void ApplyPoseStabilizer(const TrackingBackendContext &context,
                         double frameTimeSec, Core::Ports::SlamOutput &out)
{
    if (!EnvFlagEnabled("SMART_DRONE_POSE_STABILIZER", false)) {
        NormalizeRealtimePublishState(out);
        return;
    }
    SlamEngineAccess::StabilizeOutputPose(
        *context.engine, out.pose, out.poseValid, frameTimeSec,
        out.trackingState);
    NormalizeRealtimePublishState(out);
}

Core::Ports::VisualMapSnapshotRequest BuildSnapshotRequest(
    const TrackingBackendContext &context,
    const Core::Ports::SlamInputBatch &input,
    bool extractFeatures, bool extractPointCloud,
    const Core::Ports::SlamOutput &out)
{
    const cv::Mat &monoImage = context.inputMode == SlamInputMode::MonoRight
                                   ? input.stereo.right.gray
                                   : input.stereo.left.gray;
    Core::Ports::VisualMapSnapshotRequest request;
    request.leftImageWidth =
        context.monoMode ? monoImage.cols : input.stereo.left.gray.cols;
    request.leftImageHeight =
        context.monoMode ? monoImage.rows : input.stereo.left.gray.rows;
    request.rightImageWidth =
        context.monoMode ? 0 : input.stereo.right.gray.cols;
    request.rightImageHeight =
        context.monoMode ? 0 : input.stereo.right.gray.rows;
    request.includeFeatures = extractFeatures && !out.usedVisualFeatureFrontend;
    request.includePointCloud = extractPointCloud;
    request.maxPointCloudPoints = 120;
    return request;
}

void CopySnapshotSummary(const Core::Ports::VisualMapSnapshot &snapshot,
                         Core::Ports::SlamOutput &out)
{
    out.matchesInliers = snapshot.summary.matchesInliers;
    out.trackedMapPointCount = snapshot.summary.trackedMapPointCount;
    out.localMapPointCount = snapshot.summary.localMapPointCount;
    out.localMapPointHash = snapshot.summary.localMapPointHash;
    out.matchedMapPointHashBeforePoseOptimization =
        snapshot.summary.matchedMapPointHashBeforePoseOptimization;
    out.trackedMapPointHash = snapshot.summary.trackedMapPointHash;
    out.closeMapPointCount = snapshot.summary.closeMapPointCount;
}

void CopySnapshotFeatures(const TrackingBackendContext &context,
                          Core::Ports::VisualMapSnapshot &snapshot,
                          Core::Ports::SlamOutput &out)
{
    if (context.inputMode == SlamInputMode::MonoRight) {
        out.rightFeatures = std::move(snapshot.features.leftFeatures);
        return;
    }
    out.leftFeatures = std::move(snapshot.features.leftFeatures);
    out.rightFeatures = std::move(snapshot.features.rightFeatures);
}

void ExtractVisualSnapshot(const TrackingBackendContext &context,
                           const Core::Ports::SlamInputBatch &input,
                           bool extractFeatures, bool extractPointCloud,
                           Core::Ports::SlamOutput &out)
{
    const bool needVisualExtraction = extractPointCloud || extractFeatures;
    if (!needVisualExtraction || context.visualProvider == nullptr) {
        return;
    }

    Core::Ports::VisualMapSnapshotRequest request = BuildSnapshotRequest(
        context, input, extractFeatures, extractPointCloud, out);
    Core::Ports::VisualMapSnapshot snapshot =
        context.visualProvider->ExtractVisualMapSnapshot(request);
    CopySnapshotSummary(snapshot, out);
    if (request.includeFeatures) {
        CopySnapshotFeatures(context, snapshot, out);
    }
    if (extractPointCloud) {
        out.pointCloudXyz = std::move(snapshot.pointCloud.xyz);
    }
}

} // namespace

Core::Ports::SlamOutput
RunSlamTrackingBackend(SlamEngineAdapter &engine,
                       const Core::Ports::SlamInputBatch &input,
                       bool extractFeatures, bool extractPointCloud,
                       const StereoFeatureTrackRequest *stereoFeatureRequest)
{
    Core::Ports::SlamOutput out{};
    const TrackingBackendContext context = BuildTrackingBackendContext(engine);
    if (!CanTrack(context)) {
        return out;
    }

    const bool trackWithStereoFeatures = CanTrackWithStereoFeatures(
        stereoFeatureRequest, context.monoMode);
    if (!trackWithStereoFeatures &&
        context.state->m_lastVisualFeatureImageCount == 0) {
        context.state->ResetVisualFeatureStats();
    }

    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    out.usedVisualFeatureFrontend = false;
    context.state->CopyVisualFeatureStatsToOutput(out);

    Sophus::SE3f tcw;
    if (trackWithStereoFeatures) {
        tcw = TrackPreparedStereoFeatures(context, input, *stereoFeatureRequest,
                                          out);
    } else {
        tcw = TrackRawInput(context, input, out);
    }

    CopyBackendStatsToOutput(context.backend->GetBackendStats(), out);
    FillOutputPose(context, input, tcw, out);
    ApplyRealtimePoseContinuity(context, input.frameTimeSec, out);
    ApplyPoseStabilizer(context, input.frameTimeSec, out);
    NormalizeRealtimePublishState(out);
    ExtractVisualSnapshot(context, input, extractFeatures, extractPointCloud,
                          out);
    return out;
}

} // namespace SmartDrone::Adapters::Slam
