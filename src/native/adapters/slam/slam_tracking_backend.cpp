#include "adapters/slam/slam_tracking_backend.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <utility>

#include <sophus/se3.hpp>

#include "adapters/slam/slam_engine_access.h"
#include "adapters/slam/slam_env.h"
#include "adapters/slam/slam_output_utils.h"
#include "adapters/slam/slam_pose_utils.h"
#include "core/ports/slam_tracking_state.h"

namespace smartdrone::adapters::slam {

namespace {

void NormalizeRealtimePublishState(core::ports::SlamOutput &out) {
  if (!out.poseValid || !out.pose.valid || IsIdentityPose(out.pose) ||
      !EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_CONTINUITY_MARK_RECENTLY_LOST",
                      true)) {
    return;
  }
  if (out.trackingState != core::ports::kSlamTrackingOk &&
      out.trackingState != core::ports::kSlamTrackingRecentlyLost) {
    out.trackingState = core::ports::kSlamTrackingRecentlyLost;
  }
}

} // namespace

core::ports::SlamOutput
RunSlamTrackingBackend(SlamEngineAdapter &engine,
                       const core::ports::SlamInputBatch &input,
                       bool extractFeatures, bool extractPointCloud,
                       const StereoFeatureTrackRequest *stereoFeatureRequest) {
  core::ports::SlamOutput out{};
  core::ports::ISlamTrackingBackend *backend =
      SlamEngineAccess::TrackingBackend(engine);
  if (backend == nullptr || !backend->Available()) {
    return out;
  }
  core::ports::ITrackedVisualDataProvider *visualProvider =
      SlamEngineAccess::TrackedVisualDataProvider(engine);

  SlamModeSharedState &state = SlamEngineAccess::ModeState(engine);
  const SlamInputMode inputMode = SlamEngineAccess::InputMode(engine);
  const bool useImu = SlamEngineAccess::UseImu(engine);
  const bool monoMode = (inputMode != SlamInputMode::Stereo);
  const cv::Mat &monoImage = (inputMode == SlamInputMode::MonoRight)
                                 ? input.stereo.right.gray
                                 : input.stereo.left.gray;

  const bool trackWithStereoFeatures =
      stereoFeatureRequest != nullptr && stereoFeatureRequest->enabled &&
      !monoMode && !stereoFeatureRequest->leftPrepared.empty() &&
      !stereoFeatureRequest->rightPrepared.empty();
  if (!trackWithStereoFeatures && state.m_lastVisualFeatureImageCount == 0) {
    state.ResetVisualFeatureStats();
  }

  out.frameId = input.frameId;
  out.captureTimestampNs = input.captureTimestampNs;
  out.usedVisualFeatureFrontend = false;
  state.CopyVisualFeatureStatsToOutput(out);

  Sophus::SE3f tcw;
  if (trackWithStereoFeatures) {
    out.inputPrepareMs = stereoFeatureRequest->inputPrepareMs;
    out.frontendMs = stereoFeatureRequest->frontendMs;
    out.stereoPairMs = stereoFeatureRequest->stereoPairMs;
    out.featurePackMs = stereoFeatureRequest->featurePackMs;
    out.monoAugmentMs = stereoFeatureRequest->monoAugmentMs;
    state.m_lastVisualFeatureObservationHash =
        stereoFeatureRequest->observationHash;

    const auto trackStartTp = std::chrono::steady_clock::now();
    core::ports::PreparedStereoFeatureTrackRequest trackRequest;
    trackRequest.input = &input;
    trackRequest.leftPrepared = &stereoFeatureRequest->leftPrepared;
    trackRequest.rightPrepared = &stereoFeatureRequest->rightPrepared;
    trackRequest.observations = &stereoFeatureRequest->observations;
    trackRequest.useImu = useImu;
    tcw = backend->TrackPreparedStereoWithFeatures(trackRequest);
    const auto trackEndTp = std::chrono::steady_clock::now();
    backend->LogStereoFeatureDiagnostics(input.frameId,
                                         stereoFeatureRequest->observations);
    if (stereoFeatureRequest->recordTotalMs) {
      state.m_lastVisualFeatureTotalMs =
          std::chrono::duration<double, std::milli>(
              trackEndTp - stereoFeatureRequest->totalStartTp)
              .count();
    }
    out.orbTrackMs =
        std::chrono::duration<double, std::milli>(trackEndTp - trackStartTp)
            .count();
    out.frontendMs = state.m_lastVisualFeatureFrontendMs > 0.0
                         ? state.m_lastVisualFeatureFrontendMs
                         : out.frontendMs;
    state.CopyVisualFeatureStatsToOutput(out);
    out.usedVisualFeatureFrontend = true;
    out.leftFeatures = stereoFeatureRequest->leftFeaturePoints;
    out.rightFeatures = stereoFeatureRequest->rightFeaturePoints;
  } else {
    const auto orbTrackStartTp = std::chrono::steady_clock::now();
    core::ports::SlamTrackRequest trackRequest;
    trackRequest.input = &input;
    trackRequest.inputMode = inputMode;
    trackRequest.useImu = useImu;
    tcw = backend->TrackRaw(trackRequest);
    const auto orbTrackEndTp = std::chrono::steady_clock::now();
    out.orbTrackMs = std::chrono::duration<double, std::milli>(orbTrackEndTp -
                                                               orbTrackStartTp)
                         .count();
  }

  CopyBackendStatsToOutput(backend->GetBackendStats(), out);

  Sophus::SE3f twc = tcw.inverse();
  bool trajectoryPoseLost = false;
  if (EnvFlagEnabled("SMART_DRONE_ORB_LIVE_EUROC_TRAJECTORY_POSE", false)) {
    Sophus::SE3f trajectoryTwc;
    if (backend->GetLatestFrameTrajectoryPoseEuRoC(trajectoryTwc, nullptr,
                                                   &trajectoryPoseLost)) {
      twc = trajectoryTwc;
    }
  }
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

  SlamEngineAccess::GateRealtimePoseQuality(engine, out, input.frameTimeSec);
  out.pose.valid = out.poseValid;
  if (EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_CONTINUITY", true)) {
    SlamEngineAccess::MaintainRealtimePoseContinuity(
        engine, out.pose, out.poseValid, input.frameTimeSec, out.trackingState);
    if (out.poseValid && !TrackingStateCanPublishPose(out.trackingState) &&
        !IsIdentityPose(out.pose) &&
        EnvFlagEnabled(
            "SMART_DRONE_REALTIME_POSE_CONTINUITY_MARK_RECENTLY_LOST", true)) {
      out.trackingState = core::ports::kSlamTrackingRecentlyLost;
    }
    out.pose.valid = out.poseValid;
  }

  if (EnvFlagEnabled("SMART_DRONE_POSE_STABILIZER", false)) {
    SlamEngineAccess::StabilizeOutputPose(
        engine, out.pose, out.poseValid, input.frameTimeSec, out.trackingState);
    NormalizeRealtimePublishState(out);
  }
  NormalizeRealtimePublishState(out);

  const bool needVisualExtraction = extractPointCloud || extractFeatures;
  if (!needVisualExtraction) {
    return out;
  }

  const int leftWidth = monoMode ? monoImage.cols : input.stereo.left.gray.cols;
  const int leftHeight =
      monoMode ? monoImage.rows : input.stereo.left.gray.rows;
  if (visualProvider == nullptr) {
    return out;
  }
  core::ports::VisualMapSnapshotRequest snapshotRequest;
  snapshotRequest.leftImageWidth = leftWidth;
  snapshotRequest.leftImageHeight = leftHeight;
  snapshotRequest.rightImageWidth = monoMode ? 0 : input.stereo.right.gray.cols;
  snapshotRequest.rightImageHeight =
      monoMode ? 0 : input.stereo.right.gray.rows;
  snapshotRequest.includeFeatures = extractFeatures &&
                                    !out.usedVisualFeatureFrontend;
  snapshotRequest.includePointCloud = extractPointCloud;
  snapshotRequest.maxPointCloudPoints = 120;
  core::ports::VisualMapSnapshot snapshot =
      visualProvider->ExtractVisualMapSnapshot(snapshotRequest);
  out.matchesInliers = snapshot.summary.matchesInliers;
  out.trackedMapPointCount = snapshot.summary.trackedMapPointCount;
  out.localMapPointCount = snapshot.summary.localMapPointCount;
  out.localMapPointHash = snapshot.summary.localMapPointHash;
  out.matchedMapPointHashBeforePoseOptimization =
      snapshot.summary.matchedMapPointHashBeforePoseOptimization;
  out.trackedMapPointHash = snapshot.summary.trackedMapPointHash;
  out.closeMapPointCount = snapshot.summary.closeMapPointCount;
  if (snapshotRequest.includeFeatures) {
    if (inputMode == SlamInputMode::MonoRight) {
      out.rightFeatures = std::move(snapshot.features.leftFeatures);
    } else {
      out.leftFeatures = std::move(snapshot.features.leftFeatures);
      out.rightFeatures = std::move(snapshot.features.rightFeatures);
    }
  }
  if (extractPointCloud) {
    out.pointCloudXyz = std::move(snapshot.pointCloud.xyz);
  }
  return out;
}

} // namespace smartdrone::adapters::slam
