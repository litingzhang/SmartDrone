#include "adapters/slam/slam_output_utils.h"

#include "adapters/slam/slam_env.h"
#include "core/ports/slam_tracking_state.h"

#if defined(SMART_DRONE_ENABLE_ORB_SLAM3)
#include "adapters/slam/slam_engine_access.h"
#endif

namespace smartdrone::adapters::slam {

core::ports::SlamOutput
MakeOkSlamOutput(const core::ports::SlamInputBatch &input,
                 unsigned long mapId) {
  core::ports::SlamOutput out{};
  out.frameId = input.frameId;
  out.captureTimestampNs = input.captureTimestampNs;
  out.mapId = mapId;
  out.trackingState = core::ports::kSlamTrackingOk;
  out.poseValid = true;
  out.pose.valid = true;
  out.usedVisualFeatureFrontend = false;
  return out;
}

void MarkSlamOutputPoseLost(core::ports::SlamOutput &out, int trackingState) {
  out.trackingState = trackingState;
  out.poseValid = false;
  out.pose.valid = false;
}

void CopyMapSummaryToOutput(const core::ports::SlamMapSummary &summary,
                            core::ports::SlamOutput &out) {
  out.mapId = summary.mapId;
  out.matchesInliers = summary.matchesInliers;
  out.trackedMapPointCount = summary.trackedMapPointCount;
  out.localMapPointCount = summary.localMapPointCount;
  out.localMapPointHash = summary.localMapPointHash;
  out.matchedMapPointHashBeforePoseOptimization =
      summary.matchedMapPointHashBeforePoseOptimization;
  out.trackedMapPointHash = summary.trackedMapPointHash;
}

void CopyBackendStatsToOutput(const core::ports::SlamBackendStats &stats,
                              core::ports::SlamOutput &out) {
  CopyMapSummaryToOutput(stats.map, out);
  out.trackingState = stats.frame.trackingState;
  out.orbExtractMs = stats.frame.featureExtractMs;
  out.orbStereoMatchMs = stats.frame.stereoMatchMs;
  out.closeMapPointCount = stats.frame.closeMapPointCount;
  out.orbFrameId = stats.frame.frameId;
  out.referenceKeyFrameId = stats.frame.referenceKeyFrameId;
  out.lastKeyFrameId = stats.frame.lastKeyFrameId;
  out.lastKeyFrameFrameId = stats.frame.lastKeyFrameFrameId;
  out.keyFramesInMap = stats.frame.keyFramesInMap;
  out.stereoFeatureInitFrameId = stats.frame.stereoFeatureInitFrameId;
  out.stereoFeatureInjected = stats.frame.stereoFeatureInjected;
  out.stereoFeatureBootstrap = stats.frame.stereoFeatureBootstrap;
  out.stereoFeatureStabilizing = stats.frame.stereoFeatureStabilizing;
  out.localMappingWaitMs = stats.localMappingWait.waitMs;
  out.localMappingWaitQueueBefore = stats.localMappingWait.queueBefore;
  out.localMappingWaitQueueAfter = stats.localMappingWait.queueAfter;
  out.localMappingWaitTimeoutMs = stats.localMappingWait.timeoutMs;
  out.localMappingWaitRequested = stats.localMappingWait.requested;
  out.localMappingWaitTimedOut = stats.localMappingWait.timedOut;
  out.localMappingAcceptingBefore = stats.localMappingWait.acceptingBefore;
  out.localMappingAcceptingAfter = stats.localMappingWait.acceptingAfter;
}

core::ports::SlamOutput
MakePoseLostSlamOutput(SlamEngineAdapter *engine,
                       const core::ports::SlamInputBatch &input,
                       int trackingState, bool copyOrbMapSummary,
                       bool maintainRealtimeContinuity) {
  core::ports::SlamOutput out{};
  out.frameId = input.frameId;
  out.captureTimestampNs = input.captureTimestampNs;
  MarkSlamOutputPoseLost(out, trackingState);
#if defined(SMART_DRONE_ENABLE_ORB_SLAM3)
  if (copyOrbMapSummary && engine != nullptr) {
    if (core::ports::ISlamTrackingBackend *backend =
            SlamEngineAccess::TrackingBackend(*engine)) {
      CopyMapSummaryToOutput(backend->GetMapSummary(), out);
    }
  }
  if (maintainRealtimeContinuity && engine != nullptr &&
      EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_CONTINUITY", true)) {
    SlamEngineAccess::MaintainRealtimePoseContinuity(
        *engine, out.pose, out.poseValid, input.frameTimeSec,
        out.trackingState);
    out.pose.valid = out.poseValid;
  }
#else
  (void)engine;
  (void)copyOrbMapSummary;
  (void)maintainRealtimeContinuity;
#endif
  return out;
}

} // namespace smartdrone::adapters::slam
