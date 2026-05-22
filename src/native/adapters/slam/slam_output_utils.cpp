#include "adapters/slam/slam_output_utils.h"

#include "core/ports/slam_tracking_state.h"

namespace SmartDrone::Adapters::Slam {

Core::Ports::SlamOutput
MakeOkSlamOutput(const Core::Ports::SlamInputBatch &input,
                 unsigned long mapId)
{
    Core::Ports::SlamOutput out{};
    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    out.mapId = mapId;
    out.trackingState = Core::Ports::kSlamTrackingOk;
    out.poseValid = true;
    out.pose.valid = true;
    out.usedVisualFeatureFrontend = false;
    return out;
}

void MarkSlamOutputPoseLost(Core::Ports::SlamOutput &out, int trackingState)
{
    out.trackingState = trackingState;
    out.poseValid = false;
    out.pose.valid = false;
}

void CopyMapSummaryToOutput(const Core::Ports::SlamMapSummary &summary,
                            Core::Ports::SlamOutput &out)
{
    out.mapId = summary.mapId;
    out.matchesInliers = summary.matchesInliers;
    out.trackedMapPointCount = summary.trackedMapPointCount;
    out.localMapPointCount = summary.localMapPointCount;
    out.localMapPointHash = summary.localMapPointHash;
    out.matchedMapPointHashBeforePoseOptimization =
        summary.matchedMapPointHashBeforePoseOptimization;
    out.trackedMapPointHash = summary.trackedMapPointHash;
}

void CopyBackendStatsToOutput(const Core::Ports::SlamBackendStats &stats,
                              Core::Ports::SlamOutput &out)
{
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

Core::Ports::SlamOutput
MakePoseLostSlamOutput(SlamEngineAdapter *engine,
                       const Core::Ports::SlamInputBatch &input,
                       int trackingState, bool copyOrbMapSummary,
                       bool maintainRealtimeContinuity)
{
    Core::Ports::SlamOutput out{};
    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    MarkSlamOutputPoseLost(out, trackingState);
    if (copyOrbMapSummary && engine != nullptr) {
        CopyOptionalMapSummary(engine, out);
    }
    if (maintainRealtimeContinuity && engine != nullptr) {
        MaintainOptionalRealtimeContinuity(engine, out, input);
    }
    return out;
}

} // namespace SmartDrone::Adapters::Slam
