#include "adapters/slam/slam_output_utils.h"

#include "adapters/slam/slam_env.h"
#include "core/ports/slam_tracking_state.h"

#if defined(SMART_DRONE_ENABLE_ORB_SLAM3)
#include "adapters/slam/orb_slam3_backend.h"
#include "adapters/slam/slam_engine_access.h"
#endif

namespace smartdrone::adapters::slam {

core::ports::SlamOutput MakeOkSlamOutput(const core::ports::SlamInputBatch &input,
                                         unsigned long mapId)
{
    core::ports::SlamOutput out{};
    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    out.mapId = mapId;
    out.trackingState = core::ports::kSlamTrackingOk;
    out.poseValid = true;
    out.pose.valid = true;
    out.usedSuperPointFrontend = false;
    return out;
}

void MarkSlamOutputPoseLost(core::ports::SlamOutput &out, int trackingState)
{
    out.trackingState = trackingState;
    out.poseValid = false;
    out.pose.valid = false;
}

core::ports::SlamOutput MakePoseLostSlamOutput(SlamEngineAdapter *engine,
                                               const core::ports::SlamInputBatch &input,
                                               int trackingState,
                                               bool copyOrbMapSummary,
                                               bool maintainRealtimeContinuity)
{
    core::ports::SlamOutput out{};
    out.frameId = input.frameId;
    out.captureTimestampNs = input.captureTimestampNs;
    MarkSlamOutputPoseLost(out, trackingState);
#if defined(SMART_DRONE_ENABLE_ORB_SLAM3)
    if (copyOrbMapSummary && engine != nullptr) {
        if (OrbSlam3Backend *backend = SlamEngineAccess::OrbBackend(*engine)) {
            backend->CopyMapSummaryToOutput(out);
        }
    }
    if (maintainRealtimeContinuity && engine != nullptr &&
        EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_CONTINUITY", true)) {
        SlamEngineAccess::MaintainRealtimePoseContinuity(*engine, out.pose, out.poseValid,
                                                         input.frameTimeSec, out.trackingState);
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
