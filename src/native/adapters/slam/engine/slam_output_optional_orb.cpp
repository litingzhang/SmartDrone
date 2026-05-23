#include "adapters/slam/engine/slam_output_utils.h"

#include "adapters/slam/engine/slam_engine_access.h"
#include "adapters/slam/engine/slam_env.h"

namespace SmartDrone::Adapters::Slam {

void CopyOptionalMapSummary(SlamEngineAdapter *engine,
                            Core::Ports::SlamOutput &out)
{
    if (engine == nullptr) {
        return;
    }
    Core::Ports::ISlamTrackingBackend *backend =
        SlamEngineAccess::TrackingBackend(*engine);
    if (backend != nullptr) {
        CopyMapSummaryToOutput(backend->GetMapSummary(), out);
    }
}

void MaintainOptionalRealtimeContinuity(
    SlamEngineAdapter *engine, Core::Ports::SlamOutput &out,
    const Core::Ports::SlamInputBatch &input)
{
    if (engine == nullptr ||
        !EnvFlagEnabled("SMART_DRONE_REALTIME_POSE_CONTINUITY", true)) {
        return;
    }
    SlamEngineAccess::MaintainRealtimePoseContinuity(
        *engine, out.pose, out.poseValid, input.frameTimeSec,
        out.trackingState);
    out.pose.valid = out.poseValid;
}

} // namespace SmartDrone::Adapters::Slam
