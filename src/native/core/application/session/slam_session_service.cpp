#include "core/application/session/slam_session_service.h"

#include "core/application/session/slam_session_graph_service.h"

namespace smartdrone::core::application {

bool RunSlamSession(const UnifiedConfig &cfg, LiveRuntimeTuning &tuning, Px4MavlinkGateway &mav,
                    std::atomic<bool> &stop, LivePoseState &livePose, std::atomic<bool> &runningFlag)
{
    return RunSlamSessionGraph(cfg, tuning, mav, stop, livePose, runningFlag);
}

} // namespace smartdrone::core::application
