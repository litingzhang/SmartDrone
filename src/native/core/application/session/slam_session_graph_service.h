#pragma once

#include <atomic>

#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/session/slam_processing_support.h"
#include "core/application/state/live_pose_state.h"

namespace smartdrone::core::application {

bool RunSlamSessionGraph(const UnifiedConfig &cfg, LiveRuntimeTuning &tuning, Px4MavlinkGateway &mav,
                         std::atomic<bool> &stop, LivePoseState &livePose, std::atomic<bool> &runningFlag);

} // namespace smartdrone::core::application
