#pragma once

#include <atomic>

#include "core/application/config/runtime_app_types.h"
#include "core/application/state/live_pose_state.h"

namespace smartdrone::core::application {

bool RunCalibSessionGraph(const UnifiedConfig &cfg, std::atomic<bool> &stop, LivePoseState &livePose,
                          std::atomic<bool> &runningFlag);

} // namespace smartdrone::core::application
