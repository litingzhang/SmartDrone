#pragma once

#include <cstdint>

#include "core/application/config/runtime_app_types.h"

namespace SmartDrone::Core::Application {

struct ObstacleAvoidanceConfig {
    bool enabled{true};
    bool holdOnStaleCloud{false};
    float radiusM{0.75f};
    float lookaheadM{2.0f};
    float speedLookaheadS{0.0f};
    float nearFieldRadiusM{0.0f};
    int maxPointCloudAgeMs{600};
    int minCloudPoints{1};
    int minBlockingPoints{1};
};

float ObstacleAvoidanceLookaheadForSpeed(
    const ObstacleAvoidanceConfig &config, float speedMps);

ObstacleAvoidanceConfig ClampObstacleAvoidanceConfig(
    ObstacleAvoidanceConfig config);
ObstacleAvoidanceConfig ObstacleAvoidanceConfigFromRuntime(
    const RuntimeConfig &runtime);
ObstacleAvoidanceConfig ObstacleAvoidanceConfigFromTuning(
    const LiveRuntimeTuning &tuning);
ObstacleAvoidanceConfig ReadObstacleAvoidanceConfig();
uint64_t ObstacleAvoidanceMaxPointCloudAgeUs(
    const ObstacleAvoidanceConfig &config);

} // namespace SmartDrone::Core::Application
