#include "core/application/runtime/obstacle_avoidance_config.h"

#include <algorithm>
#include <atomic>

#include "common/environment.h"

namespace SmartDrone::Core::Application {
namespace {

int ClampIntValue(int value, int low, int high)
{
    return std::max(low, std::min(high, value));
}

float ClampFloatValue(float value, float low, float high)
{
    return std::max(low, std::min(high, value));
}

} // namespace

ObstacleAvoidanceConfig ReadObstacleAvoidanceConfig()
{
    ObstacleAvoidanceConfig config{};
    config.enabled = SmartDrone::Common::EnvFlagEnabled(
        "SMART_DRONE_AVOIDANCE_ENABLE", true);
    config.holdOnStaleCloud = SmartDrone::Common::EnvFlagEnabled(
        "SMART_DRONE_AVOIDANCE_HOLD_ON_STALE_CLOUD", false);
    config.radiusM = SmartDrone::Common::EnvFloatValueClamped(
        "SMART_DRONE_AVOIDANCE_RADIUS_M", 0.75f, 0.2f, 3.0f);
    config.lookaheadM = SmartDrone::Common::EnvFloatValueClamped(
        "SMART_DRONE_AVOIDANCE_LOOKAHEAD_M", 2.0f, 0.5f, 8.0f);
    config.speedLookaheadS = SmartDrone::Common::EnvFloatValueClamped(
        "SMART_DRONE_AVOIDANCE_SPEED_LOOKAHEAD_S", 0.0f, 0.0f, 5.0f);
    config.nearFieldRadiusM = SmartDrone::Common::EnvFloatValueClamped(
        "SMART_DRONE_AVOIDANCE_NEAR_FIELD_RADIUS_M", 0.0f, 0.0f, 3.0f);
    config.maxPointCloudAgeMs = SmartDrone::Common::EnvIntValueClamped(
        "SMART_DRONE_AVOIDANCE_MAX_POINT_AGE_MS", 600, 50, 5000);
    config.minCloudPoints = SmartDrone::Common::EnvIntValueClamped(
        "SMART_DRONE_AVOIDANCE_MIN_CLOUD_POINTS", 1, 1, 5000);
    config.minBlockingPoints = SmartDrone::Common::EnvIntValueClamped(
        "SMART_DRONE_AVOIDANCE_MIN_BLOCKING_POINTS", 1, 1, 50);
    return config;
}

ObstacleAvoidanceConfig ClampObstacleAvoidanceConfig(
    ObstacleAvoidanceConfig config)
{
    config.radiusM = ClampFloatValue(config.radiusM, 0.2f, 3.0f);
    config.lookaheadM = ClampFloatValue(config.lookaheadM, 0.5f, 8.0f);
    config.speedLookaheadS =
        ClampFloatValue(config.speedLookaheadS, 0.0f, 5.0f);
    config.nearFieldRadiusM =
        ClampFloatValue(config.nearFieldRadiusM, 0.0f, 3.0f);
    config.maxPointCloudAgeMs =
        ClampIntValue(config.maxPointCloudAgeMs, 50, 5000);
    config.minCloudPoints = ClampIntValue(config.minCloudPoints, 1, 5000);
    config.minBlockingPoints =
        ClampIntValue(config.minBlockingPoints, 1, 50);
    return config;
}

ObstacleAvoidanceConfig ObstacleAvoidanceConfigFromRuntime(
    const RuntimeConfig &runtime)
{
    ObstacleAvoidanceConfig config{};
    config.enabled = runtime.avoidanceEnabled;
    config.holdOnStaleCloud = runtime.avoidanceHoldOnStaleCloud;
    config.radiusM = runtime.avoidanceRadiusM;
    config.lookaheadM = runtime.avoidanceLookaheadM;
    config.speedLookaheadS = runtime.avoidanceSpeedLookaheadS;
    config.nearFieldRadiusM = runtime.avoidanceNearFieldRadiusM;
    config.maxPointCloudAgeMs = runtime.avoidanceMaxPointCloudAgeMs;
    config.minCloudPoints = runtime.avoidanceMinCloudPoints;
    config.minBlockingPoints = runtime.avoidanceMinBlockingPoints;
    return ClampObstacleAvoidanceConfig(config);
}

ObstacleAvoidanceConfig ObstacleAvoidanceConfigFromTuning(
    const LiveRuntimeTuning &tuning)
{
    ObstacleAvoidanceConfig config{};
    config.enabled =
        tuning.avoidanceEnabled.load(std::memory_order_relaxed);
    config.holdOnStaleCloud =
        tuning.avoidanceHoldOnStaleCloud.load(std::memory_order_relaxed);
    config.radiusM = tuning.avoidanceRadiusM.load(std::memory_order_relaxed);
    config.lookaheadM =
        tuning.avoidanceLookaheadM.load(std::memory_order_relaxed);
    config.speedLookaheadS =
        tuning.avoidanceSpeedLookaheadS.load(std::memory_order_relaxed);
    config.nearFieldRadiusM =
        tuning.avoidanceNearFieldRadiusM.load(std::memory_order_relaxed);
    config.maxPointCloudAgeMs =
        tuning.avoidanceMaxPointCloudAgeMs.load(std::memory_order_relaxed);
    config.minCloudPoints =
        tuning.avoidanceMinCloudPoints.load(std::memory_order_relaxed);
    config.minBlockingPoints =
        tuning.avoidanceMinBlockingPoints.load(std::memory_order_relaxed);
    return ClampObstacleAvoidanceConfig(config);
}

uint64_t ObstacleAvoidanceMaxPointCloudAgeUs(
    const ObstacleAvoidanceConfig &config)
{
    return static_cast<uint64_t>(config.maxPointCloudAgeMs) * 1000ULL;
}

float ObstacleAvoidanceLookaheadForSpeed(
    const ObstacleAvoidanceConfig &config, float speedMps)
{
    const float dynamicLookahead =
        std::max(0.0f, speedMps) * config.speedLookaheadS;
    return std::min(8.0f, std::max(config.lookaheadM, dynamicLookahead));
}

} // namespace SmartDrone::Core::Application
