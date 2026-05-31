#include "core/application/runtime/obstacle_avoidance_policy.h"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstdio>
#include <limits>

#include "core/application/runtime/local_occupancy_map.h"
#include "core/application/runtime/obstacle_avoidance_config.h"
#include "common/time_utils.h"

namespace SmartDrone::Core::Application {
namespace {

struct Point3 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

struct MotionProbe {
    Point3 origin{};
    Point3 direction{};
    float length{0.0f};
};

enum class CloudStatus {
    Fresh,
    Missing,
    Sparse,
    Stale,
};

CloudStatus CheckCloudStatus(const AvoidanceSnapshot &snapshot,
                             const ObstacleAvoidanceConfig &config)
{
    if (!snapshot.poseValid || !snapshot.pointCloudXyz ||
        snapshot.pointCloudXyz->size() < 3) {
        return CloudStatus::Missing;
    }
    if (snapshot.pointCloudXyz->size() / 3 <
        static_cast<size_t>(config.minCloudPoints)) {
        return CloudStatus::Sparse;
    }
    if (snapshot.pointCloudUpdateUs == 0) {
        return CloudStatus::Missing;
    }
    const uint64_t nowUs = MonoTimeUs();
    if (snapshot.pointCloudUpdateUs >= nowUs ||
        nowUs - snapshot.pointCloudUpdateUs <=
            ObstacleAvoidanceMaxPointCloudAgeUs(config)) {
        return CloudStatus::Fresh;
    }
    return CloudStatus::Stale;
}

AvoidanceDecision CloudStatusDecision(CloudStatus status)
{
    if (status == CloudStatus::Sparse) {
        return {true, "avoidance hold: point cloud sparse", 0.0f,
                AvoidanceHoldReason::PointCloudSparse};
    }
    if (status == CloudStatus::Stale) {
        return {true, "avoidance hold: point cloud stale", 0.0f,
                AvoidanceHoldReason::PointCloudStale};
    }
    return {true, "avoidance hold: point cloud unavailable", 0.0f,
            AvoidanceHoldReason::PointCloudUnavailable};
}

float Norm(Point3 value)
{
    return std::sqrt(value.x * value.x + value.y * value.y +
                     value.z * value.z);
}

MotionProbe BuildVelocityProbe(const MoveGoal &goal,
                               const AvoidanceSnapshot &snapshot,
                               const ObstacleAvoidanceConfig &config)
{
    const Point3 velocity{goal.vx, goal.vy, goal.vz};
    const float speed = Norm(velocity);
    MotionProbe probe{};
    probe.origin = {snapshot.x, snapshot.y, snapshot.z};
    if (!(speed > 0.05f)) {
        return probe;
    }
    probe.direction = {velocity.x / speed, velocity.y / speed,
                       velocity.z / speed};
    probe.length = ObstacleAvoidanceLookaheadForSpeed(config, speed);
    return probe;
}

MotionProbe BuildPositionProbe(const MoveGoal &goal,
                               const AvoidanceSnapshot &snapshot,
                               const ObstacleAvoidanceConfig &config)
{
    const Point3 delta{goal.x - snapshot.x, goal.y - snapshot.y,
                       goal.z - snapshot.z};
    const float distance = Norm(delta);
    MotionProbe probe{};
    probe.origin = {snapshot.x, snapshot.y, snapshot.z};
    if (!(distance > 0.05f)) {
        return probe;
    }
    probe.direction = {delta.x / distance, delta.y / distance,
                       delta.z / distance};
    probe.length = std::min(distance, config.lookaheadM);
    return probe;
}

float SnapshotYawRad(const AvoidanceSnapshot &snapshot)
{
    const float siny = 2.0f * (snapshot.qw * snapshot.qz +
                               snapshot.qx * snapshot.qy);
    const float cosy = 1.0f - 2.0f * (snapshot.qy * snapshot.qy +
                                      snapshot.qz * snapshot.qz);
    return std::atan2(siny, cosy);
}

MotionProbe BuildRcJoystickProbe(const MoveGoal &goal,
                                 const AvoidanceSnapshot &snapshot,
                                 const ObstacleAvoidanceConfig &config)
{
    const float forward = goal.pitchNorm * goal.maxV;
    const float right = goal.rollNorm * goal.maxV;
    const float down = -goal.throttleNorm * goal.maxV;
    const float yaw = SnapshotYawRad(snapshot);
    const Point3 velocity{
        forward * std::cos(yaw) - right * std::sin(yaw),
        forward * std::sin(yaw) + right * std::cos(yaw),
        down};
    MoveGoal velocityGoal{};
    velocityGoal.isVelocity = true;
    velocityGoal.vx = velocity.x;
    velocityGoal.vy = velocity.y;
    velocityGoal.vz = velocity.z;
    return BuildVelocityProbe(velocityGoal, snapshot, config);
}

MotionProbe BuildMotionProbe(const MoveGoal &goal,
                             const AvoidanceSnapshot &snapshot,
                             const ObstacleAvoidanceConfig &config)
{
    if (goal.isRcJoystick) {
        return BuildRcJoystickProbe(goal, snapshot, config);
    }
    if (goal.isVelocity) {
        return BuildVelocityProbe(goal, snapshot, config);
    }
    return BuildPositionProbe(goal, snapshot, config);
}

float OccupancyVoxelSizeM(const ObstacleAvoidanceConfig &config)
{
    return std::max(0.1f, std::min(0.5f, config.radiusM * 0.5f));
}

float DistanceAlongProbe(const Point3 &point, const MotionProbe &probe,
                         float &forward)
{
    const Point3 rel{point.x - probe.origin.x, point.y - probe.origin.y,
                     point.z - probe.origin.z};
    forward = rel.x * probe.direction.x + rel.y * probe.direction.y +
              rel.z * probe.direction.z;
    const float relNorm2 = rel.x * rel.x + rel.y * rel.y + rel.z * rel.z;
    const float lateral2 = std::max(0.0f, relNorm2 - forward * forward);
    return std::sqrt(lateral2);
}

float DistanceToOrigin(const Point3 &point, const Point3 &origin)
{
    return Norm({point.x - origin.x, point.y - origin.y,
                 point.z - origin.z});
}

bool PointBlocksProbe(const Point3 &point, const MotionProbe &probe,
                      float radius, float &forward)
{
    const float lateral = DistanceAlongProbe(point, probe, forward);
    return forward >= 0.0f && forward <= probe.length && lateral <= radius;
}

std::string FormatBlockedReason(float nearestForward)
{
    char buffer[96]{};
    std::snprintf(buffer, sizeof(buffer),
                  "avoidance hold: obstacle %.2fm ahead", nearestForward);
    return std::string(buffer);
}

std::string FormatNearFieldReason(float nearestDistance)
{
    char buffer[96]{};
    std::snprintf(buffer, sizeof(buffer),
                  "avoidance hold: obstacle %.2fm near", nearestDistance);
    return std::string(buffer);
}

Point3 VoxelPoint(const OccupiedVoxel &voxel)
{
    return {voxel.x, voxel.y, voxel.z};
}

AvoidanceDecision ScanProbe(const MotionProbe &probe,
                            const std::vector<OccupiedVoxel> &voxels,
                            const ObstacleAvoidanceConfig &config)
{
    float nearestForward = std::numeric_limits<float>::infinity();
    int blockingCount = 0;
    for (const OccupiedVoxel &voxel : voxels) {
        float forward = 0.0f;
        if (PointBlocksProbe(VoxelPoint(voxel), probe, config.radiusM,
                             forward)) {
            ++blockingCount;
            nearestForward = std::min(nearestForward, forward);
        }
    }
    if (blockingCount < config.minBlockingPoints ||
        !std::isfinite(nearestForward)) {
        return {};
    }
    return {true, FormatBlockedReason(nearestForward), nearestForward,
            AvoidanceHoldReason::ObstacleAhead};
}

AvoidanceDecision ScanNearField(const AvoidanceSnapshot &snapshot,
                                const std::vector<OccupiedVoxel> &voxels,
                                const ObstacleAvoidanceConfig &config)
{
    if (!(config.nearFieldRadiusM > 0.0f)) {
        return {};
    }
    const Point3 origin{snapshot.x, snapshot.y, snapshot.z};
    float nearestDistance = std::numeric_limits<float>::infinity();
    int blockingCount = 0;
    for (const OccupiedVoxel &voxel : voxels) {
        const float distance = DistanceToOrigin(VoxelPoint(voxel), origin);
        if (distance <= config.nearFieldRadiusM) {
            ++blockingCount;
            nearestDistance = std::min(nearestDistance, distance);
        }
    }
    if (blockingCount < config.minBlockingPoints ||
        !std::isfinite(nearestDistance)) {
        return {};
    }
    return {true, FormatNearFieldReason(nearestDistance), nearestDistance,
            AvoidanceHoldReason::ObstacleNear};
}

} // namespace

ObstacleAvoidancePolicy::ObstacleAvoidancePolicy(
    const LiveRuntimeTuning *tuning)
    : m_tuning(tuning)
{
}

ObstacleAvoidanceConfig ObstacleAvoidancePolicy::ReadConfig() const
{
    if (m_tuning) {
        return ObstacleAvoidanceConfigFromTuning(*m_tuning);
    }
    return ReadObstacleAvoidanceConfig();
}

AvoidanceDecision ObstacleAvoidancePolicy::EvaluateMoveGoal(
    const MoveGoal &goal, const AvoidanceSnapshot &snapshot) const
{
    const ObstacleAvoidanceConfig config = ReadConfig();
    if (!config.enabled) {
        return {};
    }
    const CloudStatus cloudStatus = CheckCloudStatus(snapshot, config);
    if (cloudStatus != CloudStatus::Fresh) {
        return config.holdOnStaleCloud ? CloudStatusDecision(cloudStatus)
                                       : AvoidanceDecision{};
    }
    const LocalOccupancyMap map = LocalOccupancyMap::FromPointCloud(
        *snapshot.pointCloudXyz, OccupancyVoxelSizeM(config));
    const AvoidanceDecision nearFieldDecision =
        ScanNearField(snapshot, map.OccupiedVoxels(), config);
    if (nearFieldDecision.shouldHold) {
        return nearFieldDecision;
    }
    const MotionProbe probe = BuildMotionProbe(goal, snapshot, config);
    if (!(probe.length > 0.05f)) {
        return {};
    }
    return ScanProbe(probe, map.OccupiedVoxels(), config);
}

} // namespace SmartDrone::Core::Application
