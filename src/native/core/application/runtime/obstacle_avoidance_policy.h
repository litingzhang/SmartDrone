#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "common/tlv/goal_cache.h"
#include "core/application/runtime/obstacle_avoidance_config.h"
#include "core/application/state/live_pose_types.h"

namespace SmartDrone::Core::Application {

struct AvoidanceSnapshot {
    bool poseValid{false};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float qw{1.0f};
    float qx{0.0f};
    float qy{0.0f};
    float qz{0.0f};
    std::shared_ptr<const std::vector<float>> pointCloudXyz;
    uint32_t pointCloudSeq{0};
    uint64_t pointCloudUpdateUs{0};
};

struct AvoidanceDecision {
    bool shouldHold{false};
    std::string reason;
    float nearestObstacleM{0.0f};
    AvoidanceHoldReason holdReason{AvoidanceHoldReason::None};
};

using ReadAvoidanceSnapshotFn = std::function<bool(AvoidanceSnapshot &)>;

class IAvoidanceAlgorithmPlugin {
  public:
    virtual ~IAvoidanceAlgorithmPlugin() = default;

    virtual AvoidanceDecision EvaluateMoveGoal(
        const MoveGoal &goal, const AvoidanceSnapshot &snapshot) const = 0;
};

class ObstacleAvoidancePolicy final : public IAvoidanceAlgorithmPlugin {
  public:
    ObstacleAvoidancePolicy() = default;
    explicit ObstacleAvoidancePolicy(const LiveRuntimeTuning *tuning);

    AvoidanceDecision EvaluateMoveGoal(const MoveGoal &goal,
                                       const AvoidanceSnapshot &snapshot) const override;

  private:
    ObstacleAvoidanceConfig ReadConfig() const;

    const LiveRuntimeTuning *m_tuning{nullptr};
};

} // namespace SmartDrone::Core::Application
