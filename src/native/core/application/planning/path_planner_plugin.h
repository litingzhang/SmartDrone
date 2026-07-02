#pragma once

#include <string>
#include <vector>

#include "core/application/runtime/local_occupancy_map.h"

namespace SmartDrone::Core::Application {

struct PathPlannerPoint {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

struct PathPlannerConfig {
    float voxelSizeM{0.3f};
    float clearanceM{0.6f};
    float searchPaddingM{2.0f};
    int maxExpandedNodes{20000};
};

struct PathPlannerRequest {
    PathPlannerPoint start{};
    PathPlannerPoint goal{};
    const LocalOccupancyMap *occupancyMap{nullptr};
    PathPlannerConfig config{};
};

struct PathPlannerResult {
    bool success{false};
    std::string reason;
    std::vector<PathPlannerPoint> waypoints;
    int expandedNodes{0};
};

class IPathPlannerPlugin {
  public:
    virtual ~IPathPlannerPlugin() = default;

    virtual PathPlannerResult PlanPath(
        const PathPlannerRequest &request) const = 0;
};

class FastPlannerGridPathPlannerPlugin final : public IPathPlannerPlugin {
  public:
    PathPlannerResult PlanPath(
        const PathPlannerRequest &request) const override;
};

} // namespace SmartDrone::Core::Application
