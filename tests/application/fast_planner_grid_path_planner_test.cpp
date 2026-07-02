#include "core/application/planning/path_planner_plugin.h"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace {

using SmartDrone::Core::Application::FastPlannerGridPathPlannerPlugin;
using SmartDrone::Core::Application::LocalOccupancyMap;
using SmartDrone::Core::Application::PathPlannerConfig;
using SmartDrone::Core::Application::PathPlannerPoint;
using SmartDrone::Core::Application::PathPlannerRequest;

PathPlannerConfig TestConfig()
{
    PathPlannerConfig config{};
    config.voxelSizeM = 0.5f;
    config.clearanceM = 0.25f;
    config.searchPaddingM = 1.0f;
    config.maxExpandedNodes = 5000;
    return config;
}

PathPlannerRequest MakeRequest(const LocalOccupancyMap &map)
{
    PathPlannerRequest request{};
    request.start = {0.0f, 0.0f, 0.0f};
    request.goal = {2.0f, 0.0f, 0.0f};
    request.occupancyMap = &map;
    request.config = TestConfig();
    return request;
}

float DistanceToObstacleCenter(const PathPlannerPoint &point)
{
    const float dx = point.x - 1.0f;
    const float dy = point.y;
    const float dz = point.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool KeepsObstacleClearance(const std::vector<PathPlannerPoint> &waypoints)
{
    for (const PathPlannerPoint &point : waypoints) {
        if (DistanceToObstacleCenter(point) < 0.5f) {
            return false;
        }
    }
    return true;
}

TEST(FastPlannerGridPathPlannerTest, ReturnsDirectPathWhenClear)
{
    const LocalOccupancyMap map = LocalOccupancyMap::FromPointCloud({}, 0.5f);
    const FastPlannerGridPathPlannerPlugin planner;

    const auto result = planner.PlanPath(MakeRequest(map));

    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.waypoints.size(), 2U);
    EXPECT_FLOAT_EQ(result.waypoints.front().x, 0.0f);
    EXPECT_FLOAT_EQ(result.waypoints.back().x, 2.0f);
}

TEST(FastPlannerGridPathPlannerTest, PlansAroundOccupiedVoxel)
{
    const LocalOccupancyMap map =
        LocalOccupancyMap::FromPointCloud({1.0f, 0.0f, 0.0f}, 0.5f);
    const FastPlannerGridPathPlannerPlugin planner;

    const auto result = planner.PlanPath(MakeRequest(map));

    ASSERT_TRUE(result.success);
    EXPECT_GT(result.waypoints.size(), 2U);
    EXPECT_TRUE(KeepsObstacleClearance(result.waypoints));
    EXPECT_GT(result.expandedNodes, 0);
}

TEST(FastPlannerGridPathPlannerTest, FailsWhenGoalOccupied)
{
    const LocalOccupancyMap map =
        LocalOccupancyMap::FromPointCloud({2.0f, 0.0f, 0.0f}, 0.5f);
    const FastPlannerGridPathPlannerPlugin planner;

    const auto result = planner.PlanPath(MakeRequest(map));

    EXPECT_FALSE(result.success);
    EXPECT_NE(result.reason.find("goal occupied"), std::string::npos);
}

} // namespace
