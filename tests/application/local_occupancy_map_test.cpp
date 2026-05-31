#include "core/application/runtime/local_occupancy_map.h"

#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace {

using SmartDrone::Core::Application::LocalOccupancyMap;

TEST(LocalOccupancyMapTest, MergesPointsInsideSameVoxel)
{
    const std::vector<float> cloud{
        1.0f, 0.1f, 0.0f,
        1.02f, 0.12f, 0.0f,
        1.4f, 0.1f, 0.0f};

    const LocalOccupancyMap map =
        LocalOccupancyMap::FromPointCloud(cloud, 0.3f);

    ASSERT_EQ(map.OccupiedVoxelCount(), 2U);
    EXPECT_EQ(map.OccupiedVoxels()[0].pointCount, 2);
    EXPECT_NEAR(map.OccupiedVoxels()[0].x, 1.01f, 0.001f);
    EXPECT_EQ(map.OccupiedVoxels()[1].pointCount, 1);
}

TEST(LocalOccupancyMapTest, SkipsNonFinitePoints)
{
    const std::vector<float> cloud{
        1.0f, 0.0f, 0.0f,
        std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f};

    const LocalOccupancyMap map =
        LocalOccupancyMap::FromPointCloud(cloud, 0.3f);

    ASSERT_EQ(map.OccupiedVoxelCount(), 1U);
    EXPECT_EQ(map.OccupiedVoxels()[0].pointCount, 1);
}

} // namespace
