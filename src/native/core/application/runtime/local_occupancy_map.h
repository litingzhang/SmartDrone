#pragma once

#include <cstddef>
#include <vector>

namespace SmartDrone::Core::Application {

struct OccupiedVoxel {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    int pointCount{0};
};

class LocalOccupancyMap final {
  public:
    static LocalOccupancyMap FromPointCloud(const std::vector<float> &cloud,
                                            float voxelSizeM);

    const std::vector<OccupiedVoxel> &OccupiedVoxels() const;
    size_t OccupiedVoxelCount() const;

  private:
    std::vector<OccupiedVoxel> m_voxels;
};

} // namespace SmartDrone::Core::Application
