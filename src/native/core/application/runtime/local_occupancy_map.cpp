#include "core/application/runtime/local_occupancy_map.h"

#include <algorithm>
#include <cmath>

namespace SmartDrone::Core::Application {
namespace {

struct VoxelSample {
    int ix{0};
    int iy{0};
    int iz{0};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

struct VoxelAccumulator {
    int ix{0};
    int iy{0};
    int iz{0};
    float sumX{0.0f};
    float sumY{0.0f};
    float sumZ{0.0f};
    int pointCount{0};
};

float ClampVoxelSize(float value)
{
    if (!std::isfinite(value)) {
        return 0.2f;
    }
    return std::max(0.05f, std::min(1.0f, value));
}

int VoxelCoord(float value, float voxelSize)
{
    return static_cast<int>(std::floor(value / voxelSize));
}

bool ReadVoxelSample(const std::vector<float> &cloud, size_t index,
                     float voxelSize, VoxelSample &sample)
{
    const size_t base = index * 3;
    sample = {VoxelCoord(cloud[base], voxelSize),
              VoxelCoord(cloud[base + 1], voxelSize),
              VoxelCoord(cloud[base + 2], voxelSize),
              cloud[base],
              cloud[base + 1],
              cloud[base + 2]};
    return std::isfinite(sample.x) && std::isfinite(sample.y) &&
           std::isfinite(sample.z);
}

bool VoxelSampleLess(const VoxelSample &a, const VoxelSample &b)
{
    if (a.ix != b.ix) {
        return a.ix < b.ix;
    }
    if (a.iy != b.iy) {
        return a.iy < b.iy;
    }
    return a.iz < b.iz;
}

bool SameVoxel(const VoxelAccumulator &accumulator,
               const VoxelSample &sample)
{
    return accumulator.ix == sample.ix && accumulator.iy == sample.iy &&
           accumulator.iz == sample.iz;
}

VoxelAccumulator StartAccumulator(const VoxelSample &sample)
{
    return {sample.ix, sample.iy, sample.iz, sample.x, sample.y, sample.z, 1};
}

void AddSample(VoxelAccumulator &accumulator, const VoxelSample &sample)
{
    accumulator.sumX += sample.x;
    accumulator.sumY += sample.y;
    accumulator.sumZ += sample.z;
    ++accumulator.pointCount;
}

void FlushAccumulator(const VoxelAccumulator &accumulator,
                      std::vector<OccupiedVoxel> &voxels)
{
    const float scale = 1.0f / static_cast<float>(accumulator.pointCount);
    voxels.push_back({accumulator.sumX * scale, accumulator.sumY * scale,
                      accumulator.sumZ * scale, accumulator.pointCount});
}

std::vector<VoxelSample> BuildVoxelSamples(const std::vector<float> &cloud,
                                           float voxelSize)
{
    std::vector<VoxelSample> samples;
    samples.reserve(cloud.size() / 3);
    for (size_t index = 0; index < cloud.size() / 3; ++index) {
        VoxelSample sample{};
        if (ReadVoxelSample(cloud, index, voxelSize, sample)) {
            samples.push_back(sample);
        }
    }
    return samples;
}

} // namespace

LocalOccupancyMap LocalOccupancyMap::FromPointCloud(
    const std::vector<float> &cloud, float voxelSizeM)
{
    LocalOccupancyMap map{};
    std::vector<VoxelSample> samples =
        BuildVoxelSamples(cloud, ClampVoxelSize(voxelSizeM));
    if (samples.empty()) {
        return map;
    }
    std::sort(samples.begin(), samples.end(), VoxelSampleLess);
    VoxelAccumulator accumulator = StartAccumulator(samples.front());
    for (size_t index = 1; index < samples.size(); ++index) {
        if (SameVoxel(accumulator, samples[index])) {
            AddSample(accumulator, samples[index]);
        } else {
            FlushAccumulator(accumulator, map.m_voxels);
            accumulator = StartAccumulator(samples[index]);
        }
    }
    FlushAccumulator(accumulator, map.m_voxels);
    return map;
}

const std::vector<OccupiedVoxel> &LocalOccupancyMap::OccupiedVoxels() const
{
    return m_voxels;
}

size_t LocalOccupancyMap::OccupiedVoxelCount() const
{
    return m_voxels.size();
}

} // namespace SmartDrone::Core::Application
