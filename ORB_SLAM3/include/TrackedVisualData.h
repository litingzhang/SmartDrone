#ifndef TRACKED_VISUAL_DATA_H
#define TRACKED_VISUAL_DATA_H

#include <opencv2/core/types.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ORB_SLAM3
{

struct TrackedVisualData
{
    int matchesInliers{0};
    size_t trackedMapPointCount{0};
    size_t localMapPointCount{0};
    uint64_t localMapPointHash{0};
    uint64_t matchedMapPointHashBeforePoseOptimization{0};
    uint64_t trackedMapPointHash{0};
    std::vector<float> pointCloudXyz;
    std::vector<cv::Point2f> leftFeatures;
    std::vector<cv::Point2f> rightFeatures;
};

} // namespace ORB_SLAM3

#endif // TRACKED_VISUAL_DATA_H
