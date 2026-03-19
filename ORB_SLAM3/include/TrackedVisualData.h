#ifndef TRACKED_VISUAL_DATA_H
#define TRACKED_VISUAL_DATA_H

#include <opencv2/core/types.hpp>

#include <vector>

namespace ORB_SLAM3
{

struct TrackedVisualData
{
    std::vector<float> pointCloudXyz;
    std::vector<cv::Point2f> leftFeatures;
    std::vector<cv::Point2f> rightFeatures;
};

} // namespace ORB_SLAM3

#endif // TRACKED_VISUAL_DATA_H
