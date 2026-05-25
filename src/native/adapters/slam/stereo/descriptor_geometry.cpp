#include "adapters/slam/stereo/descriptor_geometry.h"

namespace SmartDrone::Adapters::Slam {

cv::KeyPoint MakeDescriptorKeyPoint(const cv::Point2f &pt)
{
    cv::KeyPoint kp;
    kp.pt = pt;
    kp.size = 31.0f;
    kp.angle = -1.0f;
    kp.octave = 0;
    kp.response = 1.0f;
    return kp;
}

bool IsPointSafeForDescriptor(const cv::Point2f &pt, const cv::Mat &gray)
{
    return pt.x >= static_cast<float>(DESCRIPTOR_PATCH_BORDER) &&
           pt.x < static_cast<float>(gray.cols - DESCRIPTOR_PATCH_BORDER) &&
           pt.y >= static_cast<float>(DESCRIPTOR_PATCH_BORDER) &&
           pt.y < static_cast<float>(gray.rows - DESCRIPTOR_PATCH_BORDER);
}

bool IsPointNearExistingKeypoint(const cv::Point2f &pt,
                                 const std::vector<cv::KeyPoint> &existing,
                                 float minDistancePx)
{
    const float minDistSq = minDistancePx * minDistancePx;
    for (const cv::KeyPoint &keypoint : existing) {
        const cv::Point2f delta = keypoint.pt - pt;
        if ((delta.x * delta.x + delta.y * delta.y) <= minDistSq) {
            return true;
        }
    }
    return false;
}

} // namespace SmartDrone::Adapters::Slam
