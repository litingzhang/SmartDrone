#include "adapters/slam/external_descriptor_geometry.h"

namespace smartdrone::adapters::slam {

cv::KeyPoint MakeExternalKeyPoint(const cv::Point2f &pt)
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
    return pt.x >= static_cast<float>(kDescriptorPatchBorder) &&
           pt.x < static_cast<float>(gray.cols - kDescriptorPatchBorder) &&
           pt.y >= static_cast<float>(kDescriptorPatchBorder) &&
           pt.y < static_cast<float>(gray.rows - kDescriptorPatchBorder);
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

} // namespace smartdrone::adapters::slam
