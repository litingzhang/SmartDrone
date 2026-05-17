#include "adapters/slam/orb_feature_utils.h"

#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "adapters/slam/external_descriptor_geometry.h"

namespace smartdrone::adapters::slam {

bool ComputeOrbDescriptorsAtPoints(ORB_SLAM3::ORBextractor *extractor,
                                   const cv::Mat &gray,
                                   const std::vector<cv::Point2f> &points,
                                   std::vector<cv::KeyPoint> &keypoints,
                                   cv::Mat &descriptors)
{
    keypoints.clear();
    descriptors.release();
    if (extractor == nullptr || gray.empty() || points.empty()) {
        return false;
    }

    keypoints.reserve(points.size());
    for (const cv::Point2f &pt : points) {
        keypoints.push_back(MakeExternalKeyPoint(pt));
    }

    if (!extractor->ComputeDescriptorsAtKeypoints(gray, keypoints, descriptors)) {
        return false;
    }
    return !keypoints.empty() && !descriptors.empty() &&
           descriptors.rows == static_cast<int>(keypoints.size()) && descriptors.type() == CV_8U;
}

OrbExternalDescriptorProvider::OrbExternalDescriptorProvider(ORB_SLAM3::ORBextractor *extractor)
    : m_extractor(extractor)
{
}

bool OrbExternalDescriptorProvider::ComputeDescriptorsAtPoints(const cv::Mat &gray,
                                                              const std::vector<cv::Point2f> &points,
                                                              std::vector<cv::KeyPoint> &keypoints,
                                                              cv::Mat &descriptors) const
{
    return ComputeOrbDescriptorsAtPoints(m_extractor, gray, points, keypoints, descriptors);
}

bool OrbExternalDescriptorProvider::DetectAndCompute(const cv::Mat &gray,
                                                    std::vector<cv::KeyPoint> &keypoints,
                                                    cv::Mat &descriptors) const
{
    keypoints.clear();
    descriptors.release();
    if (m_extractor == nullptr || gray.empty()) {
        return false;
    }
    std::vector<int> lapping = {0, 0};
    (*m_extractor)(gray, cv::Mat(), keypoints, descriptors, lapping);
    return !keypoints.empty() && !descriptors.empty() &&
           descriptors.rows == static_cast<int>(keypoints.size()) && descriptors.type() == CV_8U;
}

int OrbExternalDescriptorProvider::DescriptorDistance(const cv::Mat &leftDescriptor,
                                                     const cv::Mat &rightDescriptor) const
{
    return ORB_SLAM3::ORBmatcher::DescriptorDistance(leftDescriptor, rightDescriptor);
}

} // namespace smartdrone::adapters::slam
