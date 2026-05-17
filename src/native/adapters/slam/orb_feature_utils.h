#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/external_feature_types.h"

namespace ORB_SLAM3 {
class ORBextractor;
}

namespace smartdrone::adapters::slam {

bool ComputeOrbDescriptorsAtPoints(ORB_SLAM3::ORBextractor *extractor,
                                   const cv::Mat &gray,
                                   const std::vector<cv::Point2f> &points,
                                   std::vector<cv::KeyPoint> &keypoints,
                                   cv::Mat &descriptors);

class OrbExternalDescriptorProvider final : public ExternalDescriptorProvider {
  public:
    explicit OrbExternalDescriptorProvider(ORB_SLAM3::ORBextractor *extractor);

    bool ComputeDescriptorsAtPoints(const cv::Mat &gray,
                                    const std::vector<cv::Point2f> &points,
                                    std::vector<cv::KeyPoint> &keypoints,
                                    cv::Mat &descriptors) const override;
    bool DetectAndCompute(const cv::Mat &gray,
                          std::vector<cv::KeyPoint> &keypoints,
                          cv::Mat &descriptors) const override;
    int DescriptorDistance(const cv::Mat &leftDescriptor,
                           const cv::Mat &rightDescriptor) const override;

  private:
    ORB_SLAM3::ORBextractor *m_extractor{nullptr};
};

} // namespace smartdrone::adapters::slam
