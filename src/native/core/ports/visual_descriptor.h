#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "core/ports/visual_feature_data.h"

namespace SmartDrone::Core::Ports {

class IVisualDescriptorProvider {
  public:
    virtual ~IVisualDescriptorProvider() = default;

    virtual bool ComputeDescriptorsAtPoints(
        const cv::Mat &gray, const std::vector<cv::Point2f> &points,
        std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) const = 0;
    virtual bool DetectAndCompute(const cv::Mat &gray,
                                  std::vector<cv::KeyPoint> &keypoints,
                                  cv::Mat &descriptors) const = 0;
    virtual int DescriptorDistance(const cv::Mat &leftDescriptor,
                                   const cv::Mat &rightDescriptor) const = 0;
};

} // namespace SmartDrone::Core::Ports
