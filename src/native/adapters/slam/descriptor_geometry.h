#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace SmartDrone::adapters::slam {

constexpr int kDescriptorPatchBorder = 19;

cv::KeyPoint MakeDescriptorKeyPoint(const cv::Point2f &pt);
bool IsPointSafeForDescriptor(const cv::Point2f &pt, const cv::Mat &gray);
bool IsPointNearExistingKeypoint(const cv::Point2f &pt,
                                 const std::vector<cv::KeyPoint> &existing,
                                 float minDistancePx = 4.0f);

} // namespace SmartDrone::adapters::slam
