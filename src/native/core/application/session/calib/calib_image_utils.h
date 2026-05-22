#pragma once

#include <opencv2/core.hpp>

namespace SmartDrone::core::application {

cv::Mat EnsureCalibGray8(const cv::Mat &src, bool &convertedOut);

} // namespace SmartDrone::core::application
