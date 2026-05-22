#pragma once

#include <opencv2/core.hpp>

namespace SmartDrone::Core::Application {

cv::Mat EnsureCalibGray8(const cv::Mat &src, bool &convertedOut);

} // namespace SmartDrone::Core::Application
