#pragma once

#include <opencv2/core.hpp>

namespace smartdrone::core::application {

cv::Mat EnsureCalibGray8(const cv::Mat &src, bool &convertedOut);

} // namespace smartdrone::core::application
