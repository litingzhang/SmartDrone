#pragma once

#include <opencv2/core.hpp>

namespace SmartDrone::adapters::slam {

cv::Mat MakeCameraMatrix(float fx, float fy, float cx, float cy);
cv::Mat MakeDistCoeffs(float k1, float k2, float p1, float p2);
cv::Mat EnsureGray8(const cv::Mat &image);

} // namespace SmartDrone::adapters::slam
