#pragma once

#include <opencv2/core.hpp>

#include "adapters/slam/stereo_calibration.h"

namespace smartdrone::adapters::slam {

struct PreparedStereoFrame {
    cv::Mat leftGray;
    cv::Mat rightGray;
    cv::Mat leftRect;
    cv::Mat rightRect;
    bool rectified{false};
};

bool PrepareStereoFrameForFrontend(const cv::Mat &leftImage, const cv::Mat &rightImage,
                                   PreparedStereoFrame &frame, StereoCalibration *calibration,
                                   bool rectify);

} // namespace smartdrone::adapters::slam
