#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace smartdrone::adapters::slam {

struct ForwardBackwardTrackingOptions {
    int windowSizePx{21};
    int maxLevel{3};
    float maxForwardBackwardErrorPx{1.5f};
};

bool TrackPointsForwardBackward(const cv::Mat &prevGray, const cv::Mat &currGray,
                                const std::vector<cv::Point2f> &prevPoints,
                                std::vector<cv::Point2f> &currPoints,
                                std::vector<uchar> &status,
                                const ForwardBackwardTrackingOptions &options = {});

} // namespace smartdrone::adapters::slam
