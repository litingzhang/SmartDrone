#pragma once

#include <opencv2/core.hpp>

#include "core/ports/visual_tracking.h"

namespace SmartDrone::Adapters::Slam {

using ForwardBackwardTrackingOptions = Core::Ports::ForwardBackwardTrackingOptions;

class DefaultPointTracker2d final : public Core::Ports::IPointTracker2d {
  public:
    bool TrackForwardBackward(const cv::Mat &prevGray, const cv::Mat &currGray,
                              const std::vector<cv::Point2f> &prevPoints,
                              std::vector<cv::Point2f> &currPoints,
                              std::vector<uchar> &status,
                              const ForwardBackwardTrackingOptions &options = {}) const override;
};

bool TrackPointsForwardBackward(const cv::Mat &prevGray, const cv::Mat &currGray,
                                const std::vector<cv::Point2f> &prevPoints,
                                std::vector<cv::Point2f> &currPoints,
                                std::vector<uchar> &status,
                                const ForwardBackwardTrackingOptions &options = {});

} // namespace SmartDrone::Adapters::Slam
