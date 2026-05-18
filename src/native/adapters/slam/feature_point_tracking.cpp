#include "adapters/slam/feature_point_tracking.h"

#include <algorithm>

#include <opencv2/video/tracking.hpp>

namespace smartdrone::adapters::slam {

bool TrackPointsForwardBackward(const cv::Mat &prevGray, const cv::Mat &currGray,
                                const std::vector<cv::Point2f> &prevPoints,
                                std::vector<cv::Point2f> &currPoints,
                                std::vector<uchar> &status,
                                const ForwardBackwardTrackingOptions &options)
{
    currPoints.clear();
    status.clear();
    if (prevGray.empty() || currGray.empty() || prevPoints.empty()) {
        return false;
    }

    const int windowSizePx = std::max(3, options.windowSizePx);
    const int maxLevel = std::max(0, options.maxLevel);
    const float maxErrorPx = std::max(0.0f, options.maxForwardBackwardErrorPx);

    std::vector<float> errors;
    cv::calcOpticalFlowPyrLK(prevGray, currGray, prevPoints, currPoints, status, errors,
                             cv::Size(windowSizePx, windowSizePx), maxLevel);
    if (currPoints.empty() || status.empty()) {
        return false;
    }

    std::vector<cv::Point2f> backwardPoints;
    std::vector<uchar> backwardStatus;
    std::vector<float> backwardErrors;
    cv::calcOpticalFlowPyrLK(currGray, prevGray, currPoints, backwardPoints, backwardStatus,
                             backwardErrors, cv::Size(windowSizePx, windowSizePx), maxLevel);

    const float maxErrorSq = maxErrorPx * maxErrorPx;
    for (size_t i = 0; i < status.size(); ++i) {
        if (!status[i] || i >= backwardStatus.size() || !backwardStatus[i] || i >= backwardPoints.size()) {
            status[i] = 0;
            continue;
        }

        const cv::Point2f delta = backwardPoints[i] - prevPoints[i];
        if ((delta.x * delta.x + delta.y * delta.y) > maxErrorSq) {
            status[i] = 0;
        }
    }

    return true;
}

bool DefaultPointTracker2d::TrackForwardBackward(const cv::Mat &prevGray, const cv::Mat &currGray,
                                                 const std::vector<cv::Point2f> &prevPoints,
                                                 std::vector<cv::Point2f> &currPoints,
                                                 std::vector<uchar> &status,
                                                 const ForwardBackwardTrackingOptions &options) const
{
    return TrackPointsForwardBackward(prevGray, currGray, prevPoints, currPoints, status, options);
}

} // namespace smartdrone::adapters::slam
