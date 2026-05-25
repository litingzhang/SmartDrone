#include "adapters/slam/stereo/feature_point_tracking.h"

#include <algorithm>

#include <opencv2/video/tracking.hpp>

namespace SmartDrone::Adapters::Slam {

bool TrackPointsForwardBackward(const ForwardBackwardTrackingRequest &request)
{
    request.currPoints.clear();
    request.status.clear();
    if (request.prevGray.empty() || request.currGray.empty() ||
        request.prevPoints.empty()) {
        return false;
    }

    const int windowSizePx = std::max(3, request.options.windowSizePx);
    const int maxLevel = std::max(0, request.options.maxLevel);
    const float maxErrorPx =
        std::max(0.0f, request.options.maxForwardBackwardErrorPx);

    std::vector<float> errors;
    cv::calcOpticalFlowPyrLK(request.prevGray, request.currGray,
                             request.prevPoints, request.currPoints,
                             request.status, errors,
                             cv::Size(windowSizePx, windowSizePx), maxLevel);
    if (request.currPoints.empty() || request.status.empty()) {
        return false;
    }

    std::vector<cv::Point2f> backwardPoints;
    std::vector<uchar> backwardStatus;
    std::vector<float> backwardErrors;
    cv::calcOpticalFlowPyrLK(request.currGray, request.prevGray,
                             request.currPoints, backwardPoints,
                             backwardStatus, backwardErrors,
                             cv::Size(windowSizePx, windowSizePx), maxLevel);

    const float maxErrorSq = maxErrorPx * maxErrorPx;
    for (size_t i = 0; i < request.status.size(); ++i) {
        if (!request.status[i] || i >= backwardStatus.size() ||
            !backwardStatus[i] || i >= backwardPoints.size()) {
            request.status[i] = 0;
            continue;
        }

        const cv::Point2f delta = backwardPoints[i] - request.prevPoints[i];
        if ((delta.x * delta.x + delta.y * delta.y) > maxErrorSq) {
            request.status[i] = 0;
        }
    }

    return true;
}

bool DefaultPointTracker2d::TrackForwardBackward(
    const ForwardBackwardTrackingRequest &request) const
{
    return TrackPointsForwardBackward(request);
}

} // namespace SmartDrone::Adapters::Slam
