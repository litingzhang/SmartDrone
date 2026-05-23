#include "adapters/slam/klt/klt_continuous_frontend.h"

#include "adapters/slam/stereo/feature_point_tracking.h"
#include "adapters/slam/klt/klt_mode_utils.h"
#include "adapters/slam/klt/klt_track_manager.h"

namespace SmartDrone::Adapters::Slam {
namespace {

KltTrackedStereoPnpObservationBuilderOptions BuildContinuousObservationOptions(
    SlamModeSharedState &state, const KltContinuousFrontendResult &result,
    const std::vector<cv::Point2f> &currentLeftPoints,
    const std::vector<cv::Point2f> &currentRightPoints,
    const std::vector<uchar> &leftStatus,
    const std::vector<uchar> &rightStatus)
{
    KltTrackedStereoPnpObservationBuilderOptions options;
    options.previousTracks = &state.m_lkTracks;
    options.currentLeftPoints = &currentLeftPoints;
    options.currentRightPoints = &currentRightPoints;
    options.leftStatus = &leftStatus;
    options.rightStatus = &rightStatus;
    options.previousImageSize = state.m_lkPrevLeft.size();
    options.currentLeftImage = &result.leftRect;
    options.currentRightImage = &result.rightRect;
    options.fx = state.m_lkFx;
    options.fy = state.m_lkFy;
    options.cx = state.m_lkCx;
    options.cy = state.m_lkCy;
    options.baseline = state.m_lkBaseline;
    return options;
}

void TrackContinuousStereoPoints(SlamModeSharedState &state,
                                 const KltContinuousFrontendResult &result,
                                 std::vector<cv::Point2f> &currentLeftPoints,
                                 std::vector<cv::Point2f> &currentRightPoints,
                                 std::vector<uchar> &leftStatus,
                                 std::vector<uchar> &rightStatus)
{
    const std::vector<cv::Point2f> previousLeftPoints =
        ExtractLkTrackLeftPoints(state.m_lkTracks);
    if (previousLeftPoints.empty()) {
        return;
    }
    const std::vector<cv::Point2f> previousRightPoints =
        ExtractLkTrackRightPoints(state.m_lkTracks);
    Core::Ports::IPointTracker2d &pointTracker = state.PointTracker2d();
    (void)pointTracker.TrackForwardBackward(state.m_lkPrevLeft, result.leftRect,
                                            previousLeftPoints, currentLeftPoints,
                                            leftStatus);
    (void)pointTracker.TrackForwardBackward(state.m_lkPrevRight, result.rightRect,
                                            previousRightPoints,
                                            currentRightPoints, rightStatus);
}

} // namespace

KltContinuousFrontendResult RunKltContinuousFrontend(SlamModeSharedState &state,
                                                     const cv::Mat &leftRaw,
                                                     const cv::Mat &rightRaw)
{
    KltContinuousFrontendResult result;
    if (!state.m_lkCalibrationLoaded ||
        !state.PrepareRectifiedStereoCpu(leftRaw, rightRaw, result.leftRect,
                                         result.rightRect)) {
        return result;
    }
    result.valid = true;
    result.havePreviousFrame = state.m_lkHavePrev;
    if (!state.m_lkHavePrev) {
        return result;
    }

    state.m_lkTracks =
        SelectLkTracksGridBalanced(state.m_lkTracks, state.m_lkPrevLeft.size());
    std::vector<cv::Point2f> previousLeftPoints =
        ExtractLkTrackLeftPoints(state.m_lkTracks);
    std::vector<cv::Point2f> currentLeftPoints;
    std::vector<cv::Point2f> currentRightPoints;
    std::vector<uchar> leftStatus;
    std::vector<uchar> rightStatus;
    TrackContinuousStereoPoints(state, result, currentLeftPoints,
                                currentRightPoints, leftStatus, rightStatus);
    result.horizontalLateralFlow =
        IsHorizontalLateralFlow(previousLeftPoints, currentLeftPoints, leftStatus,
                                result.leftRect.size());

    const KltTrackedStereoPnpObservationBuilderOptions observationOptions =
        BuildContinuousObservationOptions(state, result, currentLeftPoints,
                                          currentRightPoints, leftStatus,
                                          rightStatus);
    result.observations =
        state.VisualPnpObservationBuilder().BuildTrackedStereoObservations(
            observationOptions);
    return result;
}

} // namespace SmartDrone::Adapters::Slam
