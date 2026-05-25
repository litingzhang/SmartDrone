#include "adapters/slam/klt/klt_continuous_frontend.h"

#include "adapters/slam/stereo/feature_point_tracking.h"
#include "adapters/slam/klt/klt_mode_utils.h"
#include "adapters/slam/klt/klt_track_manager.h"

namespace SmartDrone::Adapters::Slam {
namespace {

struct ContinuousStereoTrackingResult {
    std::vector<cv::Point2f> currentLeftPoints;
    std::vector<cv::Point2f> currentRightPoints;
    std::vector<uchar> leftStatus;
    std::vector<uchar> rightStatus;
};

KltTrackedStereoPnpObservationBuilderOptions BuildContinuousObservationOptions(
    SlamModeSharedState &state, const KltContinuousFrontendResult &result,
    const ContinuousStereoTrackingResult &tracking)
{
    KltTrackedStereoPnpObservationBuilderOptions options;
    options.previousTracks = &state.m_lkTracks;
    options.currentLeftPoints = &tracking.currentLeftPoints;
    options.currentRightPoints = &tracking.currentRightPoints;
    options.leftStatus = &tracking.leftStatus;
    options.rightStatus = &tracking.rightStatus;
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
                                 ContinuousStereoTrackingResult &tracking)
{
    const std::vector<cv::Point2f> previousLeftPoints =
        ExtractLkTrackLeftPoints(state.m_lkTracks);
    if (previousLeftPoints.empty()) {
        return;
    }
    const std::vector<cv::Point2f> previousRightPoints =
        ExtractLkTrackRightPoints(state.m_lkTracks);
    Core::Ports::IPointTracker2d &pointTracker = state.PointTracker2d();
    (void)pointTracker.TrackForwardBackward(ForwardBackwardTrackingRequest{
        state.m_lkPrevLeft, result.leftRect, previousLeftPoints,
        tracking.currentLeftPoints, tracking.leftStatus});
    (void)pointTracker.TrackForwardBackward(ForwardBackwardTrackingRequest{
        state.m_lkPrevRight, result.rightRect, previousRightPoints,
        tracking.currentRightPoints, tracking.rightStatus});
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
    ContinuousStereoTrackingResult tracking;
    TrackContinuousStereoPoints(state, result, tracking);
    result.horizontalLateralFlow =
        IsHorizontalLateralFlow(previousLeftPoints, tracking.currentLeftPoints,
                                tracking.leftStatus, result.leftRect.size());

    const KltTrackedStereoPnpObservationBuilderOptions observationOptions =
        BuildContinuousObservationOptions(state, result, tracking);
    result.observations =
        state.VisualPnpObservationBuilder().BuildTrackedStereoObservations(
            observationOptions);
    return result;
}

} // namespace SmartDrone::Adapters::Slam
