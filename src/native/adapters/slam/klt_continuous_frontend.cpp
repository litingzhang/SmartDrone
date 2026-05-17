#include "adapters/slam/klt_continuous_frontend.h"

#include "adapters/slam/feature_point_tracking.h"
#include "adapters/slam/klt_mode_utils.h"
#include "adapters/slam/klt_track_manager.h"

namespace smartdrone::adapters::slam {

KltContinuousFrontendResult RunKltContinuousFrontend(
    SlamModeSharedState &state, const cv::Mat &leftRaw, const cv::Mat &rightRaw)
{
    KltContinuousFrontendResult result;
    if (!state.m_lkCalibrationLoaded ||
        !state.PrepareRectifiedStereoCpu(leftRaw, rightRaw, result.leftRect, result.rightRect)) {
        return result;
    }
    result.valid = true;
    result.havePreviousFrame = state.m_lkHavePrev;
    if (!state.m_lkHavePrev) {
        return result;
    }

    state.m_lkTracks = SelectLkTracksGridBalanced(state.m_lkTracks, state.m_lkPrevLeft.size());
    std::vector<cv::Point2f> previousLeftPoints = ExtractLkTrackLeftPoints(state.m_lkTracks);
    std::vector<cv::Point2f> currentLeftPoints;
    std::vector<cv::Point2f> currentRightPoints;
    std::vector<uchar> leftStatus;
    std::vector<uchar> rightStatus;
    if (!previousLeftPoints.empty()) {
        std::vector<cv::Point2f> previousRightPoints = ExtractLkTrackRightPoints(state.m_lkTracks);
        (void)TrackPointsForwardBackward(state.m_lkPrevLeft, result.leftRect, previousLeftPoints,
                                         currentLeftPoints, leftStatus);
        (void)TrackPointsForwardBackward(state.m_lkPrevRight, result.rightRect, previousRightPoints,
                                         currentRightPoints, rightStatus);
    }
    result.horizontalLateralFlow =
        IsHorizontalLateralFlow(previousLeftPoints, currentLeftPoints, leftStatus, result.leftRect.size());

    KltTrackedStereoPnpObservationBuilderOptions observationOptions;
    observationOptions.previousTracks = &state.m_lkTracks;
    observationOptions.currentLeftPoints = &currentLeftPoints;
    observationOptions.currentRightPoints = &currentRightPoints;
    observationOptions.leftStatus = &leftStatus;
    observationOptions.rightStatus = &rightStatus;
    observationOptions.previousImageSize = state.m_lkPrevLeft.size();
    observationOptions.currentLeftImage = &result.leftRect;
    observationOptions.currentRightImage = &result.rightRect;
    observationOptions.fx = state.m_lkFx;
    observationOptions.fy = state.m_lkFy;
    observationOptions.cx = state.m_lkCx;
    observationOptions.cy = state.m_lkCy;
    observationOptions.baseline = state.m_lkBaseline;
    result.observations = BuildKltTrackedStereoPnpObservations(observationOptions);
    return result;
}

} // namespace smartdrone::adapters::slam
