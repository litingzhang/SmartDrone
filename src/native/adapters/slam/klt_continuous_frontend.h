#pragma once

#include <opencv2/core.hpp>

#include "adapters/slam/klt_pnp_observation_builder.h"
#include "adapters/slam/slam_mode_state.h"

namespace SmartDrone::Adapters::Slam {

struct KltContinuousFrontendResult {
    bool valid{false};
    bool havePreviousFrame{false};
    bool horizontalLateralFlow{false};
    cv::Mat leftRect;
    cv::Mat rightRect;
    KltTrackedStereoPnpObservationSet observations;
};

KltContinuousFrontendResult RunKltContinuousFrontend(
    SlamModeSharedState &state, const cv::Mat &leftRaw, const cv::Mat &rightRaw);

} // namespace SmartDrone::Adapters::Slam
