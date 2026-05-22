#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/klt_pnp_observation_builder.h"
#include "adapters/slam/slam_mode_state.h"

namespace SmartDrone::adapters::slam {

struct KltPerFrameFrontendResult {
    bool valid{false};
    bool usedVpiRemap{false};
    bool preferAcceleratedPnpDefaults{false};
    bool useKeyframeReference{false};
    int keyframeInterval{1};
    cv::Mat leftRect;
    cv::Mat rightRect;
    std::vector<cv::Point2f> currentLeftPoints;
    KltPnpObservationSet observations;
    double inputPrepareMs{0.0};
    double rectifyMs{0.0};
    double disparityMs{0.0};
    double gfttMs{0.0};
    double flowMs{0.0};
    double candidateMs{0.0};
};

KltPerFrameFrontendResult RunKltPerFrameFrontend(
    SlamModeSharedState &state, const cv::Mat &leftRaw, const cv::Mat &rightRaw);

bool ShouldRefreshKltPerFrameReference(const SlamModeSharedState &state,
                                       const KltPerFrameFrontendResult &frontend,
                                       int inlierCount);

void UpdateKltPerFrameReferenceFrame(SlamModeSharedState &state,
                                     const KltPerFrameFrontendResult &frontend);

} // namespace SmartDrone::adapters::slam
