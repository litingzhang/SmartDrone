#pragma once

#include <cstdint>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "adapters/slam/slam_mode_state.h"

namespace smartdrone::adapters::slam {

void ResetKltLoopClosureState(SlamModeSharedState &state);

Sophus::SE3f ApplyKltLoopClosure(SlamModeSharedState &state, const cv::Mat &leftRect,
                                 uint64_t frameId, const Sophus::SE3f &rawTwc);

} // namespace smartdrone::adapters::slam
