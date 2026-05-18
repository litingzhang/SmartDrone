#pragma once

#include <cstdint>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "adapters/slam/slam_mode_state.h"
#include "core/ports/visual_tracking.h"

namespace smartdrone::adapters::slam {

using LkLoopClosureState = core::ports::LoopClosureState;

class DefaultVisualLoopClosureBackend final : public core::ports::IVisualLoopClosureBackend {
  public:
    void Reset(LkLoopClosureState &state) const override;
    Sophus::SE3f Apply(LkLoopClosureState &state, const cv::Mat &leftRect,
                       uint64_t frameId, const Sophus::SE3f &rawTwc) const override;
};

void ResetKltLoopClosureState(SlamModeSharedState &state);

Sophus::SE3f ApplyKltLoopClosure(SlamModeSharedState &state, const cv::Mat &leftRect,
                                 uint64_t frameId, const Sophus::SE3f &rawTwc);

} // namespace smartdrone::adapters::slam
