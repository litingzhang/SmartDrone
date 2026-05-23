#pragma once

#include <cstdint>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "adapters/slam/engine/slam_mode_state.h"
#include "core/ports/visual_tracking.h"

namespace SmartDrone::Adapters::Slam {

using LkLoopClosureState = Core::Ports::LoopClosureState;

class DefaultVisualLoopClosureBackend final : public Core::Ports::IVisualLoopClosureBackend {
  public:
    void Reset(LkLoopClosureState &state) const override;
    Sophus::SE3f Apply(LkLoopClosureState &state, const cv::Mat &leftRect,
                       uint64_t frameId, const Sophus::SE3f &rawTwc) const override;
};

void ResetKltLoopClosureState(SlamModeSharedState &state);

Sophus::SE3f ApplyKltLoopClosure(SlamModeSharedState &state, const cv::Mat &leftRect,
                                 uint64_t frameId, const Sophus::SE3f &rawTwc);

} // namespace SmartDrone::Adapters::Slam
