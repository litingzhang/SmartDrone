#pragma once

#include <opencv2/core.hpp>

#include "core/ports/visual_tracking.h"

namespace SmartDrone::Adapters::Slam {

using ForwardBackwardTrackingRequest = Core::Ports::ForwardBackwardTrackingRequest;
using ForwardBackwardTrackingOptions = Core::Ports::ForwardBackwardTrackingOptions;

class DefaultPointTracker2d final : public Core::Ports::IPointTracker2d {
  public:
    bool TrackForwardBackward(
        const ForwardBackwardTrackingRequest &request) const override;
};

bool TrackPointsForwardBackward(const ForwardBackwardTrackingRequest &request);

} // namespace SmartDrone::Adapters::Slam
