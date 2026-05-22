#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/slam_mode_state.h"
#include "core/ports/visual_tracking.h"

namespace SmartDrone::Adapters::Slam {

using KltPerFramePnpObservationBuilderOptions = Core::Ports::PerFramePnpObservationBuilderOptions;
using KltPnpObservationSet = Core::Ports::PnpObservationSet;
using KltTrackedStereoPnpObservationBuilderOptions = Core::Ports::TrackedStereoPnpObservationBuilderOptions;
using KltTrackedStereoPnpObservationSet = Core::Ports::TrackedStereoPnpObservationSet;

class DefaultVisualPnpObservationBuilder final : public Core::Ports::IVisualPnpObservationBuilder {
  public:
    KltPnpObservationSet BuildPerFrameObservations(
        const KltPerFramePnpObservationBuilderOptions &options) const override;
    KltTrackedStereoPnpObservationSet BuildTrackedStereoObservations(
        const KltTrackedStereoPnpObservationBuilderOptions &options) const override;
};

KltPnpObservationSet BuildKltPerFramePnpObservations(
    const KltPerFramePnpObservationBuilderOptions &options);

KltTrackedStereoPnpObservationSet BuildKltTrackedStereoPnpObservations(
    const KltTrackedStereoPnpObservationBuilderOptions &options);

} // namespace SmartDrone::Adapters::Slam
