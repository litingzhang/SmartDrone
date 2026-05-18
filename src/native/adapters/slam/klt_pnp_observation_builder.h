#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "adapters/slam/slam_mode_state.h"
#include "core/ports/visual_tracking.h"

namespace smartdrone::adapters::slam {

using KltPerFramePnpObservationBuilderOptions = core::ports::PerFramePnpObservationBuilderOptions;
using KltPnpObservationSet = core::ports::PnpObservationSet;
using KltTrackedStereoPnpObservationBuilderOptions = core::ports::TrackedStereoPnpObservationBuilderOptions;
using KltTrackedStereoPnpObservationSet = core::ports::TrackedStereoPnpObservationSet;

class DefaultVisualPnpObservationBuilder final : public core::ports::IVisualPnpObservationBuilder {
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

} // namespace smartdrone::adapters::slam
