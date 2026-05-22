#pragma once

#include "core/ports/visual_frame_observation.h"

namespace SmartDrone::adapters::slam {

class DefaultVisualFrameObservationLoader final
    : public core::ports::IVisualFrameObservationLoader {
  public:
    bool LoadMonoObservation(
        const core::ports::MonoFrameObservationLoadRequest &request,
        core::ports::VisualFrameObservationData &outData) const override;
    bool LoadStereoObservation(
        const core::ports::StereoFrameObservationLoadRequest &request,
        core::ports::VisualFrameObservationData &outData) const override;
};

bool LoadMonoFrameObservation(
    const core::ports::MonoFrameObservationLoadRequest &request,
    core::ports::VisualFrameObservationData &outData);
bool LoadStereoFrameObservation(
    const core::ports::StereoFrameObservationLoadRequest &request,
    core::ports::VisualFrameObservationData &outData);

} // namespace SmartDrone::adapters::slam
