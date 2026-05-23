#pragma once

#include "core/ports/visual_frame_observation.h"

namespace SmartDrone::Adapters::Slam {

class DefaultVisualFrameObservationLoader final
    : public Core::Ports::IVisualFrameObservationLoader {
  public:
    bool LoadMonoObservation(
        const Core::Ports::MonoFrameObservationLoadRequest &request,
        Core::Ports::VisualFrameObservationData &outData) const override;
    bool LoadStereoObservation(
        const Core::Ports::StereoFrameObservationLoadRequest &request,
        Core::Ports::VisualFrameObservationData &outData) const override;
};

bool LoadMonoFrameObservation(
    const Core::Ports::MonoFrameObservationLoadRequest &request,
    Core::Ports::VisualFrameObservationData &outData);
bool LoadStereoFrameObservation(
    const Core::Ports::StereoFrameObservationLoadRequest &request,
    Core::Ports::VisualFrameObservationData &outData);

} // namespace SmartDrone::Adapters::Slam
