#pragma once

#include <cstdint>

#include "core/ports/stereo_processing.h"

namespace SmartDrone::Adapters::Slam {

class DefaultStereoFeaturePacketBuilder final
    : public Core::Ports::IStereoFeaturePacketBuilder {
  public:
    bool BuildPacket(const Core::Ports::StereoFeaturePacketBuildInput &input,
                     Core::Ports::StereoFeaturePacket &packet) const override;
    uint64_t HashStereoData(
        const Core::Ports::StereoFeatureObservationPacket &data) const override;
};

bool BuildStereoFeaturePacket(
    const Core::Ports::StereoFeaturePacketBuildInput &input,
    Core::Ports::StereoFeaturePacket &packet);
uint64_t HashStereoFeatureObservations(
    const Core::Ports::StereoFeatureObservationPacket &data);

} // namespace SmartDrone::Adapters::Slam
