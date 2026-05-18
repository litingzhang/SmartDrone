#pragma once

#include <cstdint>

#include "core/ports/stereo_processing.h"

namespace smartdrone::adapters::slam {

class DefaultStereoFeaturePacketBuilder final
    : public core::ports::IStereoFeaturePacketBuilder {
public:
  bool BuildPacket(const core::ports::StereoFeaturePacketBuildInput &input,
                   core::ports::StereoFeaturePacket &packet) const override;
  uint64_t HashStereoData(
      const core::ports::StereoFeatureObservationPacket &data) const override;
};

bool BuildStereoFeaturePacket(
    const core::ports::StereoFeaturePacketBuildInput &input,
    core::ports::StereoFeaturePacket &packet);
uint64_t HashStereoFeatureObservations(
    const core::ports::StereoFeatureObservationPacket &data);

} // namespace smartdrone::adapters::slam
