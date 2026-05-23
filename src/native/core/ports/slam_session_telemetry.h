#pragma once

#include <cstdint>
#include <limits>

#include "core/ports/frame_timing.h"

namespace SmartDrone::Core::Ports {

struct SlamRangeSensor {
    float currentDistance{std::numeric_limits<float>::quiet_NaN()};
    uint8_t signalQuality{0};
};

class ISlamSessionTelemetryPort {
  public:
    virtual ~ISlamSessionTelemetryPort() = default;

    virtual void SetFrameTimingTracker(IFrameTimingTracker *tracker) = 0;
    virtual bool GetDownwardRange(SlamRangeSensor &out, uint64_t maxAgeUs) const = 0;
    virtual void StopSetpointStream() = 0;
};

} // namespace SmartDrone::Core::Ports
