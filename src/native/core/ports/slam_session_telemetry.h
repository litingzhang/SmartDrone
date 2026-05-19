#pragma once

#include <cstdint>
#include <limits>

namespace smartdrone::core::application {
class FrameTimingTracker;
}

namespace smartdrone::core::ports {

struct SlamRangeSensor {
    float currentDistance{std::numeric_limits<float>::quiet_NaN()};
    uint8_t signalQuality{0};
};

class ISlamSessionTelemetryPort {
  public:
    virtual ~ISlamSessionTelemetryPort() = default;

    virtual void SetFrameTimingTracker(smartdrone::core::application::FrameTimingTracker *tracker) = 0;
    virtual bool GetDownwardRange(SlamRangeSensor &out, uint64_t maxAgeUs) const = 0;
    virtual void StopSetpointStream() = 0;
};

} // namespace smartdrone::core::ports
