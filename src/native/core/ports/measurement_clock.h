#pragma once

#include <cstdint>

namespace SmartDrone::Core::Ports {

class IMeasurementClock {
  public:
    virtual ~IMeasurementClock() = default;

    virtual std::uint64_t NowNs() const = 0;
    virtual std::uint32_t ResetCounter() const = 0;
    virtual bool Valid() const = 0;
};

} // namespace SmartDrone::Core::Ports
