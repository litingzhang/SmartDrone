#pragma once

#include <cstdint>

#include "core/domain/imu_sample.h"

namespace SmartDrone::Core::Ports {

enum class ImuSampleReadStatus : std::uint8_t {
    Ready,
    Pending,
    Failed,
};

} // namespace SmartDrone::Core::Ports

namespace SmartDrone::Core::Ports {

class IImuSampleSource {
  public:
    virtual ~IImuSampleSource() = default;

    virtual bool Start() = 0;
    virtual bool EnsureOpen() = 0;
    virtual void Stop() = 0;
    virtual ImuSampleReadStatus ReadSample(ImuSample &sample) = 0;
    virtual ImuScale Scale() const = 0;
    virtual bool Failed() const = 0;
};

} // namespace SmartDrone::Core::Ports
