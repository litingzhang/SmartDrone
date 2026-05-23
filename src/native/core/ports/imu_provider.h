#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/domain/imu_sample.h"

namespace SmartDrone::Core::Ports {

struct ImuReading {
    int64_t timestampNs{0};
    float ax{0.0f};
    float ay{0.0f};
    float az{0.0f};
    float gx{0.0f};
    float gy{0.0f};
    float gz{0.0f};
};

class IImuProvider {
  public:
    virtual ~IImuProvider() = default;

    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual bool Ready() const = 0;
    virtual std::vector<ImuReading> PopWindow(int64_t fromNs, int64_t toNs) = 0;
};

class IImuWindowSource {
  public:
    virtual ~IImuWindowSource() = default;

    virtual std::size_t Size() const = 0;
    virtual std::vector<ImuReading> PopBetweenNs(
        const SmartDrone::Core::Domain::ImuTimeRange &range) = 0;
};

} // namespace SmartDrone::Core::Ports
