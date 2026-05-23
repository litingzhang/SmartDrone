#pragma once

#include <cstdint>

namespace SmartDrone::Core::Domain {

struct ImuSample {
    std::int64_t tNs{};
    float ax{};
    float ay{};
    float az{};
    float gx{};
    float gy{};
    float gz{};
};

struct ImuScale {
    float accelLsbPerG{2048.0F};
    float gyroLsbPerDps{16.4F};
};

struct ImuTimeRange {
    std::int64_t startNs{};
    std::int64_t endNs{};
    std::int64_t slackBeforeNs{};
    std::int64_t slackAfterNs{};
};

} // namespace SmartDrone::Core::Domain

using ImuSample = SmartDrone::Core::Domain::ImuSample;
using ImuScale = SmartDrone::Core::Domain::ImuScale;
using ImuTimeRange = SmartDrone::Core::Domain::ImuTimeRange;
