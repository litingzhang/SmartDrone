#pragma once

#include <cstdint>

namespace SmartDrone::core::application {

enum class CalibImuSampleStatus : std::uint8_t {
    Written,
    Pending,
    Failed,
};

struct CalibImuSampleResult {
    CalibImuSampleStatus status{CalibImuSampleStatus::Pending};
};

} // namespace SmartDrone::core::application
