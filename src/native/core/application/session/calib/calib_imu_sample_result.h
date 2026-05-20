#pragma once

#include <cstdint>

namespace smartdrone::core::application {

enum class CalibImuSampleStatus : std::uint8_t {
    Written,
    Pending,
    Failed,
};

struct CalibImuSampleResult {
    CalibImuSampleStatus status{CalibImuSampleStatus::Pending};
};

} // namespace smartdrone::core::application
