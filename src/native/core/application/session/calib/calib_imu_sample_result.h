#pragma once

#include <cstdint>

namespace SmartDrone::Core::Application {

enum class CalibImuSampleStatus : std::uint8_t {
    Written,
    Pending,
    Failed,
};

struct CalibImuSampleResult {
    CalibImuSampleStatus status{CalibImuSampleStatus::Pending};
};

} // namespace SmartDrone::Core::Application
