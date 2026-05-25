#pragma once

#include <cstdint>

#include "core/domain/imu_sample.h"

namespace SmartDrone::Core::Application {

enum class CalibImuSampleStatus : std::uint8_t {
    Ready,
    Pending,
    Failed,
};

struct CalibImuSampleResult {
    CalibImuSampleStatus status{CalibImuSampleStatus::Pending};
    ImuSample sample{};
};

} // namespace SmartDrone::Core::Application
