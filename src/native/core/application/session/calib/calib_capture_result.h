#pragma once

#include <cstdint>

namespace SmartDrone::core::application {

enum class CalibFrameCaptureStatus : std::uint8_t {
    Captured,
    Pending,
    SessionAbort,
};

struct CalibFrameCaptureResult {
    CalibFrameCaptureStatus status{CalibFrameCaptureStatus::Pending};
};

} // namespace SmartDrone::core::application
