#pragma once

#include <cstdint>

namespace smartdrone::core::application {

enum class CalibFrameCaptureStatus : std::uint8_t {
    Captured,
    Pending,
    SessionAbort,
};

struct CalibFrameCaptureResult {
    CalibFrameCaptureStatus status{CalibFrameCaptureStatus::Pending};
};

} // namespace smartdrone::core::application
