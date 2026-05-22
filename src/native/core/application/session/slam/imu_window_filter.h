#pragma once

#include <cstddef>
#include <vector>

#include "core/ports/imu_provider.h"

namespace SmartDrone::core::application {

struct ImuWindowValidation {
    size_t inputCount{0};
    size_t outputCount{0};
    size_t droppedNonFinite{0};
    size_t droppedNonMonotonic{0};
    size_t droppedOutOfRange{0};
    double largestGapSec{0.0};
    double firstLeadSec{0.0};
    double tailLagSec{0.0};
    const char *failureReason{nullptr};
};

bool IsFiniteImuReading(const SmartDrone::core::ports::ImuReading &reading);
bool SanitizeImuWindow(std::vector<SmartDrone::core::ports::ImuReading> &vImu,
                       double prevFrameTime, double frameTime,
                       double expectedImuDtSec, ImuWindowValidation &stats);

} // namespace SmartDrone::core::application
