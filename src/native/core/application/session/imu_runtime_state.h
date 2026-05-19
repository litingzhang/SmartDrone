#pragma once

#include <atomic>
#include <cstdint>

#include "core/application/state/imu_buffer.h"

namespace smartdrone::core::application {

struct ImuThreadState {
    ImuBuffer imuBuffer;
    std::atomic<bool> imuOk{false};
    std::atomic<uint64_t> imuCnt{0};
    std::atomic<uint64_t> imuDrop{0};
    std::atomic<float> accelLsbPerG{0.0f};
    std::atomic<float> gyroLsbPerDps{0.0f};
};

} // namespace smartdrone::core::application
