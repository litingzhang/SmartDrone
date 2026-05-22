#pragma once

#include <cstdint>

#include "common/epg/epg.h"

namespace SmartDrone::core::application {

constexpr Epg::PortId SYSTEM_RUNTIME_PULSE_PORT = 0;

struct SystemRuntimePulse {
    std::uint64_t sequence{0};
};

void DrainSystemRuntimePulse(Epg::TaskContext &context);
void PushSystemRuntimePulse(Epg::TaskContext &context,
                            std::uint64_t &sequence);

} // namespace SmartDrone::core::application
