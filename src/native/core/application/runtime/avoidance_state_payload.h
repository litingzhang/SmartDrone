#pragma once

#include <cstdint>
#include <vector>

#include "core/application/runtime/udp_command_runtime_config.h"

namespace SmartDrone::Core::Application {

std::vector<uint8_t> BuildAvoidanceStatePayload(
    const UdpRuntimeStateSnapshot &snapshot);

} // namespace SmartDrone::Core::Application
