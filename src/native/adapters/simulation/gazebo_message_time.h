#pragma once

#include <cstdint>

namespace gz::msgs {
class Time;
}

namespace SmartDrone::Adapters::Simulation {

std::uint64_t GazeboMessageTimeToNs(const gz::msgs::Time &time);

} // namespace SmartDrone::Adapters::Simulation
