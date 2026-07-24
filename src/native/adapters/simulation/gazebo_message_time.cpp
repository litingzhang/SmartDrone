#include "adapters/simulation/gazebo_message_time.h"

#include <limits>

#include <gz/msgs/time.pb.h>

namespace SmartDrone::Adapters::Simulation {

std::uint64_t GazeboMessageTimeToNs(const gz::msgs::Time &time)
{
    if (time.sec() < 0 || time.nsec() < 0 || time.nsec() >= 1000000000) {
        return 0;
    }
    const auto seconds = static_cast<std::uint64_t>(time.sec());
    if (seconds > std::numeric_limits<std::uint64_t>::max() /
                      1000000000ULL) {
        return 0;
    }
    return seconds * 1000000000ULL +
           static_cast<std::uint64_t>(time.nsec());
}

} // namespace SmartDrone::Adapters::Simulation
