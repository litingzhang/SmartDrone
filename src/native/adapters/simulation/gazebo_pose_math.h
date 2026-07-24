#pragma once

#include <cstdint>

#include "core/ports/pose_publisher.h"

namespace SmartDrone::Adapters::Simulation {

struct GazeboPoseValue {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double qw{1.0};
    double qx{0.0};
    double qy{0.0};
    double qz{0.0};
};

struct GazeboNedPoseSample {
    std::uint64_t measurementTimestampNs{0};
    std::uint8_t resetCounter{0};
    SmartDrone::Core::Ports::PoseEstimate pose;
};

SmartDrone::Core::Ports::PoseEstimate ConvertGazeboPoseToNedFrd(
    const GazeboPoseValue &input);
SmartDrone::Core::Ports::VelocityEstimate EstimateGazeboNedVelocity(
    const GazeboNedPoseSample &previous,
    const GazeboNedPoseSample &current);

} // namespace SmartDrone::Adapters::Simulation
