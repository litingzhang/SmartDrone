#include "adapters/simulation/gazebo_pose_math.h"

#include <cmath>

namespace SmartDrone::Adapters::Simulation {
namespace {

struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

Quaternion Multiply(const Quaternion &left, const Quaternion &right)
{
    return {
        left.w * right.w - left.x * right.x - left.y * right.y -
            left.z * right.z,
        left.w * right.x + left.x * right.w + left.y * right.z -
            left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w +
            left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x +
            left.z * right.w};
}

Quaternion ConvertQuaternion(const GazeboPoseValue &input)
{
    constexpr double DIAGONAL = 0.7071067811865476;
    const Quaternion enuToNed{0.0, DIAGONAL, DIAGONAL, 0.0};
    const Quaternion enuFromFlu{input.qw, input.qx, input.qy, input.qz};
    const Quaternion fluFromFrd{0.0, 1.0, 0.0, 0.0};
    Quaternion result = Multiply(Multiply(enuToNed, enuFromFlu),
                                 fluFromFrd);
    const double norm = std::sqrt(result.w * result.w + result.x * result.x +
                                  result.y * result.y + result.z * result.z);
    if (norm <= 1.0e-9) {
        return {};
    }
    const double sign = result.w < 0.0 ? -1.0 : 1.0;
    return {sign * result.w / norm, sign * result.x / norm,
            sign * result.y / norm, sign * result.z / norm};
}

} // namespace

SmartDrone::Core::Ports::PoseEstimate ConvertGazeboPoseToNedFrd(
    const GazeboPoseValue &input)
{
    const Quaternion orientation = ConvertQuaternion(input);
    return {true, static_cast<float>(input.y), static_cast<float>(input.x),
            static_cast<float>(-input.z), static_cast<float>(orientation.w),
            static_cast<float>(orientation.x),
            static_cast<float>(orientation.y),
            static_cast<float>(orientation.z)};
}

SmartDrone::Core::Ports::VelocityEstimate EstimateGazeboNedVelocity(
    const GazeboNedPoseSample &previous,
    const GazeboNedPoseSample &current)
{
    constexpr std::uint64_t MAX_VELOCITY_INTERVAL_NS = 500000000ULL;
    if (!previous.pose.valid || !current.pose.valid ||
        previous.resetCounter != current.resetCounter ||
        current.measurementTimestampNs <= previous.measurementTimestampNs) {
        return {};
    }
    const std::uint64_t elapsedNs = current.measurementTimestampNs -
                                    previous.measurementTimestampNs;
    if (elapsedNs > MAX_VELOCITY_INTERVAL_NS) {
        return {};
    }
    const float inverseSeconds = static_cast<float>(1.0e9 / elapsedNs);
    return {(current.pose.x - previous.pose.x) * inverseSeconds,
            (current.pose.y - previous.pose.y) * inverseSeconds,
            (current.pose.z - previous.pose.z) * inverseSeconds, true};
}

} // namespace SmartDrone::Adapters::Simulation
