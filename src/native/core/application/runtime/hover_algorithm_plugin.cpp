#include "core/application/runtime/hover_algorithm_plugin.h"

#include <limits>

namespace SmartDrone::Core::Application {
namespace {

using SmartDrone::Core::Ports::VehicleLocalPosition;
using SmartDrone::Core::Ports::VehicleManualControl;
using SmartDrone::Core::Ports::VehicleSetpointLocalNed;

VehicleSetpointLocalNed BuildZeroVelocitySetpoint()
{
    VehicleSetpointLocalNed setpoint{};
    setpoint.vx = 0.0f;
    setpoint.vy = 0.0f;
    setpoint.vz = 0.0f;
    setpoint.yawRate = 0.0f;
    return setpoint;
}

const char *MissingPositionError(const HoverAlgorithmContext &context)
{
    if (context.avoidanceDecision.shouldHold) {
        return "missing local position for avoidance position hold";
    }
    return "missing local position for offboard setpoint init";
}

} // namespace

VehicleManualControl Px4PositionHoverAlgorithmPlugin::BuildManualHold(
    const HoverAlgorithmContext &context) const
{
    (void)context;
    return VehicleManualControl{};
}

bool Px4PositionHoverAlgorithmPlugin::ApplyOffboardHold(
    SmartDrone::Core::Ports::IVehicleControlPort &vehicleControl,
    const HoverAlgorithmContext &context,
    std::string *err) const
{
    (void)context;
    VehicleLocalPosition local{};
    if (vehicleControl.GetLocalPositionNed(local, 500000)) {
        vehicleControl.UpdateStreamPosition(
            local.x, local.y, local.z,
            std::numeric_limits<float>::quiet_NaN());
        return true;
    }
    vehicleControl.UpdateStreamSetpoint(BuildZeroVelocitySetpoint());
    if (err) {
        *err = MissingPositionError(context);
    }
    return false;
}

} // namespace SmartDrone::Core::Application
