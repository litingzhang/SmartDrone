#pragma once

#include <string>

#include "core/application/runtime/obstacle_avoidance_policy.h"
#include "core/ports/vehicle_control_port.h"

namespace SmartDrone::Core::Application {

struct HoverAlgorithmContext {
    AvoidanceSnapshot avoidanceSnapshot{};
    AvoidanceDecision avoidanceDecision{};
};

class IHoverAlgorithmPlugin {
  public:
    virtual ~IHoverAlgorithmPlugin() = default;

    virtual SmartDrone::Core::Ports::VehicleManualControl BuildManualHold(
        const HoverAlgorithmContext &context) const = 0;
    virtual bool ApplyOffboardHold(
        SmartDrone::Core::Ports::IVehicleControlPort &vehicleControl,
        const HoverAlgorithmContext &context, std::string *err) const = 0;
};

class Px4PositionHoverAlgorithmPlugin final : public IHoverAlgorithmPlugin {
  public:
    SmartDrone::Core::Ports::VehicleManualControl BuildManualHold(
        const HoverAlgorithmContext &context) const override;
    bool ApplyOffboardHold(
        SmartDrone::Core::Ports::IVehicleControlPort &vehicleControl,
        const HoverAlgorithmContext &context, std::string *err) const override;
};

} // namespace SmartDrone::Core::Application
