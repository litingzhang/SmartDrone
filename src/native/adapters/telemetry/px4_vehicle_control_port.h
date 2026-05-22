#pragma once

#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "core/ports/slam_session_telemetry.h"
#include "core/ports/vehicle_control_port.h"

namespace SmartDrone::Adapters::Telemetry {

class Px4VehicleControlPort final : public SmartDrone::Core::Ports::IVehicleControlPort,
                                    public SmartDrone::Core::Ports::ISlamSessionTelemetryPort {
  public:
    explicit Px4VehicleControlPort(Px4MavlinkGateway &mavlink);

    void SetFrameTimingTracker(SmartDrone::Core::Application::FrameTimingTracker *tracker) override;
    bool BeginArm(bool arm) override;
    bool BeginEmergencyStop() override;
    bool BeginLand() override;
    bool BeginSetModePosition() override;
    bool BeginSetModeOffboard() override;
    void StartSetpointStreamHz(double hz) override;
    void StopSetpointStream() override;
    void UpdateStreamPosition(float x, float y, float z, float yaw) override;
    void UpdateStreamSetpoint(const SmartDrone::Core::Ports::VehicleSetpointLocalNed &setpoint) override;
    void SendManualControl(const SmartDrone::Core::Ports::VehicleManualControl &input) override;
    bool GetLocalPositionNed(SmartDrone::Core::Ports::VehicleLocalPosition &out, uint64_t maxAgeUs) const override;
    bool GetFlightModeInfo(SmartDrone::Core::Ports::VehicleFlightMode &out) const override;
    bool GetDownwardRange(SmartDrone::Core::Ports::VehicleDownwardRange &out, uint64_t maxAgeUs) const override;
    bool GetDownwardRange(SmartDrone::Core::Ports::SlamRangeSensor &out, uint64_t maxAgeUs) const override;
    bool TryConsumeCommandAck(SmartDrone::Core::Ports::VehicleCommandAckKind command, uint8_t &outResult) override;
    uint8_t PositionModeId() const override;
    uint8_t OffboardModeId() const override;

  private:
    Px4MavlinkGateway &m_mavlink;
};

} // namespace SmartDrone::Adapters::Telemetry
