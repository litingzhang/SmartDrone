#pragma once

#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "core/ports/slam_session_telemetry.h"
#include "core/ports/vehicle_control_port.h"

namespace SmartDrone::adapters::telemetry {

class Px4VehicleControlPort final : public SmartDrone::core::ports::IVehicleControlPort,
                                    public SmartDrone::core::ports::ISlamSessionTelemetryPort {
  public:
    explicit Px4VehicleControlPort(Px4MavlinkGateway &mavlink);

    void SetFrameTimingTracker(SmartDrone::core::application::FrameTimingTracker *tracker) override;
    bool BeginArm(bool arm) override;
    bool BeginEmergencyStop() override;
    bool BeginLand() override;
    bool BeginSetModePosition() override;
    bool BeginSetModeOffboard() override;
    void StartSetpointStreamHz(double hz) override;
    void StopSetpointStream() override;
    void UpdateStreamPosition(float x, float y, float z, float yaw) override;
    void UpdateStreamSetpoint(const SmartDrone::core::ports::VehicleSetpointLocalNed &setpoint) override;
    void SendManualControl(const SmartDrone::core::ports::VehicleManualControl &input) override;
    bool GetLocalPositionNed(SmartDrone::core::ports::VehicleLocalPosition &out, uint64_t maxAgeUs) const override;
    bool GetFlightModeInfo(SmartDrone::core::ports::VehicleFlightMode &out) const override;
    bool GetDownwardRange(SmartDrone::core::ports::VehicleDownwardRange &out, uint64_t maxAgeUs) const override;
    bool GetDownwardRange(SmartDrone::core::ports::SlamRangeSensor &out, uint64_t maxAgeUs) const override;
    bool TryConsumeCommandAck(SmartDrone::core::ports::VehicleCommandAckKind command, uint8_t &outResult) override;
    uint8_t PositionModeId() const override;
    uint8_t OffboardModeId() const override;

  private:
    Px4MavlinkGateway &m_mavlink;
};

} // namespace SmartDrone::adapters::telemetry
