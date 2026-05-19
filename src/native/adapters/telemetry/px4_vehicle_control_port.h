#pragma once

#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "core/ports/slam_session_telemetry.h"
#include "core/ports/vehicle_control_port.h"

namespace smartdrone::adapters::telemetry {

class Px4VehicleControlPort final : public smartdrone::core::ports::IVehicleControlPort,
                                    public smartdrone::core::ports::ISlamSessionTelemetryPort {
  public:
    explicit Px4VehicleControlPort(Px4MavlinkGateway &mavlink);

    void SetFrameTimingTracker(smartdrone::core::application::FrameTimingTracker *tracker) override;
    bool BeginArm(bool arm) override;
    bool BeginEmergencyStop() override;
    bool BeginLand() override;
    bool BeginSetModePosition() override;
    bool BeginSetModeOffboard() override;
    void StartSetpointStreamHz(double hz) override;
    void StopSetpointStream() override;
    void UpdateStreamPosition(float x, float y, float z, float yaw) override;
    void UpdateStreamSetpoint(const smartdrone::core::ports::VehicleSetpointLocalNed &setpoint) override;
    void SendManualControl(const smartdrone::core::ports::VehicleManualControl &input) override;
    bool GetLocalPositionNed(smartdrone::core::ports::VehicleLocalPosition &out, uint64_t maxAgeUs) const override;
    bool GetFlightModeInfo(smartdrone::core::ports::VehicleFlightMode &out) const override;
    bool GetDownwardRange(smartdrone::core::ports::VehicleDownwardRange &out, uint64_t maxAgeUs) const override;
    bool GetDownwardRange(smartdrone::core::ports::SlamRangeSensor &out, uint64_t maxAgeUs) const override;
    bool TryConsumeCommandAck(smartdrone::core::ports::VehicleCommandAckKind command, uint8_t &outResult) override;
    uint8_t PositionModeId() const override;
    uint8_t OffboardModeId() const override;

  private:
    Px4MavlinkGateway &m_mavlink;
};

} // namespace smartdrone::adapters::telemetry
