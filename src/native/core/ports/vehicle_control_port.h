#pragma once

#include <cstdint>
#include <limits>

namespace SmartDrone::core::ports {

struct VehicleManualControl {
    float throttleNorm{0.0f};
    float yawNorm{0.0f};
    float pitchNorm{0.0f};
    float rollNorm{0.0f};
};

struct VehicleDownwardRange {
    float currentDistance{std::numeric_limits<float>::quiet_NaN()};
};

struct VehicleLocalPosition {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

struct VehicleFlightMode {
    uint8_t mainMode{0};
    uint8_t subMode{0};
    bool armed{false};
};

struct VehicleSetpointLocalNed {
    float x{std::numeric_limits<float>::quiet_NaN()};
    float y{std::numeric_limits<float>::quiet_NaN()};
    float z{std::numeric_limits<float>::quiet_NaN()};
    float vx{std::numeric_limits<float>::quiet_NaN()};
    float vy{std::numeric_limits<float>::quiet_NaN()};
    float vz{std::numeric_limits<float>::quiet_NaN()};
    float yaw{std::numeric_limits<float>::quiet_NaN()};
    float yawRate{std::numeric_limits<float>::quiet_NaN()};
};

enum class VehicleCommandAckKind {
    ArmDisarm,
    Land,
    SetMode,
};

class IVehicleControlPort {
  public:
    virtual ~IVehicleControlPort() = default;

    virtual bool BeginArm(bool arm) = 0;
    virtual bool BeginEmergencyStop() = 0;
    virtual bool BeginLand() = 0;
    virtual bool BeginSetModePosition() = 0;
    virtual bool BeginSetModeOffboard() = 0;
    virtual void StartSetpointStreamHz(double hz) = 0;
    virtual void StopSetpointStream() = 0;
    virtual void UpdateStreamPosition(float x, float y, float z, float yaw) = 0;
    virtual void UpdateStreamSetpoint(const VehicleSetpointLocalNed &setpoint) = 0;
    virtual void SendManualControl(const VehicleManualControl &input) = 0;
    virtual bool GetLocalPositionNed(VehicleLocalPosition &out, uint64_t maxAgeUs) const = 0;
    virtual bool GetFlightModeInfo(VehicleFlightMode &out) const = 0;
    virtual bool GetDownwardRange(VehicleDownwardRange &out, uint64_t maxAgeUs) const = 0;
    virtual bool TryConsumeCommandAck(VehicleCommandAckKind command, uint8_t &outResult) = 0;
    virtual uint8_t PositionModeId() const = 0;
    virtual uint8_t OffboardModeId() const = 0;
};

} // namespace SmartDrone::core::ports
