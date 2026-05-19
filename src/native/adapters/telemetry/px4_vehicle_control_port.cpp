#include "adapters/telemetry/px4_vehicle_control_port.h"

namespace smartdrone::adapters::telemetry {

using smartdrone::core::ports::VehicleDownwardRange;
using smartdrone::core::ports::VehicleCommandAckKind;
using smartdrone::core::ports::VehicleFlightMode;
using smartdrone::core::ports::VehicleLocalPosition;
using smartdrone::core::ports::VehicleManualControl;
using smartdrone::core::ports::VehicleSetpointLocalNed;
using smartdrone::core::ports::SlamRangeSensor;

namespace {

Px4MavlinkGateway::SetpointLocalNED ToPx4Setpoint(const VehicleSetpointLocalNed &input)
{
    Px4MavlinkGateway::SetpointLocalNED output{};
    output.x = input.x;
    output.y = input.y;
    output.z = input.z;
    output.vx = input.vx;
    output.vy = input.vy;
    output.vz = input.vz;
    output.yaw = input.yaw;
    output.yawspeed = input.yawRate;
    return output;
}

Px4MavlinkGateway::ManualControlInput ToPx4ManualControl(const VehicleManualControl &input)
{
    Px4MavlinkGateway::ManualControlInput output{};
    output.throttleNorm = input.throttleNorm;
    output.yawNorm = input.yawNorm;
    output.pitchNorm = input.pitchNorm;
    output.rollNorm = input.rollNorm;
    return output;
}

uint16_t ToPx4CommandId(VehicleCommandAckKind command)
{
    switch (command) {
    case VehicleCommandAckKind::ArmDisarm:
        return MAV_CMD_COMPONENT_ARM_DISARM;
    case VehicleCommandAckKind::Land:
        return MAV_CMD_NAV_LAND;
    case VehicleCommandAckKind::SetMode:
        return MAV_CMD_DO_SET_MODE;
    }
    return 0;
}

} // namespace

Px4VehicleControlPort::Px4VehicleControlPort(Px4MavlinkGateway &mavlink) : m_mavlink(mavlink) {}

void Px4VehicleControlPort::SetFrameTimingTracker(smartdrone::core::application::FrameTimingTracker *tracker)
{
    m_mavlink.SetFrameTimingTracker(tracker);
}

bool Px4VehicleControlPort::BeginArm(bool arm) { return m_mavlink.BeginArm(arm); }

bool Px4VehicleControlPort::BeginEmergencyStop() { return m_mavlink.BeginEmergencyStop(); }

bool Px4VehicleControlPort::BeginLand() { return m_mavlink.BeginLand(); }

bool Px4VehicleControlPort::BeginSetModePosition() { return m_mavlink.BeginSetModePosition(); }

bool Px4VehicleControlPort::BeginSetModeOffboard() { return m_mavlink.BeginSetModeOffboard(); }

void Px4VehicleControlPort::StartSetpointStreamHz(double hz) { m_mavlink.StartSetpointStreamHz(hz); }

void Px4VehicleControlPort::StopSetpointStream() { m_mavlink.StopSetpointStream(); }

void Px4VehicleControlPort::UpdateStreamPosition(float x, float y, float z, float yaw)
{
    m_mavlink.UpdateStreamPosition(x, y, z, yaw);
}

void Px4VehicleControlPort::UpdateStreamSetpoint(const VehicleSetpointLocalNed &setpoint)
{
    m_mavlink.UpdateStreamSetpoint(ToPx4Setpoint(setpoint));
}

void Px4VehicleControlPort::SendManualControl(const VehicleManualControl &input)
{
    m_mavlink.SendManualControl(ToPx4ManualControl(input));
}

bool Px4VehicleControlPort::GetLocalPositionNed(VehicleLocalPosition &out, uint64_t maxAgeUs) const
{
    Px4MavlinkGateway::LocalPositionNed local{};
    if (!m_mavlink.GetLocalPositionNed(local, maxAgeUs)) {
        return false;
    }
    out.x = local.x;
    out.y = local.y;
    out.z = local.z;
    return true;
}

bool Px4VehicleControlPort::GetFlightModeInfo(VehicleFlightMode &out) const
{
    Px4MavlinkGateway::FlightModeInfo info{};
    if (!m_mavlink.GetFlightModeInfo(info)) {
        return false;
    }
    out.mainMode = info.mainMode;
    out.subMode = info.subMode;
    out.armed = info.armed;
    return true;
}

bool Px4VehicleControlPort::GetDownwardRange(VehicleDownwardRange &out, uint64_t maxAgeUs) const
{
    Px4MavlinkGateway::DownwardDistanceSensor range{};
    if (!m_mavlink.GetDownwardDistanceSensor(range, maxAgeUs)) {
        return false;
    }
    out.currentDistance = range.currentDistance;
    return true;
}

bool Px4VehicleControlPort::GetDownwardRange(SlamRangeSensor &out, uint64_t maxAgeUs) const
{
    Px4MavlinkGateway::DownwardDistanceSensor range{};
    if (!m_mavlink.GetDownwardDistanceSensor(range, maxAgeUs)) {
        return false;
    }
    out.currentDistance = range.currentDistance;
    out.signalQuality = range.signalQuality;
    return true;
}

bool Px4VehicleControlPort::TryConsumeCommandAck(VehicleCommandAckKind command, uint8_t &outResult)
{
    return m_mavlink.TryConsumeCommandAck(ToPx4CommandId(command), outResult);
}

uint8_t Px4VehicleControlPort::PositionModeId() const
{
    return Px4MavlinkGateway::PX4_CUSTOM_MAIN_MODE_POSCTL;
}

uint8_t Px4VehicleControlPort::OffboardModeId() const
{
    return Px4MavlinkGateway::PX4_CUSTOM_MAIN_MODE_OFFBOARD;
}

} // namespace smartdrone::adapters::telemetry
