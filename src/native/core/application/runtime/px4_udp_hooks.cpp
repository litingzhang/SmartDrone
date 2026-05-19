#include "core/application/runtime/px4_udp_hooks.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

#include "core/ports/slam_tracking_state.h"

namespace smartdrone::core::application {

using smartdrone::core::ports::VehicleDownwardRange;
using smartdrone::core::ports::VehicleCommandAckKind;
using smartdrone::core::ports::VehicleFlightMode;
using smartdrone::core::ports::VehicleLocalPosition;
using smartdrone::core::ports::VehicleManualControl;
using smartdrone::core::ports::VehicleSetpointLocalNed;

Px4UdpHooks::Px4UdpHooks(Px4UdpHooksConfig config)
    : m_vehicleControl(config.vehicleControl), m_readRuntimeGate(std::move(config.readRuntimeGate)),
      m_publishVehicleFlightState(std::move(config.publishVehicleFlightState))
{
}

Px4UdpHooks::~Px4UdpHooks()
{
}

float Px4UdpHooks::ClampSignedUnit(float value) { return std::max(-1.0f, std::min(1.0f, value)); }

bool Px4UdpHooks::IsTrackingPoseUsable(int trackingState)
{
    return ports::IsSlamTrackingPoseUsable(trackingState);
}

bool Px4UdpHooks::IsPoseQualityUsable(LivePoseQuality quality) { return quality != LivePoseQuality::Lost; }

RuntimeCommandGate Px4UdpHooks::ReadCommandGate() const
{
    RuntimeGateSnapshot snapshot{};
    const bool hasSnapshot = m_readRuntimeGate && m_readRuntimeGate(snapshot);
    const bool vioOk = hasSnapshot && snapshot.runtimeMode == RUNTIME_MODE_SLAM && snapshot.poseValid &&
                       IsTrackingPoseUsable(snapshot.trackingState) && IsPoseQualityUsable(snapshot.poseQuality);
    RuntimeCommandGate gate{};
    gate.localizationReady = vioOk;
    gate.guidedControlReady = vioOk;
    return gate;
}

bool Px4UdpHooks::ArmVehicle(std::string *err)
{
    CancelAutoLanding();
    EnsureManualControlStream();
    SetManualControlNeutral();
    SendManualControlSnapshot();
    if (!m_vehicleControl.BeginArm(true)) {
        if (err)
            *err = "px4 arm send failed";
        return false;
    }
    TrackCommandAck(VehicleCommandAckKind::ArmDisarm, "arm");
    return true;
}

bool Px4UdpHooks::DisarmVehicle(std::string *err)
{
    CancelAutoLanding();
    m_vehicleControl.StopSetpointStream();
    m_streamStarted.store(false, std::memory_order_relaxed);
    SetManualControlNeutral();
    SendManualControlSnapshot();
    DisableRemoteControl(true);
    if (!m_vehicleControl.BeginArm(false)) {
        if (err)
            *err = "px4 disarm send failed";
        return false;
    }
    TrackCommandAck(VehicleCommandAckKind::ArmDisarm, "disarm");
    return true;
}

bool Px4UdpHooks::StopVehicleImmediately(std::string *err)
{
    CancelAutoLanding();
    m_vehicleControl.StopSetpointStream();
    m_streamStarted.store(false, std::memory_order_relaxed);
    DisableRemoteControl(true);
    if (!m_vehicleControl.BeginEmergencyStop()) {
        if (err)
            *err = "px4 emergency stop send failed";
        return false;
    }
    TrackCommandAck(VehicleCommandAckKind::ArmDisarm, "emergency stop");
    return true;
}

bool Px4UdpHooks::EnterGuidedControl(std::string *err)
{
    CancelAutoLanding();

    // OFFBOARD relies on setpoint stream; disable manual joystick streaming.
    m_manualControlStreaming.store(false, std::memory_order_relaxed);
    SetManualControlNeutral();
    SendManualControlSnapshot();

    VehicleLocalPosition local{};
    if (!m_vehicleControl.GetLocalPositionNed(local, 500000)) {
        if (err) {
            *err = "missing local position for offboard setpoint init";
        }
        return false;
    }

    // Initialize OFFBOARD with a hold-position setpoint around current pose.
    m_vehicleControl.UpdateStreamPosition(local.x, local.y, local.z, NAN);

    if (!EnsureSetpointStream()) {
        if (err) {
            *err = "setpoint stream start failed";
        }
        return false;
    }

    m_remoteModeRequested.store(true, std::memory_order_relaxed);
    m_offboardModeRequested.store(true, std::memory_order_relaxed);
    if (!EnsureOffboardMode(true, err)) {
        m_vehicleControl.StopSetpointStream();
        m_streamStarted.store(false, std::memory_order_relaxed);
        m_remoteModeRequested.store(false, std::memory_order_relaxed);
        m_offboardModeRequested.store(false, std::memory_order_relaxed);
        if (err && err->empty()) {
            *err = "px4 offboard mode rejected";
        }
        return false;
    }
    return true;
}

bool Px4UdpHooks::HoldVehicle(std::string *err)
{
    (void)err;
    CancelAutoLanding();
    m_vehicleControl.StopSetpointStream();
    m_streamStarted.store(false, std::memory_order_relaxed);
    m_offboardModeRequested.store(false, std::memory_order_relaxed);
    EnsureManualControlStream();
    SetManualControlNeutral();
    SendManualControlSnapshot();
    return true;
}

bool Px4UdpHooks::EnterPositionControl(std::string *err)
{
    CancelAutoLanding();

    // POSITION mode should not keep OFFBOARD setpoint stream alive.
    m_vehicleControl.StopSetpointStream();
    m_streamStarted.store(false, std::memory_order_relaxed);

    EnsureManualControlStream();
    SetManualControlNeutral();
    SendManualControlSnapshot();

    VehicleFlightMode beforeMode{};
    const bool haveBeforeMode = m_vehicleControl.GetFlightModeInfo(beforeMode);
    if (haveBeforeMode) {
        std::cout << "[px4] POSITION requested from mode main=" << static_cast<int>(beforeMode.mainMode)
                  << " sub=" << static_cast<int>(beforeMode.subMode) << " armed=" << (beforeMode.armed ? 1 : 0)
                  << "\n";
    } else {
        std::cout << "[px4] POSITION requested (no heartbeat mode snapshot)\n";
    }

    m_remoteModeRequested.store(true, std::memory_order_relaxed);
    m_offboardModeRequested.store(false, std::memory_order_relaxed);
    if (!EnsurePositionMode(true, err)) {
        m_remoteModeRequested.store(false, std::memory_order_relaxed);
        if (err && err->empty()) {
            *err = "px4 position mode rejected";
        }
        return false;
    }

    VehicleFlightMode afterMode{};
    if (m_vehicleControl.GetFlightModeInfo(afterMode)) {
        std::cout << "[px4] POSITION result mode main=" << static_cast<int>(afterMode.mainMode)
                  << " sub=" << static_cast<int>(afterMode.subMode) << " armed=" << (afterMode.armed ? 1 : 0)
                  << "\n";
    }
    return true;
}

bool Px4UdpHooks::LandVehicle(std::string *err)
{
    CancelAutoLanding();
    m_vehicleControl.StopSetpointStream();
    m_streamStarted.store(false, std::memory_order_relaxed);
    SetManualControlNeutral();
    SendManualControlSnapshot();
    DisableRemoteControl(true);
    if (!m_vehicleControl.BeginLand()) {
        if (err) {
            *err = "px4 land command send failed";
        }
        return false;
    }
    TrackCommandAck(VehicleCommandAckKind::Land, "land");
    std::cout << "[land] land command sent to vehicle\n";
    return true;
}

bool Px4UdpHooks::ApplyMoveGoal(const MoveGoal &goal, std::string *err)
{
    CancelAutoLanding();
    if (goal.isRcJoystick) {
        return ApplyRcMoveGoal(goal);
    }
    return ApplyOffboardMoveGoal(goal, err);
}

bool Px4UdpHooks::ApplyRcMoveGoal(const MoveGoal &goal)
{
    VehicleManualControl input{};
    input.throttleNorm = ClampSignedUnit(goal.throttleNorm);
    input.yawNorm = ClampSignedUnit(goal.yawNorm);
    input.pitchNorm = ClampSignedUnit(goal.pitchNorm);
    input.rollNorm = ClampSignedUnit(goal.rollNorm);
    EnsureManualControlStream();
    {
        std::lock_guard<std::mutex> lock(m_manualControlMtx);
        m_manualControlInput = input;
    }
    SendManualControlSnapshot();
    return true;
}

bool Px4UdpHooks::EnsureOffboardMoveReady(std::string *err)
{
    if (!m_offboardModeRequested.load(std::memory_order_relaxed)) {
        m_vehicleControl.StopSetpointStream();
        m_streamStarted.store(false, std::memory_order_relaxed);
        if (err) {
            *err = "setpoint move only allowed in offboard mode";
        }
        return false;
    }
    if (!EnsureSetpointStream()) {
        if (err) {
            *err = "setpoint stream start failed";
        }
        return false;
    }
    return true;
}

VehicleSetpointLocalNed Px4UdpHooks::BuildMoveSetpoint(const MoveGoal &goal) const
{
    VehicleSetpointLocalNed setpoint{};
    if (goal.isVelocity) {
        setpoint.x = NAN;
        setpoint.y = NAN;
        setpoint.z = NAN;
        setpoint.vx = goal.vx;
        setpoint.vy = goal.vy;
        setpoint.vz = goal.vz;
        setpoint.yaw = NAN;
        setpoint.yawRate = goal.yawRate;
    } else {
        setpoint.x = goal.x;
        setpoint.y = goal.y;
        setpoint.z = goal.z;
        setpoint.vx = NAN;
        setpoint.vy = NAN;
        setpoint.vz = NAN;
        setpoint.yaw = goal.yaw;
        setpoint.yawRate = NAN;
    }
    return setpoint;
}

bool Px4UdpHooks::ApplyOffboardMoveGoal(const MoveGoal &goal, std::string *err)
{
    if (!EnsureOffboardMoveReady(err)) {
        return false;
    }
    const VehicleSetpointLocalNed setpoint = BuildMoveSetpoint(goal);
    m_vehicleControl.UpdateStreamSetpoint(setpoint);
    if (!EnsureOffboardMode(false, nullptr)) {
        if (err) {
            *err = "offboard mode not active";
        }
        return false;
    }
    return true;
}

void Px4UdpHooks::CancelAutoLanding()
{
    bool wasActive = false;
    {
        std::lock_guard<std::mutex> lock(m_autoLandingMtx);
        wasActive = m_autoLanding.active;
        m_autoLanding = AutoLandingState{};
    }
    if (wasActive) {
        SetManualControlNeutral();
    }
}

bool Px4UdpHooks::IsAutoLandingActive() const
{
    std::lock_guard<std::mutex> lock(m_autoLandingMtx);
    return m_autoLanding.active;
}

bool Px4UdpHooks::EnsureSetpointStream()
{
    bool expected = false;
    if (m_streamStarted.compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
        m_vehicleControl.StartSetpointStreamHz(20.0);
    }
    return true;
}

void Px4UdpHooks::EnsureManualControlStream() { m_manualControlStreaming.store(true, std::memory_order_relaxed); }

void Px4UdpHooks::DisableRemoteControl(bool stopManualStream)
{
    m_remoteModeRequested.store(false, std::memory_order_relaxed);
    m_offboardModeRequested.store(false, std::memory_order_relaxed);
    if (stopManualStream) {
        m_manualControlStreaming.store(false, std::memory_order_relaxed);
    }
    std::lock_guard<std::mutex> lock(m_remoteModeMtx);
    m_modeChangeRequested = false;
    m_requestedMainMode = 0;
    m_lastPositionModeRequest = std::chrono::steady_clock::time_point{};
}

void Px4UdpHooks::SetManualControlNeutral()
{
    std::lock_guard<std::mutex> lock(m_manualControlMtx);
    m_manualControlInput = VehicleManualControl{};
}

void Px4UdpHooks::SetManualControlInput(const VehicleManualControl &input)
{
    std::lock_guard<std::mutex> lock(m_manualControlMtx);
    m_manualControlInput = input;
}

VehicleManualControl Px4UdpHooks::GetManualControlSnapshot() const
{
    std::lock_guard<std::mutex> lock(m_manualControlMtx);
    return m_manualControlInput;
}

void Px4UdpHooks::SendManualControlSnapshot()
{
    m_vehicleControl.SendManualControl(GetManualControlSnapshot());
}

bool Px4UdpHooks::EnsureFlightMode(uint8_t mainMode, bool force, std::string *err, const char *modeName)
{
    VehicleFlightMode currentMode{};
    const bool haveCurrentMode = m_vehicleControl.GetFlightModeInfo(currentMode);
    if (haveCurrentMode && currentMode.mainMode == mainMode) {
        return true;
    }

    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(m_remoteModeMtx);
        if (!force && m_modeChangeRequested && m_requestedMainMode == mainMode &&
            (now - m_lastPositionModeRequest) < std::chrono::milliseconds(600)) {
            return true;
        }
        m_modeChangeRequested = true;
        m_requestedMainMode = mainMode;
        m_lastPositionModeRequest = now;
    }

    const bool ok = (mainMode == m_vehicleControl.PositionModeId()) ? m_vehicleControl.BeginSetModePosition()
                                                                    : m_vehicleControl.BeginSetModeOffboard();
    if (!ok) {
        if (err) {
            *err = std::string("px4 ") + (modeName ? modeName : "mode") + " send failed";
        }
        return false;
    }

    TrackCommandAck(VehicleCommandAckKind::SetMode, modeName ? modeName : "mode");
    std::cout << "[px4] remote mode -> " << (modeName ? modeName : "unknown") << "\n";
    return true;
}

bool Px4UdpHooks::EnsurePositionMode(bool force, std::string *err)
{
    return EnsureFlightMode(m_vehicleControl.PositionModeId(), force, err, "POSCTL");
}

bool Px4UdpHooks::EnsureOffboardMode(bool force, std::string *err)
{
    return EnsureFlightMode(m_vehicleControl.OffboardModeId(), force, err, "OFFBOARD");
}

bool Px4UdpHooks::PrepareAutoLandingRangeWindow(const VehicleDownwardRange &range,
                                                std::chrono::steady_clock::time_point now,
                                                bool &shouldDisarm)
{
    if (!m_autoLanding.active) {
        return false;
    }
    if (!m_autoLanding.haveRangeWindow) {
        m_autoLanding.haveRangeWindow = true;
        m_autoLanding.rangeWindowMin = range.currentDistance;
        m_autoLanding.rangeWindowMax = range.currentDistance;
        m_autoLanding.rangeWindowStart = now;
        return false;
    }
    m_autoLanding.rangeWindowMin = std::min(m_autoLanding.rangeWindowMin, range.currentDistance);
    m_autoLanding.rangeWindowMax = std::max(m_autoLanding.rangeWindowMax, range.currentDistance);
    if ((now - m_autoLanding.rangeWindowStart) < kAutoLandStableDuration) {
        return false;
    }
    const bool rangeStable =
        (m_autoLanding.rangeWindowMax - m_autoLanding.rangeWindowMin) <= kAutoLandStableRangeDeltaM;
    const bool nearGround = range.currentDistance <= kAutoLandNearGroundM;
    shouldDisarm = rangeStable && nearGround &&
                   (m_autoLanding.lastDisarmAttempt.time_since_epoch().count() == 0 ||
                    (now - m_autoLanding.lastDisarmAttempt) >= kAutoLandDisarmRetry);
    return true;
}

void Px4UdpHooks::UpdateAutoLanding()
{
    if (!IsAutoLandingActive()) {
        return;
    }

    VehicleManualControl input{};
    input.throttleNorm = kAutoLandThrottleNorm;
    SetManualControlInput(input);

    VehicleDownwardRange range{};
    if (!m_vehicleControl.GetDownwardRange(range, kAutoLandRangeMaxAgeUs) || !std::isfinite(range.currentDistance)) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    bool shouldDisarm = false;
    {
        std::lock_guard<std::mutex> lock(m_autoLandingMtx);
        if (!PrepareAutoLandingRangeWindow(range, now, shouldDisarm)) {
            return;
        }
        if (shouldDisarm) {
            m_autoLanding.lastDisarmAttempt = now;
        } else {
            m_autoLanding.rangeWindowMin = range.currentDistance;
            m_autoLanding.rangeWindowMax = range.currentDistance;
            m_autoLanding.rangeWindowStart = now;
        }
    }

    if (!shouldDisarm) {
        return;
    }

    BeginAutoLandingDisarm();
    CancelAutoLanding();
    SetManualControlNeutral();
    SendManualControlSnapshot();
    DisableRemoteControl(true);
    std::cout << "[land] touchdown detected by range stability, disarmed\n";
}

void Px4UdpHooks::BeginAutoLandingDisarm()
{
    if (m_vehicleControl.BeginArm(false)) {
        TrackCommandAck(VehicleCommandAckKind::ArmDisarm, "auto land disarm");
    }
}

void Px4UdpHooks::TrackCommandAck(VehicleCommandAckKind command, const std::string &label)
{
    std::lock_guard<std::mutex> lock(m_commandAckMtx);
    m_pendingCommandAck.active = true;
    m_pendingCommandAck.command = command;
    m_pendingCommandAck.label = label;
    m_pendingCommandAck.deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(900);
}

void Px4UdpHooks::StepCommandAck()
{
    PendingCommandAck pending{};
    {
        std::lock_guard<std::mutex> lock(m_commandAckMtx);
        pending = m_pendingCommandAck;
    }
    if (!pending.active) {
        return;
    }

    uint8_t result = 255;
    if (m_vehicleControl.TryConsumeCommandAck(pending.command, result)) {
        std::cout << "[px4] " << pending.label << " ack=" << static_cast<int>(result) << "\n";
        ClearCommandAckIfCurrent(pending.command, pending.label);
        return;
    }
    if (std::chrono::steady_clock::now() < pending.deadline) {
        return;
    }
    std::cout << "[px4] " << pending.label << " ack timeout\n";
    ClearCommandAckIfCurrent(pending.command, pending.label);
}

void Px4UdpHooks::ClearCommandAckIfCurrent(VehicleCommandAckKind command, const std::string &label)
{
    std::lock_guard<std::mutex> lock(m_commandAckMtx);
    if (!m_pendingCommandAck.active || m_pendingCommandAck.command != command || m_pendingCommandAck.label != label) {
        return;
    }
    m_pendingCommandAck = PendingCommandAck{};
}

void Px4UdpHooks::StepManualControl()
{
    StepCommandAck();
    VehicleFlightMode flightMode{};
    if (m_vehicleControl.GetFlightModeInfo(flightMode) && m_publishVehicleFlightState) {
        m_publishVehicleFlightState(flightMode.armed, flightMode.mainMode, flightMode.subMode);
    }
    UpdateAutoLanding();
    if (m_manualControlStreaming.load(std::memory_order_relaxed)) {
        SendManualControlSnapshot();
    }
    if (!m_remoteModeRequested.load(std::memory_order_relaxed)) {
        return;
    }
    if (m_offboardModeRequested.load(std::memory_order_relaxed)) {
        EnsureOffboardMode(false, nullptr);
        return;
    }
    EnsurePositionMode(false, nullptr);
}

} // namespace smartdrone::core::application
