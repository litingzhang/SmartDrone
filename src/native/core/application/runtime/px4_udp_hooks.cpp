#include "core/application/runtime/px4_udp_hooks.h"

#include <cmath>
#include <iostream>

#include "common/thread_launch.h"
#include "core/ports/slam_tracking_state.h"

namespace smartdrone::core::application {

Px4UdpHooks::Px4UdpHooks(Px4MavlinkGateway &mavlink, LivePoseState &livePose) : m_mavlink(mavlink), m_livePose(livePose)
{
    m_manualLoop = smartdrone::common::StartThread(
        smartdrone::common::MakeThreadLaunchInfo(smartdrone::common::ThreadRole::ManualControl, "Px4UdpHooks"),
        [this]() { ManualControlLoop(); });
}

Px4UdpHooks::~Px4UdpHooks()
{
    m_manualLoopStop.store(true, std::memory_order_relaxed);
    if (m_manualLoop.joinable()) {
        m_manualLoop.join();
    }
}

float Px4UdpHooks::ClampSignedUnit(float value) { return std::max(-1.0f, std::min(1.0f, value)); }

bool Px4UdpHooks::IsTrackingPoseUsable(int trackingState)
{
    return ports::IsSlamTrackingPoseUsable(trackingState);
}

bool Px4UdpHooks::IsOdomQualityUsable(OdomQualityMode quality) { return quality != OdomQualityMode::LOST; }

VehicleGate Px4UdpHooks::GetGate() const
{
    LivePoseState::Snapshot snapshot{};
    const bool hasSnapshot = m_livePose.ReadSnapshot(snapshot);
    const bool vioOk = hasSnapshot && snapshot.runtimeMode == RUNTIME_MODE_SLAM && snapshot.poseValid &&
                       IsTrackingPoseUsable(snapshot.trackingState) && IsOdomQualityUsable(snapshot.odomQuality);
    VehicleGate gate{};
    gate.vioOk = vioOk;
    gate.offboardReady = vioOk;
    return gate;
}

bool Px4UdpHooks::Arm(std::string *err)
{
    CancelAutoLanding();
    EnsureManualControlStream();
    SetManualControlNeutral();
    SendManualControlSnapshot();
    if (!m_mavlink.Arm(true)) {
        if (err)
            *err = "px4 arm rejected";
        return false;
    }
    return true;
}

bool Px4UdpHooks::Disarm(std::string *err)
{
    CancelAutoLanding();
    m_mavlink.StopSetpointStream();
    m_streamStarted.store(false, std::memory_order_relaxed);
    SetManualControlNeutral();
    SendManualControlSnapshot();
    DisableRemoteControl(true);
    if (!m_mavlink.Arm(false)) {
        if (err)
            *err = "px4 disarm rejected";
        return false;
    }
    return true;
}

bool Px4UdpHooks::EmergencyStop(std::string *err)
{
    CancelAutoLanding();
    m_mavlink.StopSetpointStream();
    m_streamStarted.store(false, std::memory_order_relaxed);
    DisableRemoteControl(true);
    if (!m_mavlink.EmergencyStop()) {
        if (err)
            *err = "px4 emergency stop rejected";
        return false;
    }
    return true;
}

bool Px4UdpHooks::SetOffboard(std::string *err)
{
    CancelAutoLanding();

    // OFFBOARD relies on setpoint stream; disable manual joystick streaming.
    m_manualControlStreaming.store(false, std::memory_order_relaxed);
    SetManualControlNeutral();
    SendManualControlSnapshot();

    Px4MavlinkGateway::LocalPositionNed local{};
    if (!m_mavlink.GetLocalPositionNed(local, 500000)) {
        if (err) {
            *err = "missing local position for offboard setpoint init";
        }
        return false;
    }

    if (!EnsureSetpointStream()) {
        if (err) {
            *err = "setpoint stream start failed";
        }
        return false;
    }

    // Initialize OFFBOARD with a hold-position setpoint around current pose.
    m_mavlink.UpdateStreamPosition(local.x, local.y, local.z, NAN);

    m_remoteModeRequested.store(true, std::memory_order_relaxed);
    m_offboardModeRequested.store(true, std::memory_order_relaxed);
    if (!EnsureOffboardMode(true, err)) {
        m_mavlink.StopSetpointStream();
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

bool Px4UdpHooks::Hold(std::string *err)
{
    (void)err;
    CancelAutoLanding();
    m_mavlink.StopSetpointStream();
    m_streamStarted.store(false, std::memory_order_relaxed);
    m_offboardModeRequested.store(false, std::memory_order_relaxed);
    EnsureManualControlStream();
    SetManualControlNeutral();
    SendManualControlSnapshot();
    return true;
}

bool Px4UdpHooks::Position(std::string *err)
{
    CancelAutoLanding();

    // POSITION mode should not keep OFFBOARD setpoint stream alive.
    m_mavlink.StopSetpointStream();
    m_streamStarted.store(false, std::memory_order_relaxed);

    EnsureManualControlStream();
    SetManualControlNeutral();
    SendManualControlSnapshot();

    Px4MavlinkGateway::FlightModeInfo beforeMode{};
    const bool haveBeforeMode = m_mavlink.GetFlightModeInfo(beforeMode);
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

    Px4MavlinkGateway::FlightModeInfo afterMode{};
    if (m_mavlink.GetFlightModeInfo(afterMode)) {
        std::cout << "[px4] POSITION result mode main=" << static_cast<int>(afterMode.mainMode)
                  << " sub=" << static_cast<int>(afterMode.subMode) << " armed=" << (afterMode.armed ? 1 : 0)
                  << "\n";
    }
    return true;
}

bool Px4UdpHooks::Land(std::string *err)
{
    CancelAutoLanding();
    m_mavlink.StopSetpointStream();
    m_streamStarted.store(false, std::memory_order_relaxed);
    SetManualControlNeutral();
    SendManualControlSnapshot();
    DisableRemoteControl(true);
    if (!m_mavlink.SendLand()) {
        if (err) {
            *err = "px4 land command rejected";
        }
        return false;
    }
    std::cout << "[land] MAV_CMD_NAV_LAND sent to PX4\n";
    return true;
}

bool Px4UdpHooks::SetMoveGoal(const MoveGoal &goal, std::string *err)
{
    CancelAutoLanding();

    if (goal.isRcJoystick) {
        Px4MavlinkGateway::ManualControlInput input{};
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

    // Non-RC move is OFFBOARD-only.
    if (!m_offboardModeRequested.load(std::memory_order_relaxed)) {
        m_mavlink.StopSetpointStream();
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

    Px4MavlinkGateway::SetpointLocalNED setpoint{};
    if (goal.isVelocity) {
        setpoint.x = NAN;
        setpoint.y = NAN;
        setpoint.z = NAN;
        setpoint.vx = goal.vx;
        setpoint.vy = goal.vy;
        setpoint.vz = goal.vz;
        setpoint.yaw = NAN;
        setpoint.yawspeed = goal.yawRate;
    } else {
        setpoint.x = goal.x;
        setpoint.y = goal.y;
        setpoint.z = goal.z;
        setpoint.vx = NAN;
        setpoint.vy = NAN;
        setpoint.vz = NAN;
        setpoint.yaw = goal.yaw;
        setpoint.yawspeed = NAN;
    }

    m_mavlink.UpdateStreamSetpoint(setpoint);
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
        m_mavlink.StartSetpointStreamHz(20.0);
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
    m_manualControlInput = Px4MavlinkGateway::ManualControlInput{};
}

void Px4UdpHooks::SetManualControlInput(const Px4MavlinkGateway::ManualControlInput &input)
{
    std::lock_guard<std::mutex> lock(m_manualControlMtx);
    m_manualControlInput = input;
}

Px4MavlinkGateway::ManualControlInput Px4UdpHooks::GetManualControlSnapshot() const
{
    std::lock_guard<std::mutex> lock(m_manualControlMtx);
    return m_manualControlInput;
}

void Px4UdpHooks::SendManualControlSnapshot() { m_mavlink.SendManualControl(GetManualControlSnapshot()); }

bool Px4UdpHooks::EnsureFlightMode(uint8_t mainMode, bool force, std::string *err, const char *modeName)
{
    Px4MavlinkGateway::FlightModeInfo currentMode{};
    const bool haveCurrentMode = m_mavlink.GetFlightModeInfo(currentMode);
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

    const bool ok = (mainMode == Px4MavlinkGateway::PX4_CUSTOM_MAIN_MODE_POSCTL) ? m_mavlink.SetModePosition()
                                                                                   : m_mavlink.SetModeOffboard();
    if (!ok) {
        if (err) {
            *err = std::string("px4 ") + (modeName ? modeName : "mode") + " rejected";
        }
        return false;
    }

    std::cout << "[px4] remote mode -> " << (modeName ? modeName : "unknown") << "\n";
    return true;
}

bool Px4UdpHooks::EnsurePositionMode(bool force, std::string *err)
{
    return EnsureFlightMode(Px4MavlinkGateway::PX4_CUSTOM_MAIN_MODE_POSCTL, force, err, "POSCTL");
}

bool Px4UdpHooks::EnsureOffboardMode(bool force, std::string *err)
{
    return EnsureFlightMode(Px4MavlinkGateway::PX4_CUSTOM_MAIN_MODE_OFFBOARD, force, err, "OFFBOARD");
}

void Px4UdpHooks::UpdateAutoLanding()
{
    if (!IsAutoLandingActive()) {
        return;
    }

    Px4MavlinkGateway::ManualControlInput input{};
    input.throttleNorm = kAutoLandThrottleNorm;
    SetManualControlInput(input);

    Px4MavlinkGateway::DownwardDistanceSensor range{};
    if (!m_mavlink.GetDownwardDistanceSensor(range, kAutoLandRangeMaxAgeUs) || !std::isfinite(range.currentDistance)) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    bool shouldDisarm = false;
    {
        std::lock_guard<std::mutex> lock(m_autoLandingMtx);
        if (!m_autoLanding.active) {
            return;
        }

        if (!m_autoLanding.haveRangeWindow) {
            m_autoLanding.haveRangeWindow = true;
            m_autoLanding.rangeWindowMin = range.currentDistance;
            m_autoLanding.rangeWindowMax = range.currentDistance;
            m_autoLanding.rangeWindowStart = now;
            return;
        }

        m_autoLanding.rangeWindowMin = std::min(m_autoLanding.rangeWindowMin, range.currentDistance);
        m_autoLanding.rangeWindowMax = std::max(m_autoLanding.rangeWindowMax, range.currentDistance);

        if ((now - m_autoLanding.rangeWindowStart) < kAutoLandStableDuration) {
            return;
        }

        const bool rangeStable =
            (m_autoLanding.rangeWindowMax - m_autoLanding.rangeWindowMin) <= kAutoLandStableRangeDeltaM;
        const bool nearGround = range.currentDistance <= kAutoLandNearGroundM;
        if (rangeStable && nearGround &&
            (m_autoLanding.lastDisarmAttempt.time_since_epoch().count() == 0 ||
             (now - m_autoLanding.lastDisarmAttempt) >= kAutoLandDisarmRetry)) {
            m_autoLanding.lastDisarmAttempt = now;
            shouldDisarm = true;
        } else {
            m_autoLanding.rangeWindowMin = range.currentDistance;
            m_autoLanding.rangeWindowMax = range.currentDistance;
            m_autoLanding.rangeWindowStart = now;
        }
    }

    if (!shouldDisarm) {
        return;
    }

    if (!m_mavlink.Arm(false)) {
        return;
    }

    CancelAutoLanding();
    SetManualControlNeutral();
    SendManualControlSnapshot();
    DisableRemoteControl(true);
    std::cout << "[land] touchdown detected by range stability, disarmed\n";
}

void Px4UdpHooks::ManualControlLoop()
{
    while (!m_manualLoopStop.load(std::memory_order_relaxed)) {
        Px4MavlinkGateway::FlightModeInfo flightMode{};
        if (m_mavlink.GetFlightModeInfo(flightMode)) {
            m_livePose.SetVehicleFlightState(flightMode.armed, flightMode.mainMode, flightMode.subMode);
        }

        UpdateAutoLanding();

        if (m_manualControlStreaming.load(std::memory_order_relaxed)) {
            SendManualControlSnapshot();
        }

        if (m_remoteModeRequested.load(std::memory_order_relaxed)) {
            if (m_offboardModeRequested.load(std::memory_order_relaxed)) {
                EnsureOffboardMode(false, nullptr);
            } else {
                EnsurePositionMode(false, nullptr);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

} // namespace smartdrone::core::application
