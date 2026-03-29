#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "System.h"
#include "adapters/telemetry/px4_mavlink_gateway.hpp"
#include "common/tlv/mavlink_hooks.hpp"
#include "core/application/live_pose_state.hpp"

namespace smartdrone::core::application {

class Px4UdpHooks final : public MavlinkHooks {
public:
    Px4UdpHooks(Px4MavlinkGateway& mavlink, LivePoseState& livePose) : m_mavlink(mavlink), m_livePose(livePose)
    {
        m_manualLoop = std::thread([this]() { ManualControlLoop(); });
    }

    ~Px4UdpHooks() override
    {
        m_manualLoopStop.store(true, std::memory_order_relaxed);
        if (m_manualLoop.joinable()) {
            m_manualLoop.join();
        }
    }

    VehicleGate GetGate() const override;
    bool Arm(std::string* err) override;
    bool Disarm(std::string* err) override;
    bool EmergencyStop(std::string* err) override;
    bool SetOffboard(std::string* err) override;
    bool Hold(std::string* err) override;
    bool Land(std::string* err) override;
    bool SetMoveGoal(const MoveGoal& goal, std::string* err) override;

private:
    enum class RemoteFlightMode : uint8_t {
        Altitude = Px4MavlinkGateway::PX4_CUSTOM_MAIN_MODE_ALTCTL,
        Position = Px4MavlinkGateway::PX4_CUSTOM_MAIN_MODE_POSCTL,
    };

    struct AutoLandingState {
        bool active{false};
        bool haveRangeWindow{false};
        float rangeWindowMin{NAN};
        float rangeWindowMax{NAN};
        std::chrono::steady_clock::time_point rangeWindowStart{};
        std::chrono::steady_clock::time_point lastDisarmAttempt{};
    };

    static constexpr float kAutoLandThrottleNorm = -0.6f;
    static constexpr float kAutoLandStableRangeDeltaM = 0.03f;
    static constexpr float kAutoLandNearGroundM = 0.35f;
    static constexpr uint64_t kAutoLandRangeMaxAgeUs = 300000ULL;
    static constexpr auto kAutoLandStableDuration = std::chrono::seconds(2);
    static constexpr auto kAutoLandDisarmRetry = std::chrono::milliseconds(500);

    static float ClampSignedUnit(float value)
    {
        return std::max(-1.0f, std::min(1.0f, value));
    }

    static bool IsTrackingPoseUsable(int trackingState)
    {
        return trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::OK_KLT;
    }

    static bool IsOdomQualityUsable(OdomQualityMode quality)
    {
        return quality != OdomQualityMode::LOST;
    }

    bool IsVioControlUsable() const;
    RemoteFlightMode DesiredRemoteFlightMode() const;
    static const char* RemoteFlightModeToString(RemoteFlightMode mode);
    void CancelAutoLanding();
    bool IsAutoLandingActive() const;
    bool EnsureSetpointStream();
    void EnsureManualControlStream();
    void DisableRemoteControl(bool stopManualStream);
    void SetManualControlNeutral();
    void SetManualControlInput(const Px4MavlinkGateway::ManualControlInput& input);
    Px4MavlinkGateway::ManualControlInput GetManualControlSnapshot() const;
    void SendManualControlSnapshot();
    void WarmupManualControlLink();
    void StartAutoLanding();
    bool MaybeSyncRemoteFlightMode(bool force, std::string* err);
    void UpdateAutoLanding();
    void ManualControlLoop();

    Px4MavlinkGateway& m_mavlink;
    LivePoseState& m_livePose;
    std::atomic<bool> m_streamStarted{false};
    std::atomic<bool> m_manualControlStreaming{false};
    std::atomic<bool> m_remoteModeRequested{false};
    std::atomic<bool> m_manualLoopStop{false};
    std::thread m_manualLoop;
    mutable std::mutex m_manualControlMtx;
    Px4MavlinkGateway::ManualControlInput m_manualControlInput{};
    mutable std::mutex m_remoteModeMtx;
    mutable std::mutex m_autoLandingMtx;
    AutoLandingState m_autoLanding{};
    std::optional<RemoteFlightMode> m_lastRequestedRemoteMode;
    std::chrono::steady_clock::time_point m_lastRemoteModeRequest{};
};

inline VehicleGate Px4UdpHooks::GetGate() const
{
    LivePoseState::Snapshot snapshot{};
    const bool hasSnapshot = m_livePose.ReadSnapshot(snapshot);
    const bool vioOk = hasSnapshot &&
                       snapshot.runtimeMode == RUNTIME_MODE_SLAM &&
                       snapshot.poseValid &&
                       IsTrackingPoseUsable(snapshot.trackingState) &&
                       IsOdomQualityUsable(snapshot.odomQuality);
    VehicleGate gate{};
    gate.vioOk = vioOk;
    gate.offboardReady = vioOk;
    return gate;
}

inline bool Px4UdpHooks::Arm(std::string* err)
{
    CancelAutoLanding();
    EnsureManualControlStream();
    SendManualControlSnapshot();
    if (!m_mavlink.Arm(true)) {
        if (err) *err = "px4 arm rejected";
        return false;
    }
    return true;
}

inline bool Px4UdpHooks::Disarm(std::string* err)
{
    CancelAutoLanding();
    SetManualControlNeutral();
    SendManualControlSnapshot();
    DisableRemoteControl(true);
    if (!m_mavlink.Arm(false)) {
        if (err) *err = "px4 disarm rejected";
        return false;
    }
    return true;
}

inline bool Px4UdpHooks::EmergencyStop(std::string* err)
{
    CancelAutoLanding();
    m_mavlink.StopSetpointStream();
    m_streamStarted.store(false, std::memory_order_relaxed);
    DisableRemoteControl(true);
    if (!m_mavlink.EmergencyStop()) {
        if (err) *err = "px4 emergency stop rejected";
        return false;
    }
    return true;
}

inline bool Px4UdpHooks::SetOffboard(std::string* err)
{
    CancelAutoLanding();
    EnsureManualControlStream();
    m_remoteModeRequested.store(true, std::memory_order_relaxed);
    WarmupManualControlLink();
    if (!MaybeSyncRemoteFlightMode(true, err)) {
        m_remoteModeRequested.store(false, std::memory_order_relaxed);
        if (err && err->empty()) *err = "px4 remote mode rejected";
        return false;
    }
    return true;
}

inline bool Px4UdpHooks::Hold(std::string* err)
{
    CancelAutoLanding();
    EnsureManualControlStream();
    SetManualControlNeutral();
    SendManualControlSnapshot();
    m_remoteModeRequested.store(true, std::memory_order_relaxed);
    if (!MaybeSyncRemoteFlightMode(true, err)) {
        m_remoteModeRequested.store(false, std::memory_order_relaxed);
        if (err && err->empty()) *err = "px4 hold mode rejected";
        return false;
    }
    return true;
}

inline bool Px4UdpHooks::Land(std::string* err)
{
    CancelAutoLanding();
    m_mavlink.StopSetpointStream();
    m_streamStarted.store(false, std::memory_order_relaxed);
    EnsureManualControlStream();
    m_remoteModeRequested.store(true, std::memory_order_relaxed);
    StartAutoLanding();
    WarmupManualControlLink();
    if (!MaybeSyncRemoteFlightMode(true, err)) {
        CancelAutoLanding();
        SetManualControlNeutral();
        DisableRemoteControl(true);
        if (err && err->empty()) *err = "px4 remote land mode rejected";
        return false;
    }
    SendManualControlSnapshot();
    std::cout << "[land] remote descent started\n";
    return true;
}

inline bool Px4UdpHooks::SetMoveGoal(const MoveGoal& goal, std::string* err)
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

    if (!EnsureSetpointStream()) {
        if (err) *err = "setpoint stream start failed";
        return false;
    }

    Px4MavlinkGateway::SetpointLocalNED setpoint{};
    if (goal.isVelocity) {
        setpoint.x = NAN; setpoint.y = NAN; setpoint.z = NAN;
        setpoint.vx = goal.vx; setpoint.vy = goal.vy; setpoint.vz = goal.vz;
        setpoint.yaw = NAN; setpoint.yawspeed = goal.yawRate;
    } else {
        setpoint.x = goal.x; setpoint.y = goal.y; setpoint.z = goal.z;
        setpoint.vx = NAN; setpoint.vy = NAN; setpoint.vz = NAN;
        setpoint.yaw = goal.yaw; setpoint.yawspeed = NAN;
    }
    m_mavlink.UpdateStreamSetpoint(setpoint);
    return true;
}

inline bool Px4UdpHooks::IsVioControlUsable() const
{
    LivePoseState::Snapshot snapshot{};
    return m_livePose.ReadSnapshot(snapshot) &&
           snapshot.runtimeMode == RUNTIME_MODE_SLAM &&
           snapshot.poseValid &&
           IsTrackingPoseUsable(snapshot.trackingState) &&
           IsOdomQualityUsable(snapshot.odomQuality);
}

inline Px4UdpHooks::RemoteFlightMode Px4UdpHooks::DesiredRemoteFlightMode() const
{
    return IsVioControlUsable() ? RemoteFlightMode::Position : RemoteFlightMode::Altitude;
}

inline const char* Px4UdpHooks::RemoteFlightModeToString(RemoteFlightMode mode)
{
    return mode == RemoteFlightMode::Position ? "POSCTL" : "ALTCTL";
}

inline void Px4UdpHooks::CancelAutoLanding()
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

inline bool Px4UdpHooks::IsAutoLandingActive() const
{
    std::lock_guard<std::mutex> lock(m_autoLandingMtx);
    return m_autoLanding.active;
}

inline bool Px4UdpHooks::EnsureSetpointStream()
{
    bool expected = false;
    if (m_streamStarted.compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
        m_mavlink.StartSetpointStreamHz(20.0);
    }
    return true;
}

inline void Px4UdpHooks::EnsureManualControlStream()
{
    m_manualControlStreaming.store(true, std::memory_order_relaxed);
}

inline void Px4UdpHooks::DisableRemoteControl(bool stopManualStream)
{
    m_remoteModeRequested.store(false, std::memory_order_relaxed);
    if (stopManualStream) {
        m_manualControlStreaming.store(false, std::memory_order_relaxed);
    }
    std::lock_guard<std::mutex> lock(m_remoteModeMtx);
    m_lastRequestedRemoteMode.reset();
    m_lastRemoteModeRequest = std::chrono::steady_clock::time_point{};
}

inline void Px4UdpHooks::SetManualControlNeutral()
{
    std::lock_guard<std::mutex> lock(m_manualControlMtx);
    m_manualControlInput = Px4MavlinkGateway::ManualControlInput{};
}

inline void Px4UdpHooks::SetManualControlInput(const Px4MavlinkGateway::ManualControlInput& input)
{
    std::lock_guard<std::mutex> lock(m_manualControlMtx);
    m_manualControlInput = input;
}

inline Px4MavlinkGateway::ManualControlInput Px4UdpHooks::GetManualControlSnapshot() const
{
    std::lock_guard<std::mutex> lock(m_manualControlMtx);
    return m_manualControlInput;
}

inline void Px4UdpHooks::SendManualControlSnapshot()
{
    m_mavlink.SendManualControl(GetManualControlSnapshot());
}

inline void Px4UdpHooks::WarmupManualControlLink()
{
    for (int i = 0; i < 3; ++i) {
        SendManualControlSnapshot();
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
}

inline void Px4UdpHooks::StartAutoLanding()
{
    AutoLandingState next{};
    next.active = true;
    {
        std::lock_guard<std::mutex> lock(m_autoLandingMtx);
        m_autoLanding = next;
    }
    Px4MavlinkGateway::ManualControlInput input{};
    input.throttleNorm = kAutoLandThrottleNorm;
    SetManualControlInput(input);
}

inline bool Px4UdpHooks::MaybeSyncRemoteFlightMode(bool force, std::string* err)
{
    const RemoteFlightMode desired = DesiredRemoteFlightMode();
    Px4MavlinkGateway::FlightModeInfo currentMode{};
    if (m_mavlink.GetFlightModeInfo(currentMode) &&
        currentMode.mainMode == static_cast<uint8_t>(desired)) {
        return true;
    }

    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(m_remoteModeMtx);
        if (!force &&
            m_lastRequestedRemoteMode &&
            *m_lastRequestedRemoteMode == desired &&
            (now - m_lastRemoteModeRequest) < std::chrono::milliseconds(600)) {
            return true;
        }
        m_lastRequestedRemoteMode = desired;
        m_lastRemoteModeRequest = now;
    }

    const bool ok = (desired == RemoteFlightMode::Position)
                        ? m_mavlink.SetModePosition()
                        : m_mavlink.SetModeAltitude();
    if (!ok) {
        if (err) {
            *err = std::string("px4 ") + RemoteFlightModeToString(desired) + " rejected";
        }
        return false;
    }

    std::cout << "[px4] remote manual mode -> " << RemoteFlightModeToString(desired) << "\n";
    return true;
}

inline void Px4UdpHooks::UpdateAutoLanding()
{
    if (!IsAutoLandingActive()) {
        return;
    }

    Px4MavlinkGateway::ManualControlInput input{};
    input.throttleNorm = kAutoLandThrottleNorm;
    SetManualControlInput(input);

    Px4MavlinkGateway::DownwardDistanceSensor range{};
    if (!m_mavlink.GetDownwardDistanceSensor(range, kAutoLandRangeMaxAgeUs) ||
        !std::isfinite(range.currentDistance)) {
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

inline void Px4UdpHooks::ManualControlLoop()
{
    while (!m_manualLoopStop.load(std::memory_order_relaxed)) {
        UpdateAutoLanding();

        if (m_manualControlStreaming.load(std::memory_order_relaxed)) {
            SendManualControlSnapshot();
        }

        if (m_remoteModeRequested.load(std::memory_order_relaxed)) {
            MaybeSyncRemoteFlightMode(false, nullptr);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

}  // namespace smartdrone::core::application
