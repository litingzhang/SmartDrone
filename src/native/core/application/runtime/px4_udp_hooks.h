#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "System.h"
#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "common/tlv/mavlink_hooks.h"
#include "core/application/state/live_pose_state.h"

namespace smartdrone::core::application {

class Px4UdpHooks final : public MavlinkHooks {
public:
    Px4UdpHooks(Px4MavlinkGateway& mavlink, LivePoseState& livePose);
    ~Px4UdpHooks() override;

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

    static float ClampSignedUnit(float value);
    static bool IsTrackingPoseUsable(int trackingState);
    static bool IsOdomQualityUsable(OdomQualityMode quality);

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

}  // namespace smartdrone::core::application
