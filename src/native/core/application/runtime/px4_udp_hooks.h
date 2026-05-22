#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include "common/tlv/runtime_command_hooks.h"
#include "core/application/state/live_pose_types.h"
#include "core/ports/vehicle_control_port.h"

namespace SmartDrone::Core::Ports {
class IVehicleControlPort;
}

namespace SmartDrone::Core::Application {

struct RuntimeGateSnapshot {
    uint8_t runtimeMode{RUNTIME_MODE_IDLE};
    bool poseValid{false};
    uint8_t trackingState{0xFF};
    LivePoseQuality poseQuality{LivePoseQuality::Lost};
};

using ReadRuntimeGateFn = std::function<bool(RuntimeGateSnapshot &)>;
using PublishVehicleFlightStateFn = std::function<void(bool, uint8_t, uint8_t)>;

struct Px4UdpHooksConfig {
    SmartDrone::Core::Ports::IVehicleControlPort &vehicleControl;
    ReadRuntimeGateFn readRuntimeGate;
    PublishVehicleFlightStateFn publishVehicleFlightState;
};

class Px4UdpHooks final : public RuntimeCommandHook {
  public:
    explicit Px4UdpHooks(Px4UdpHooksConfig config);
    ~Px4UdpHooks() override;

    RuntimeCommandGate ReadCommandGate() const override;
    bool ArmVehicle(std::string *err) override;
    bool DisarmVehicle(std::string *err) override;
    bool StopVehicleImmediately(std::string *err) override;
    bool EnterGuidedControl(std::string *err) override;
    bool HoldVehicle(std::string *err) override;
    bool EnterPositionControl(std::string *err) override;
    bool LandVehicle(std::string *err) override;
    bool ApplyMoveGoal(const MoveGoal &goal, std::string *err) override;
    void StepManualControl();

  private:
    struct AutoLandingState {
        bool active{false};
        bool haveRangeWindow{false};
        float rangeWindowMin{NAN};
        float rangeWindowMax{NAN};
        std::chrono::steady_clock::time_point rangeWindowStart{};
        std::chrono::steady_clock::time_point lastDisarmAttempt{};
    };

    struct PendingCommandAck {
        bool active{false};
        Ports::VehicleCommandAckKind command{Ports::VehicleCommandAckKind::ArmDisarm};
        std::string label;
        std::chrono::steady_clock::time_point deadline{};
    };

    static constexpr float kAutoLandThrottleNorm = -0.6f;
    static constexpr float kAutoLandStableRangeDeltaM = 0.03f;
    static constexpr float kAutoLandNearGroundM = 0.35f;
    static constexpr uint64_t kAutoLandRangeMaxAgeUs = 300000ULL;
    static constexpr auto kAutoLandStableDuration = std::chrono::seconds(2);
    static constexpr auto kAutoLandDisarmRetry = std::chrono::milliseconds(500);

    static float ClampSignedUnit(float value);
    static bool IsTrackingPoseUsable(int trackingState);
    static bool IsPoseQualityUsable(LivePoseQuality quality);

    void CancelAutoLanding();
    bool IsAutoLandingActive() const;
    bool EnsureSetpointStream();
    void EnsureManualControlStream();
    void DisableRemoteControl(bool stopManualStream);
    void SetManualControlNeutral();
    void SetManualControlInput(const SmartDrone::Core::Ports::VehicleManualControl &input);
    SmartDrone::Core::Ports::VehicleManualControl GetManualControlSnapshot() const;
    void SendManualControlSnapshot();
    bool ApplyRcMoveGoal(const MoveGoal &goal);
    bool ApplyOffboardMoveGoal(const MoveGoal &goal, std::string *err);
    bool EnsureOffboardMoveReady(std::string *err);
    SmartDrone::Core::Ports::VehicleSetpointLocalNed BuildMoveSetpoint(const MoveGoal &goal) const;
    bool EnsureOffboardMode(bool force, std::string *err);
    bool EnsureFlightMode(uint8_t mainMode, bool force, std::string *err, const char *modeName);
    bool EnsurePositionMode(bool force, std::string *err);
    bool PrepareAutoLandingRangeWindow(const SmartDrone::Core::Ports::VehicleDownwardRange &range,
                                       std::chrono::steady_clock::time_point now,
                                       bool &shouldDisarm);
    void UpdateAutoLanding();
    void BeginAutoLandingDisarm();
    void TrackCommandAck(Ports::VehicleCommandAckKind command, const std::string &label);
    void StepCommandAck();
    void ClearCommandAckIfCurrent(Ports::VehicleCommandAckKind command, const std::string &label);

    SmartDrone::Core::Ports::IVehicleControlPort &m_vehicleControl;
    ReadRuntimeGateFn m_readRuntimeGate;
    PublishVehicleFlightStateFn m_publishVehicleFlightState;
    std::atomic<bool> m_streamStarted{false};
    std::atomic<bool> m_manualControlStreaming{false};
    std::atomic<bool> m_remoteModeRequested{false};
    std::atomic<bool> m_offboardModeRequested{false};
    mutable std::mutex m_manualControlMtx;
    SmartDrone::Core::Ports::VehicleManualControl m_manualControlInput{};
    mutable std::mutex m_remoteModeMtx;
    mutable std::mutex m_autoLandingMtx;
    AutoLandingState m_autoLanding{};
    mutable std::mutex m_commandAckMtx;
    PendingCommandAck m_pendingCommandAck{};
    bool m_modeChangeRequested{false};
    uint8_t m_requestedMainMode{0};
    std::chrono::steady_clock::time_point m_lastPositionModeRequest{};
};

} // namespace SmartDrone::Core::Application
