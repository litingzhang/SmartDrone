#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
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
    struct PendingCommandAck {
        Ports::VehicleCommandAckKind command{Ports::VehicleCommandAckKind::ArmDisarm};
        std::string label;
        std::chrono::steady_clock::time_point deadline{};
    };

    struct RemoteModeRequestState {
        uint8_t mainMode{0};
        std::chrono::steady_clock::time_point requestTime{};
    };

    static float ClampSignedUnit(float value);
    static bool IsTrackingPoseUsable(int trackingState);
    static bool IsPoseQualityUsable(LivePoseQuality quality);

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
    void ClearRemoteModeRequest();
    bool TryMarkFlightModeRequest(uint8_t mainMode, bool force, std::chrono::steady_clock::time_point now);
    bool EnsureOffboardMode(bool force, std::string *err);
    bool EnsureFlightMode(uint8_t mainMode, bool force, std::string *err, const char *modeName);
    bool EnsurePositionMode(bool force, std::string *err);
    void TrackCommandAck(Ports::VehicleCommandAckKind command, const std::string &label);
    void StepCommandAck();
    void ClearCommandAckIfCurrent(const std::shared_ptr<const PendingCommandAck> &pending);

    SmartDrone::Core::Ports::IVehicleControlPort &m_vehicleControl;
    ReadRuntimeGateFn m_readRuntimeGate;
    PublishVehicleFlightStateFn m_publishVehicleFlightState;
    std::atomic<bool> m_streamStarted{false};
    std::atomic<bool> m_manualControlStreaming{false};
    std::atomic<bool> m_remoteModeRequested{false};
    std::atomic<bool> m_offboardModeRequested{false};
    std::shared_ptr<const SmartDrone::Core::Ports::VehicleManualControl> m_manualControlInput;
    std::shared_ptr<const RemoteModeRequestState> m_remoteModeRequestState;
    std::shared_ptr<const PendingCommandAck> m_pendingCommandAck;
};

} // namespace SmartDrone::Core::Application
