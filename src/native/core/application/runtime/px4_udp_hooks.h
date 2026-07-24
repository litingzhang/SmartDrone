#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "common/tlv/runtime_command_hooks.h"
#include "core/application/runtime/hover_algorithm_plugin.h"
#include "core/application/runtime/obstacle_avoidance_policy.h"
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
    uint64_t poseUpdateUs{0};
};

using ReadRuntimeGateFn = std::function<bool(RuntimeGateSnapshot &)>;
using PublishVisualLossFn = std::function<void()>;
using PublishVehicleFlightStateFn =
    std::function<void(bool, bool, uint8_t, uint8_t)>;
using PublishAvoidanceTelemetryFn = std::function<void(const AvoidanceTelemetry &)>;

struct Px4UdpHooksConfig {
    SmartDrone::Core::Ports::IVehicleControlPort &vehicleControl;
    LiveRuntimeTuning *tuning{nullptr};
    ReadRuntimeGateFn readRuntimeGate;
    ReadAvoidanceSnapshotFn readAvoidanceSnapshot;
    PublishVehicleFlightStateFn publishVehicleFlightState;
    PublishAvoidanceTelemetryFn publishAvoidanceTelemetry;
    IAvoidanceAlgorithmPlugin *avoidancePlugin{nullptr};
    IHoverAlgorithmPlugin *hoverPlugin{nullptr};
    bool requireVisualLocalization{true};
    uint32_t visualPoseMaxAgeMs{100};
    uint32_t visualLossLandTimeoutMs{500};
    PublishVisualLossFn publishVisualLoss;
    uint32_t offboardWarmupMs{1100};
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
    bool IsLandingConfirmed() const override;
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
    static uint32_t PointCloudAgeMs(uint64_t updateUs);

    bool EnsureSetpointStream();
    void StopSetpointStream();
    bool OffboardWarmupComplete() const;
    void EnsureManualControlStream();
    void DisableRemoteControl(bool stopManualStream);
    void SetManualControlNeutral();
    void SetManualControlInput(const SmartDrone::Core::Ports::VehicleManualControl &input);
    SmartDrone::Core::Ports::VehicleManualControl GetManualControlSnapshot() const;
    void SendManualControlSnapshot();
    bool ApplyRcMoveGoal(const MoveGoal &goal, std::string *err);
    bool ApplyRcAvoidanceHoldIfNeeded(const MoveGoal &goal, std::string *err);
    bool ApplyOffboardMoveGoal(const MoveGoal &goal, std::string *err);
    bool EnsureOffboardMoveReady(std::string *err);
    SmartDrone::Core::Ports::VehicleSetpointLocalNed BuildMoveSetpoint(const MoveGoal &goal) const;
    bool ApplyAvoidanceHoldIfNeeded(const MoveGoal &goal, std::string *err);
    void StoreActiveOffboardGoal(const MoveGoal &goal);
    std::shared_ptr<const MoveGoal> LoadActiveOffboardGoal() const;
    void ResetActiveOffboardGoal();
    void ClearActiveOffboardGoal();
    void StepOffboardAvoidance();
    bool AvoidanceEnabled() const;
    void PublishAvoidanceIdle();
    void PublishAvoidanceStatus(const AvoidanceDecision &decision,
                                const AvoidanceSnapshot &snapshot,
                                bool holding);
    void ClearRemoteModeRequest();
    bool TryMarkFlightModeRequest(uint8_t mainMode, bool force, std::chrono::steady_clock::time_point now);
    bool EnsureOffboardMode(bool force, std::string *err);
    bool EnsureFlightMode(uint8_t mainMode, bool force, std::string *err, const char *modeName);
    bool EnsurePositionMode(bool force, std::string *err);
    void TrackCommandAck(Ports::VehicleCommandAckKind command, const std::string &label);
    void StepCommandAck();
    void ClearCommandAckIfCurrent(const std::shared_ptr<const PendingCommandAck> &pending);
    bool ReadVisualLocalizationReady() const;
    bool ReadVisualSnapshot(RuntimeGateSnapshot &snapshot) const;
    bool IsVisualSnapshotFresh(const RuntimeGateSnapshot &snapshot,
                               uint64_t nowUs) const;
    void MaybePublishVisualLoss(uint64_t nowUs);
    void StepVisualSafety(const SmartDrone::Core::Ports::VehicleFlightMode &flightMode);
    void TriggerVisualFailsafeLand(
        uint64_t nowUs,
        const SmartDrone::Core::Ports::VehicleFlightMode &flightMode);
    void RetryVisualFailsafeLand(uint64_t nowUs, bool armed,
                                 bool landingConfirmed);
    void ResetVisualSafety();
    std::shared_ptr<const SmartDrone::Core::Ports::VehicleFlightMode>
    ResolveSafetyFlightMode(
        bool haveFlightMode,
        const SmartDrone::Core::Ports::VehicleFlightMode &flightMode);

    SmartDrone::Core::Ports::IVehicleControlPort &m_vehicleControl;
    ReadRuntimeGateFn m_readRuntimeGate;
    ReadAvoidanceSnapshotFn m_readAvoidanceSnapshot;
    LiveRuntimeTuning *m_tuning{nullptr};
    PublishVehicleFlightStateFn m_publishVehicleFlightState;
    PublishAvoidanceTelemetryFn m_publishAvoidanceTelemetry;
    PublishVisualLossFn m_publishVisualLoss;
    ObstacleAvoidancePolicy m_defaultAvoidancePlugin;
    Px4PositionHoverAlgorithmPlugin m_defaultHoverPlugin;
    IAvoidanceAlgorithmPlugin *m_avoidancePlugin{nullptr};
    IHoverAlgorithmPlugin *m_hoverPlugin{nullptr};
    std::atomic<uint32_t> m_avoidanceHoldCount{0};
    std::atomic<bool> m_streamStarted{false};
    std::atomic<uint64_t> m_setpointStreamStartUs{0};
    std::atomic<bool> m_manualControlStreaming{false};
    std::atomic<bool> m_remoteModeRequested{false};
    std::atomic<bool> m_offboardModeRequested{false};
    std::shared_ptr<const SmartDrone::Core::Ports::VehicleManualControl> m_manualControlInput;
    std::shared_ptr<const MoveGoal> m_activeOffboardGoal;
    std::shared_ptr<const RemoteModeRequestState> m_remoteModeRequestState;
    std::shared_ptr<const PendingCommandAck> m_pendingCommandAck;
    std::shared_ptr<const SmartDrone::Core::Ports::VehicleFlightMode>
        m_lastSafetyFlightMode;
    bool m_requireVisualLocalization{true};
    uint64_t m_visualPoseMaxAgeUs{100000};
    uint64_t m_visualLossLandTimeoutUs{500000};
    uint64_t m_offboardWarmupUs{1100000};
    std::atomic<uint64_t> m_visualLossStartUs{0};
    std::atomic<uint64_t> m_visualLandLastRequestUs{0};
    std::atomic<uint64_t> m_visualInvalidLastPublishUs{0};
    std::atomic<bool> m_visualFailsafeActive{false};
};

} // namespace SmartDrone::Core::Application
