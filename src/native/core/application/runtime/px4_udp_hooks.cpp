#include "core/application/runtime/px4_udp_hooks.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <utility>

#include "common/time_utils.h"
#include "core/application/runtime/obstacle_avoidance_config.h"
#include "core/ports/slam_tracking_state.h"

namespace SmartDrone::Core::Application {

using SmartDrone::Core::Ports::VehicleCommandAckKind;
using SmartDrone::Core::Ports::VehicleFlightMode;
using SmartDrone::Core::Ports::VehicleManualControl;
using SmartDrone::Core::Ports::VehicleSetpointLocalNed;

namespace {

bool SameFlightMode(const VehicleFlightMode &left,
                    const VehicleFlightMode &right)
{
    return left.mainMode == right.mainMode && left.subMode == right.subMode &&
           left.armed == right.armed;
}

} // namespace

Px4UdpHooks::Px4UdpHooks(Px4UdpHooksConfig config)
    : m_vehicleControl(config.vehicleControl), m_readRuntimeGate(std::move(config.readRuntimeGate)),
      m_readAvoidanceSnapshot(std::move(config.readAvoidanceSnapshot)),
      m_tuning(config.tuning),
      m_publishVehicleFlightState(std::move(config.publishVehicleFlightState)),
      m_publishAvoidanceTelemetry(std::move(config.publishAvoidanceTelemetry)),
      m_publishVisualLoss(std::move(config.publishVisualLoss)),
      m_defaultAvoidancePlugin(config.tuning),
      m_defaultHoverPlugin(),
      m_avoidancePlugin(config.avoidancePlugin ? config.avoidancePlugin
                                               : &m_defaultAvoidancePlugin),
      m_hoverPlugin(config.hoverPlugin ? config.hoverPlugin
                                       : &m_defaultHoverPlugin),
      m_requireVisualLocalization(config.requireVisualLocalization),
      m_visualPoseMaxAgeUs(
          static_cast<uint64_t>(std::max<uint32_t>(
              50, config.visualPoseMaxAgeMs)) * 1000ULL),
      m_visualLossLandTimeoutUs(
          static_cast<uint64_t>(std::max<uint32_t>(
              100, config.visualLossLandTimeoutMs)) * 1000ULL),
      m_offboardWarmupUs(
          static_cast<uint64_t>(config.offboardWarmupMs) * 1000ULL)
{
}

Px4UdpHooks::~Px4UdpHooks()
{
}

float Px4UdpHooks::ClampSignedUnit(float value)
{
    return std::max(-1.0f, std::min(1.0f, value));
}

bool Px4UdpHooks::IsTrackingPoseUsable(int trackingState)
{
    return Ports::IsSlamTrackingPoseUsable(trackingState);
}

bool Px4UdpHooks::IsPoseQualityUsable(LivePoseQuality quality)
{
    return quality != LivePoseQuality::Lost;
}

uint32_t Px4UdpHooks::PointCloudAgeMs(uint64_t updateUs)
{
    if (updateUs == 0) {
        return 0;
    }
    const uint64_t nowUs = MonoTimeUs();
    if (updateUs >= nowUs) {
        return 0;
    }
    return static_cast<uint32_t>(
        std::min<uint64_t>(0xFFFFFFFFULL, (nowUs - updateUs) / 1000ULL));
}

RuntimeCommandGate Px4UdpHooks::ReadCommandGate() const
{
    if (!m_requireVisualLocalization) {
        Ports::VehicleLocalPosition localPosition{};
        const bool localPositionReady =
            m_vehicleControl.GetLocalPositionNed(localPosition, 500000);
        return {localPositionReady, localPositionReady};
    }
    const bool visualReady = ReadVisualLocalizationReady();
    return {visualReady, visualReady};
}

bool Px4UdpHooks::ReadVisualLocalizationReady() const
{
    RuntimeGateSnapshot snapshot{};
    return ReadVisualSnapshot(snapshot) &&
           IsVisualSnapshotFresh(snapshot, MonoTimeUs());
}

bool Px4UdpHooks::ReadVisualSnapshot(RuntimeGateSnapshot &snapshot) const
{
    return m_readRuntimeGate && m_readRuntimeGate(snapshot) &&
           snapshot.runtimeMode == RUNTIME_MODE_SLAM && snapshot.poseValid &&
           IsTrackingPoseUsable(snapshot.trackingState) &&
           IsPoseQualityUsable(snapshot.poseQuality);
}

bool Px4UdpHooks::IsVisualSnapshotFresh(
    const RuntimeGateSnapshot &snapshot, uint64_t nowUs) const
{
    return snapshot.poseUpdateUs != 0 && snapshot.poseUpdateUs <= nowUs &&
           nowUs - snapshot.poseUpdateUs <= m_visualPoseMaxAgeUs;
}

bool Px4UdpHooks::ArmVehicle(std::string *err)
{
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
    ResetVisualSafety();
    ClearActiveOffboardGoal();
    StopSetpointStream();
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
    ClearActiveOffboardGoal();
    StopSetpointStream();
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
    ResetVisualSafety();
    // OFFBOARD relies on setpoint stream; disable manual joystick streaming.
    ClearActiveOffboardGoal();
    m_manualControlStreaming.store(false, std::memory_order_relaxed);
    SetManualControlNeutral();
    SendManualControlSnapshot();

    HoverAlgorithmContext hoverContext{};
    std::string hoverErr;
    if (!m_hoverPlugin->ApplyOffboardHold(m_vehicleControl, hoverContext,
                                          &hoverErr)) {
        if (err) {
            *err = hoverErr.empty() ? "missing local position for offboard setpoint init"
                                    : hoverErr;
        }
        return false;
    }

    if (!EnsureSetpointStream()) {
        if (err) {
            *err = "setpoint stream start failed";
        }
        return false;
    }

    m_remoteModeRequested.store(true, std::memory_order_relaxed);
    m_offboardModeRequested.store(true, std::memory_order_relaxed);
    if (!EnsureOffboardMode(true, err)) {
        StopSetpointStream();
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
    ClearActiveOffboardGoal();
    StopSetpointStream();
    m_offboardModeRequested.store(false, std::memory_order_relaxed);
    EnsureManualControlStream();
    HoverAlgorithmContext hoverContext{};
    SetManualControlInput(m_hoverPlugin->BuildManualHold(hoverContext));
    SendManualControlSnapshot();
    return true;
}

bool Px4UdpHooks::EnterPositionControl(std::string *err)
{
    // POSITION mode should not keep OFFBOARD setpoint stream alive.
    ClearActiveOffboardGoal();
    StopSetpointStream();

    EnsureManualControlStream();
    SetManualControlNeutral();
    SendManualControlSnapshot();

    VehicleFlightMode beforeMode{};
    const bool haveBeforeMode = m_vehicleControl.GetFlightModeInfo(beforeMode);
    if (haveBeforeMode) {
        std::cerr << "[px4] POSITION requested from mode main=" << static_cast<int>(beforeMode.mainMode)
                  << " sub=" << static_cast<int>(beforeMode.subMode) << " armed=" << (beforeMode.armed ? 1 : 0)
                  << "\n";
    } else {
        std::cerr << "[px4] POSITION requested (no heartbeat mode snapshot)\n";
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
        std::cerr << "[px4] POSITION result mode main=" << static_cast<int>(afterMode.mainMode)
                  << " sub=" << static_cast<int>(afterMode.subMode) << " armed=" << (afterMode.armed ? 1 : 0)
                  << "\n";
    }
    return true;
}

bool Px4UdpHooks::LandVehicle(std::string *err)
{
    ResetVisualSafety();
    ClearActiveOffboardGoal();
    StopSetpointStream();
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
    std::cerr << "[land] land command sent to vehicle\n";
    return true;
}

bool Px4UdpHooks::IsLandingConfirmed() const
{
    VehicleFlightMode flightMode{};
    return m_vehicleControl.GetFlightModeInfo(flightMode) &&
           m_vehicleControl.IsLandingMode(flightMode);
}

bool Px4UdpHooks::ApplyMoveGoal(const MoveGoal &goal, std::string *err)
{
    if (goal.isRcJoystick) {
        return ApplyRcMoveGoal(goal, err);
    }
    return ApplyOffboardMoveGoal(goal, err);
}

bool Px4UdpHooks::ApplyRcMoveGoal(const MoveGoal &goal, std::string *err)
{
    ClearActiveOffboardGoal();
    if (ApplyRcAvoidanceHoldIfNeeded(goal, err)) {
        return false;
    }
    VehicleManualControl input{};
    input.throttleNorm = ClampSignedUnit(goal.throttleNorm);
    input.yawNorm = ClampSignedUnit(goal.yawNorm);
    input.pitchNorm = ClampSignedUnit(goal.pitchNorm);
    input.rollNorm = ClampSignedUnit(goal.rollNorm);
    EnsureManualControlStream();
    SetManualControlInput(input);
    SendManualControlSnapshot();
    return true;
}

bool Px4UdpHooks::ApplyRcAvoidanceHoldIfNeeded(const MoveGoal &goal,
                                              std::string *err)
{
    if (!m_readAvoidanceSnapshot) {
        PublishAvoidanceIdle();
        return false;
    }
    AvoidanceSnapshot snapshot{};
    if (!m_readAvoidanceSnapshot(snapshot)) {
        PublishAvoidanceIdle();
        return false;
    }
    const AvoidanceDecision decision =
        m_avoidancePlugin->EvaluateMoveGoal(goal, snapshot);
    if (!decision.shouldHold) {
        PublishAvoidanceStatus(decision, snapshot, false);
        return false;
    }
    EnsureManualControlStream();
    HoverAlgorithmContext hoverContext{};
    hoverContext.avoidanceSnapshot = snapshot;
    hoverContext.avoidanceDecision = decision;
    SetManualControlInput(m_hoverPlugin->BuildManualHold(hoverContext));
    SendManualControlSnapshot();
    PublishAvoidanceStatus(decision, snapshot, true);
    if (err) {
        *err = decision.reason;
    }
    std::cerr << "[avoid] " << decision.reason << "\n";
    return true;
}

bool Px4UdpHooks::EnsureOffboardMoveReady(std::string *err)
{
    if (!m_offboardModeRequested.load(std::memory_order_relaxed)) {
        StopSetpointStream();
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
    if (ApplyAvoidanceHoldIfNeeded(goal, err)) {
        ResetActiveOffboardGoal();
        return false;
    }
    const VehicleSetpointLocalNed setpoint = BuildMoveSetpoint(goal);
    m_vehicleControl.UpdateStreamSetpoint(setpoint);
    if (!EnsureOffboardMode(false, nullptr)) {
        ClearActiveOffboardGoal();
        if (err) {
            *err = "offboard mode not active";
        }
        return false;
    }
    StoreActiveOffboardGoal(goal);
    return true;
}

bool Px4UdpHooks::ApplyAvoidanceHoldIfNeeded(const MoveGoal &goal,
                                             std::string *err)
{
    if (!m_readAvoidanceSnapshot) {
        PublishAvoidanceIdle();
        return false;
    }
    AvoidanceSnapshot snapshot{};
    if (!m_readAvoidanceSnapshot(snapshot)) {
        PublishAvoidanceIdle();
        return false;
    }
    const AvoidanceDecision decision =
        m_avoidancePlugin->EvaluateMoveGoal(goal, snapshot);
    if (!decision.shouldHold) {
        PublishAvoidanceStatus(decision, snapshot, false);
        return false;
    }
    std::string holdErr;
    HoverAlgorithmContext hoverContext{};
    hoverContext.avoidanceSnapshot = snapshot;
    hoverContext.avoidanceDecision = decision;
    const bool held = m_hoverPlugin->ApplyOffboardHold(
        m_vehicleControl, hoverContext, &holdErr);
    PublishAvoidanceStatus(decision, snapshot, true);
    if (err) {
        *err = held || holdErr.empty() ? decision.reason
                                       : decision.reason + "; " + holdErr;
    }
    std::cerr << "[avoid] " << decision.reason << "\n";
    return true;
}

void Px4UdpHooks::StoreActiveOffboardGoal(const MoveGoal &goal)
{
    auto snapshot = std::make_shared<const MoveGoal>(goal);
    std::atomic_store_explicit(&m_activeOffboardGoal, std::move(snapshot),
                               std::memory_order_release);
}

std::shared_ptr<const MoveGoal> Px4UdpHooks::LoadActiveOffboardGoal() const
{
    return std::atomic_load_explicit(&m_activeOffboardGoal,
                                     std::memory_order_acquire);
}

void Px4UdpHooks::ClearActiveOffboardGoal()
{
    ResetActiveOffboardGoal();
    PublishAvoidanceIdle();
}

void Px4UdpHooks::ResetActiveOffboardGoal()
{
    std::atomic_store_explicit(&m_activeOffboardGoal,
                               std::shared_ptr<const MoveGoal>{},
                               std::memory_order_release);
}

void Px4UdpHooks::StepOffboardAvoidance()
{
    std::shared_ptr<const MoveGoal> goal = LoadActiveOffboardGoal();
    if (!goal) {
        return;
    }
    if (ApplyAvoidanceHoldIfNeeded(*goal, nullptr)) {
        ResetActiveOffboardGoal();
    }
}

void Px4UdpHooks::PublishAvoidanceIdle()
{
    if (!m_publishAvoidanceTelemetry) {
        return;
    }
    AvoidanceTelemetry telemetry{};
    telemetry.enabled = AvoidanceEnabled();
    telemetry.holdCount = m_avoidanceHoldCount.load(std::memory_order_relaxed);
    m_publishAvoidanceTelemetry(telemetry);
}

bool Px4UdpHooks::AvoidanceEnabled() const
{
    if (m_tuning) {
        return m_tuning->avoidanceEnabled.load(std::memory_order_relaxed);
    }
    return ReadObstacleAvoidanceConfig().enabled;
}

void Px4UdpHooks::PublishAvoidanceStatus(
    const AvoidanceDecision &decision,
    const AvoidanceSnapshot &snapshot,
    bool holding)
{
    if (!m_publishAvoidanceTelemetry) {
        return;
    }
    AvoidanceTelemetry telemetry{};
    telemetry.enabled = AvoidanceEnabled();
    telemetry.activeGoal = !holding;
    telemetry.holding = holding;
    telemetry.holdReason = holding ? decision.holdReason
                                   : AvoidanceHoldReason::None;
    telemetry.nearestObstacleM = decision.nearestObstacleM;
    telemetry.holdCount = holding
                              ? m_avoidanceHoldCount.fetch_add(
                                    1, std::memory_order_relaxed) + 1
                              : m_avoidanceHoldCount.load(
                                    std::memory_order_relaxed);
    telemetry.pointCloudAgeMs = PointCloudAgeMs(snapshot.pointCloudUpdateUs);
    m_publishAvoidanceTelemetry(telemetry);
}

bool Px4UdpHooks::EnsureSetpointStream()
{
    bool expected = false;
    if (m_streamStarted.compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
        m_setpointStreamStartUs.store(MonoTimeUs(), std::memory_order_relaxed);
        m_vehicleControl.StartSetpointStreamHz(20.0);
    }
    return true;
}

void Px4UdpHooks::StopSetpointStream()
{
    m_vehicleControl.StopSetpointStream();
    m_streamStarted.store(false, std::memory_order_relaxed);
    m_setpointStreamStartUs.store(0, std::memory_order_relaxed);
}

bool Px4UdpHooks::OffboardWarmupComplete() const
{
    const uint64_t startUs =
        m_setpointStreamStartUs.load(std::memory_order_relaxed);
    const uint64_t nowUs = MonoTimeUs();
    return startUs != 0 && nowUs >= startUs &&
           nowUs - startUs >= m_offboardWarmupUs;
}

void Px4UdpHooks::EnsureManualControlStream()
{
    m_manualControlStreaming.store(true, std::memory_order_relaxed);
}

void Px4UdpHooks::DisableRemoteControl(bool stopManualStream)
{
    ClearActiveOffboardGoal();
    m_remoteModeRequested.store(false, std::memory_order_relaxed);
    m_offboardModeRequested.store(false, std::memory_order_relaxed);
    if (stopManualStream) {
        m_manualControlStreaming.store(false, std::memory_order_relaxed);
    }
    ClearRemoteModeRequest();
}

void Px4UdpHooks::SetManualControlNeutral()
{
    SetManualControlInput(VehicleManualControl{});
}

void Px4UdpHooks::SetManualControlInput(const VehicleManualControl &input)
{
    auto snapshot = std::make_shared<const VehicleManualControl>(input);
    std::atomic_store_explicit(&m_manualControlInput, std::move(snapshot), std::memory_order_release);
}

VehicleManualControl Px4UdpHooks::GetManualControlSnapshot() const
{
    std::shared_ptr<const VehicleManualControl> snapshot =
        std::atomic_load_explicit(&m_manualControlInput, std::memory_order_acquire);
    if (!snapshot) {
        return VehicleManualControl{};
    }
    return *snapshot;
}

void Px4UdpHooks::SendManualControlSnapshot()
{
    m_vehicleControl.SendManualControl(GetManualControlSnapshot());
}

void Px4UdpHooks::ClearRemoteModeRequest()
{
    std::atomic_store_explicit(&m_remoteModeRequestState, std::shared_ptr<const RemoteModeRequestState>{},
                               std::memory_order_release);
}

bool Px4UdpHooks::TryMarkFlightModeRequest(uint8_t mainMode,
                                           bool force,
                                           std::chrono::steady_clock::time_point now)
{
    auto next = std::make_shared<const RemoteModeRequestState>(RemoteModeRequestState{mainMode, now});
    std::shared_ptr<const RemoteModeRequestState> current =
        std::atomic_load_explicit(&m_remoteModeRequestState, std::memory_order_acquire);
    while (true) {
        if (current) {
            const bool duplicateRecent =
                current->mainMode == mainMode && (now - current->requestTime) < std::chrono::milliseconds(600);
            if (!force && duplicateRecent) {
                return false;
            }
        }
        if (std::atomic_compare_exchange_weak_explicit(&m_remoteModeRequestState, &current, next,
                                                       std::memory_order_acq_rel,
                                                       std::memory_order_acquire)) {
            return true;
        }
    }
}

bool Px4UdpHooks::EnsureFlightMode(uint8_t mainMode, bool force, std::string *err, const char *modeName)
{
    VehicleFlightMode currentMode{};
    const bool haveCurrentMode = m_vehicleControl.GetFlightModeInfo(currentMode);
    if (haveCurrentMode && currentMode.mainMode == mainMode) {
        return true;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!TryMarkFlightModeRequest(mainMode, force, now)) {
        return true;
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
    std::cerr << "[px4] remote mode -> " << (modeName ? modeName : "unknown") << "\n";
    return true;
}

bool Px4UdpHooks::EnsurePositionMode(bool force, std::string *err)
{
    return EnsureFlightMode(m_vehicleControl.PositionModeId(), force, err, "POSCTL");
}

bool Px4UdpHooks::EnsureOffboardMode(bool force, std::string *err)
{
    if (!OffboardWarmupComplete()) {
        return true;
    }
    return EnsureFlightMode(m_vehicleControl.OffboardModeId(), force, err, "OFFBOARD");
}

void Px4UdpHooks::TrackCommandAck(VehicleCommandAckKind command, const std::string &label)
{
    PendingCommandAck pending{};
    pending.command = command;
    pending.label = label;
    pending.deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(900);
    auto snapshot = std::make_shared<const PendingCommandAck>(std::move(pending));
    std::atomic_store_explicit(&m_pendingCommandAck, std::move(snapshot), std::memory_order_release);
}

void Px4UdpHooks::StepCommandAck()
{
    std::shared_ptr<const PendingCommandAck> pending =
        std::atomic_load_explicit(&m_pendingCommandAck, std::memory_order_acquire);
    if (!pending) {
        return;
    }

    uint8_t result = 255;
    if (m_vehicleControl.TryConsumeCommandAck(pending->command, result)) {
        std::cerr << "[px4] " << pending->label << " ack=" << static_cast<int>(result) << "\n";
        ClearCommandAckIfCurrent(pending);
        return;
    }
    if (std::chrono::steady_clock::now() < pending->deadline) {
        return;
    }
    std::cerr << "[px4] " << pending->label << " ack timeout\n";
    ClearCommandAckIfCurrent(pending);
}

void Px4UdpHooks::ClearCommandAckIfCurrent(const std::shared_ptr<const PendingCommandAck> &pending)
{
    std::shared_ptr<const PendingCommandAck> expected = pending;
    std::atomic_compare_exchange_strong_explicit(&m_pendingCommandAck, &expected,
                                                 std::shared_ptr<const PendingCommandAck>{},
                                                 std::memory_order_acq_rel,
                                                 std::memory_order_acquire);
}

void Px4UdpHooks::ResetVisualSafety()
{
    m_visualLossStartUs.store(0, std::memory_order_relaxed);
    m_visualLandLastRequestUs.store(0, std::memory_order_relaxed);
    m_visualInvalidLastPublishUs.store(0, std::memory_order_relaxed);
    m_visualFailsafeActive.store(false, std::memory_order_relaxed);
}

void Px4UdpHooks::MaybePublishVisualLoss(uint64_t nowUs)
{
    constexpr uint64_t INVALID_PUBLISH_INTERVAL_US = 50000ULL;
    const uint64_t lastPublishUs =
        m_visualInvalidLastPublishUs.load(std::memory_order_relaxed);
    if (lastPublishUs != 0 &&
        nowUs - lastPublishUs < INVALID_PUBLISH_INTERVAL_US) {
        return;
    }
    m_visualInvalidLastPublishUs.store(nowUs, std::memory_order_relaxed);
    if (m_publishVisualLoss) {
        m_publishVisualLoss();
    }
}

void Px4UdpHooks::RetryVisualFailsafeLand(uint64_t nowUs, bool armed,
                                          bool landingConfirmed)
{
    if (!armed) {
        ResetVisualSafety();
        return;
    }
    if (landingConfirmed) {
        return;
    }
    const uint64_t lastRequestUs =
        m_visualLandLastRequestUs.load(std::memory_order_relaxed);
    if (lastRequestUs != 0 && nowUs - lastRequestUs < 1000000ULL) {
        return;
    }
    m_visualLandLastRequestUs.store(nowUs, std::memory_order_relaxed);
    if (m_vehicleControl.BeginLand()) {
        TrackCommandAck(VehicleCommandAckKind::Land, "visual failsafe land");
        return;
    }
    std::cerr << "[visual_failsafe] land command send failed; retrying\n";
}

void Px4UdpHooks::TriggerVisualFailsafeLand(
    uint64_t nowUs, const VehicleFlightMode &flightMode)
{
    ClearActiveOffboardGoal();
    StopSetpointStream();
    SetManualControlNeutral();
    SendManualControlSnapshot();
    DisableRemoteControl(true);
    m_visualFailsafeActive.store(true, std::memory_order_relaxed);
    std::cerr << "[visual_failsafe] localization lost for "
              << (m_visualLossLandTimeoutUs / 1000ULL)
              << "ms; controlled LAND requested\n";
    RetryVisualFailsafeLand(
        nowUs, flightMode.armed,
        m_vehicleControl.IsLandingMode(flightMode));
}

void Px4UdpHooks::StepVisualSafety(const VehicleFlightMode &flightMode)
{
    const uint64_t nowUs = MonoTimeUs();
    if (m_visualFailsafeActive.load(std::memory_order_relaxed)) {
        MaybePublishVisualLoss(nowUs);
        RetryVisualFailsafeLand(
            nowUs, flightMode.armed,
            m_vehicleControl.IsLandingMode(flightMode));
        return;
    }
    const bool monitoring = m_requireVisualLocalization && flightMode.armed &&
                            flightMode.mainMode ==
                                m_vehicleControl.OffboardModeId();
    RuntimeGateSnapshot snapshot{};
    const bool haveUsablePose = ReadVisualSnapshot(snapshot);
    if (!monitoring) {
        m_visualLossStartUs.store(0, std::memory_order_relaxed);
        return;
    }
    if (haveUsablePose && IsVisualSnapshotFresh(snapshot, nowUs)) {
        m_visualLossStartUs.store(0, std::memory_order_relaxed);
        return;
    }
    MaybePublishVisualLoss(nowUs);
    uint64_t lossStartUs = m_visualLossStartUs.load(std::memory_order_relaxed);
    if (haveUsablePose && snapshot.poseUpdateUs != 0 &&
        snapshot.poseUpdateUs <= nowUs) {
        lossStartUs = snapshot.poseUpdateUs;
        m_visualLossStartUs.store(lossStartUs, std::memory_order_relaxed);
    } else if (lossStartUs == 0) {
        lossStartUs = nowUs;
        m_visualLossStartUs.store(lossStartUs, std::memory_order_relaxed);
    }
    if (nowUs - lossStartUs >= m_visualLossLandTimeoutUs) {
        TriggerVisualFailsafeLand(nowUs, flightMode);
    }
}

std::shared_ptr<const VehicleFlightMode> Px4UdpHooks::ResolveSafetyFlightMode(
    bool haveFlightMode, const VehicleFlightMode &flightMode)
{
    std::shared_ptr<const VehicleFlightMode> cached =
        std::atomic_load_explicit(&m_lastSafetyFlightMode,
                                  std::memory_order_acquire);
    if (!haveFlightMode) {
        return cached;
    }
    if (cached && SameFlightMode(*cached, flightMode)) {
        return cached;
    }
    auto fresh = std::make_shared<const VehicleFlightMode>(flightMode);
    std::atomic_store_explicit(&m_lastSafetyFlightMode, fresh,
                               std::memory_order_release);
    return fresh;
}

void Px4UdpHooks::StepManualControl()
{
    StepCommandAck();
    VehicleFlightMode flightMode{};
    const bool haveFlightMode = m_vehicleControl.GetFlightModeInfo(flightMode);
    const std::shared_ptr<const VehicleFlightMode> safetyFlightMode =
        ResolveSafetyFlightMode(haveFlightMode, flightMode);
    if (safetyFlightMode) {
        StepVisualSafety(*safetyFlightMode);
    } else if (m_visualFailsafeActive.load(std::memory_order_relaxed)) {
        RetryVisualFailsafeLand(MonoTimeUs(), true, false);
    }
    if (m_publishVehicleFlightState) {
        m_publishVehicleFlightState(haveFlightMode, flightMode.armed,
                                    flightMode.mainMode, flightMode.subMode);
    }
    if (m_manualControlStreaming.load(std::memory_order_relaxed)) {
        SendManualControlSnapshot();
    }
    if (!m_remoteModeRequested.load(std::memory_order_relaxed)) {
        return;
    }
    if (m_offboardModeRequested.load(std::memory_order_relaxed)) {
        StepOffboardAvoidance();
        EnsureOffboardMode(false, nullptr);
        return;
    }
    EnsurePositionMode(false, nullptr);
}

} // namespace SmartDrone::Core::Application
