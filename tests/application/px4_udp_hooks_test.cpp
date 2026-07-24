#include "core/application/runtime/px4_udp_hooks.h"

#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "common/environment.h"
#include "common/time_utils.h"
#include "core/ports/slam_tracking_state.h"
#include "core/ports/vehicle_control_port.h"

namespace {

using SmartDrone::Core::Application::AvoidanceHoldReason;
using SmartDrone::Core::Application::AvoidanceDecision;
using SmartDrone::Core::Application::AvoidanceSnapshot;
using SmartDrone::Core::Application::AvoidanceTelemetry;
using SmartDrone::Core::Application::HoverAlgorithmContext;
using SmartDrone::Core::Application::IAvoidanceAlgorithmPlugin;
using SmartDrone::Core::Application::IHoverAlgorithmPlugin;
using SmartDrone::Core::Application::LivePoseQuality;
using SmartDrone::Core::Application::Px4UdpHooks;
using SmartDrone::Core::Application::Px4UdpHooksConfig;
using SmartDrone::Core::Application::RuntimeGateSnapshot;
using SmartDrone::Core::Ports::IVehicleControlPort;
using SmartDrone::Core::Ports::VehicleCommandAckKind;
using SmartDrone::Core::Ports::VehicleDownwardRange;
using SmartDrone::Core::Ports::VehicleFlightMode;
using SmartDrone::Core::Ports::VehicleLocalPosition;
using SmartDrone::Core::Ports::VehicleManualControl;
using SmartDrone::Core::Ports::VehicleSetpointLocalNed;

class FakeVehicleControlPort final : public IVehicleControlPort {
  public:
    bool BeginArm(bool) override
    {
        return true;
    }

    bool BeginEmergencyStop() override
    {
        return true;
    }

    bool BeginLand() override
    {
        ++m_landCount;
        return true;
    }

    bool BeginSetModePosition() override
    {
        return true;
    }

    bool BeginSetModeOffboard() override
    {
        ++m_offboardModeCount;
        return true;
    }

    void StartSetpointStreamHz(double) override
    {
    }

    void StopSetpointStream() override
    {
    }

    void UpdateStreamPosition(float x, float y, float z, float yaw) override
    {
        m_lastSetpoint.x = x;
        m_lastSetpoint.y = y;
        m_lastSetpoint.z = z;
        m_lastSetpoint.yaw = yaw;
    }

    void UpdateStreamSetpoint(const VehicleSetpointLocalNed &setpoint) override
    {
        m_lastSetpoint = setpoint;
    }

    void SendManualControl(const VehicleManualControl &input) override
    {
        m_lastManual = input;
        ++m_manualCount;
    }

    bool GetLocalPositionNed(VehicleLocalPosition &out, uint64_t) const override
    {
        if (!m_haveLocalPosition) {
            return false;
        }
        out = m_localPosition;
        return true;
    }

    bool GetFlightModeInfo(VehicleFlightMode &out) const override
    {
        if (!m_haveFlightMode) {
            return false;
        }
        out = m_flightMode;
        return true;
    }

    bool IsLandingMode(const VehicleFlightMode &) const override
    {
        return m_landingModeConfirmed;
    }

    bool GetDownwardRange(VehicleDownwardRange &, uint64_t) const override
    {
        return false;
    }

    bool TryConsumeCommandAck(VehicleCommandAckKind, uint8_t &) override
    {
        return false;
    }

    uint8_t PositionModeId() const override
    {
        return 3;
    }

    uint8_t OffboardModeId() const override
    {
        return 6;
    }

    int ManualCount() const
    {
        return m_manualCount;
    }

    VehicleManualControl LastManual() const
    {
        return m_lastManual;
    }

    void SetLocalPositionAvailable(bool available)
    {
        m_haveLocalPosition = available;
    }

    void SetFlightMode(const VehicleFlightMode &flightMode)
    {
        m_flightMode = flightMode;
        m_haveFlightMode = true;
    }

    void SetFlightModeAvailable(bool available)
    {
        m_haveFlightMode = available;
    }

    void SetLandingModeConfirmed(bool confirmed)
    {
        m_landingModeConfirmed = confirmed;
    }

    int LandCount() const
    {
        return m_landCount;
    }

    int OffboardModeCount() const
    {
        return m_offboardModeCount;
    }

  private:
    VehicleManualControl m_lastManual{};
    VehicleSetpointLocalNed m_lastSetpoint{};
    VehicleLocalPosition m_localPosition{};
    VehicleFlightMode m_flightMode{};
    bool m_haveLocalPosition{true};
    bool m_haveFlightMode{true};
    bool m_landingModeConfirmed{false};
    int m_manualCount{0};
    int m_landCount{0};
    int m_offboardModeCount{0};
};

class HoldAvoidancePlugin final : public IAvoidanceAlgorithmPlugin {
  public:
    AvoidanceDecision EvaluateMoveGoal(
        const MoveGoal &, const AvoidanceSnapshot &) const override
    {
        ++m_evaluateCount;
        return {true, "custom avoidance hold", 1.0f,
                AvoidanceHoldReason::ObstacleNear};
    }

    int EvaluateCount() const
    {
        return m_evaluateCount;
    }

  private:
    mutable int m_evaluateCount{0};
};

class CustomHoverPlugin final : public IHoverAlgorithmPlugin {
  public:
    VehicleManualControl BuildManualHold(
        const HoverAlgorithmContext &context) const override
    {
        ++m_manualHoldCount;
        m_lastHoldReason = context.avoidanceDecision.holdReason;
        VehicleManualControl input{};
        input.throttleNorm = -0.2f;
        return input;
    }

    bool ApplyOffboardHold(IVehicleControlPort &,
                           const HoverAlgorithmContext &context,
                           std::string *) const override
    {
        ++m_offboardHoldCount;
        m_lastHoldReason = context.avoidanceDecision.holdReason;
        return true;
    }

    int ManualHoldCount() const
    {
        return m_manualHoldCount;
    }

    int OffboardHoldCount() const
    {
        return m_offboardHoldCount;
    }

    AvoidanceHoldReason LastHoldReason() const
    {
        return m_lastHoldReason;
    }

  private:
    mutable int m_manualHoldCount{0};
    mutable int m_offboardHoldCount{0};
    mutable AvoidanceHoldReason m_lastHoldReason{AvoidanceHoldReason::None};
};

AvoidanceSnapshot MakeSnapshot(std::vector<float> cloud)
{
    AvoidanceSnapshot snapshot{};
    snapshot.poseValid = true;
    snapshot.pointCloudXyz =
        std::make_shared<const std::vector<float>>(std::move(cloud));
    snapshot.pointCloudUpdateUs = MonoTimeUs();
    return snapshot;
}

MoveGoal MakeRcGoal()
{
    MoveGoal goal{};
    goal.isRcJoystick = true;
    goal.throttleNorm = 0.5f;
    goal.pitchNorm = 1.0f;
    goal.rollNorm = 0.25f;
    goal.maxV = 1.0f;
    return goal;
}

class Px4UdpHooksTest : public testing::Test {
  protected:
    void SetUp() override
    {
        SmartDrone::Common::SetEnvVar("SMART_DRONE_AVOIDANCE_ENABLE", "1");
        SmartDrone::Common::SetEnvVar("SMART_DRONE_AVOIDANCE_RADIUS_M",
                                      "0.75");
        SmartDrone::Common::SetEnvVar("SMART_DRONE_AVOIDANCE_LOOKAHEAD_M",
                                      "2.0");
        SmartDrone::Common::UnsetEnvVar(
            "SMART_DRONE_AVOIDANCE_HOLD_ON_STALE_CLOUD");
    }

    void TearDown() override
    {
        SmartDrone::Common::UnsetEnvVar("SMART_DRONE_AVOIDANCE_ENABLE");
        SmartDrone::Common::UnsetEnvVar("SMART_DRONE_AVOIDANCE_RADIUS_M");
        SmartDrone::Common::UnsetEnvVar("SMART_DRONE_AVOIDANCE_LOOKAHEAD_M");
        SmartDrone::Common::UnsetEnvVar(
            "SMART_DRONE_AVOIDANCE_HOLD_ON_STALE_CLOUD");
    }
};

TEST_F(Px4UdpHooksTest, RcJoystickNeutralizesWhenObstacleBlocks)
{
    FakeVehicleControlPort vehicle;
    AvoidanceTelemetry telemetry{};
    Px4UdpHooks hooks(Px4UdpHooksConfig{
        vehicle,
        nullptr,
        {},
        [](AvoidanceSnapshot &snapshot) {
            snapshot = MakeSnapshot({1.0f, 0.1f, 0.0f});
            return true;
        },
        {},
        [&telemetry](const AvoidanceTelemetry &value) {
            telemetry = value;
        }});

    std::string err;
    EXPECT_FALSE(hooks.ApplyMoveGoal(MakeRcGoal(), &err));

    EXPECT_NE(err.find("obstacle"), std::string::npos);
    EXPECT_EQ(vehicle.ManualCount(), 1);
    EXPECT_FLOAT_EQ(vehicle.LastManual().throttleNorm, 0.0f);
    EXPECT_FLOAT_EQ(vehicle.LastManual().pitchNorm, 0.0f);
    EXPECT_FLOAT_EQ(vehicle.LastManual().rollNorm, 0.0f);
    EXPECT_TRUE(telemetry.holding);
    EXPECT_EQ(telemetry.holdReason, AvoidanceHoldReason::ObstacleAhead);
}

TEST_F(Px4UdpHooksTest, RcJoystickPassesThroughWhenClear)
{
    FakeVehicleControlPort vehicle;
    AvoidanceTelemetry telemetry{};
    Px4UdpHooks hooks(Px4UdpHooksConfig{
        vehicle,
        nullptr,
        {},
        [](AvoidanceSnapshot &snapshot) {
            snapshot = MakeSnapshot({1.0f, 2.0f, 0.0f});
            return true;
        },
        {},
        [&telemetry](const AvoidanceTelemetry &value) {
            telemetry = value;
        }});

    std::string err;
    EXPECT_TRUE(hooks.ApplyMoveGoal(MakeRcGoal(), &err));

    EXPECT_TRUE(err.empty());
    EXPECT_EQ(vehicle.ManualCount(), 1);
    EXPECT_FLOAT_EQ(vehicle.LastManual().throttleNorm, 0.5f);
    EXPECT_FLOAT_EQ(vehicle.LastManual().pitchNorm, 1.0f);
    EXPECT_FLOAT_EQ(vehicle.LastManual().rollNorm, 0.25f);
    EXPECT_FALSE(telemetry.holding);
}

TEST_F(Px4UdpHooksTest, UsesInjectedPluginsForRcHold)
{
    FakeVehicleControlPort vehicle;
    HoldAvoidancePlugin avoidancePlugin;
    CustomHoverPlugin hoverPlugin;
    Px4UdpHooks hooks(Px4UdpHooksConfig{
        vehicle,
        nullptr,
        {},
        [](AvoidanceSnapshot &snapshot) {
            snapshot = MakeSnapshot(std::vector<float>{});
            return true;
        },
        {},
        {},
        &avoidancePlugin,
        &hoverPlugin});

    std::string err;
    EXPECT_FALSE(hooks.ApplyMoveGoal(MakeRcGoal(), &err));

    EXPECT_EQ(avoidancePlugin.EvaluateCount(), 1);
    EXPECT_EQ(hoverPlugin.ManualHoldCount(), 1);
    EXPECT_EQ(hoverPlugin.OffboardHoldCount(), 0);
    EXPECT_EQ(hoverPlugin.LastHoldReason(),
              AvoidanceHoldReason::ObstacleNear);
    EXPECT_FLOAT_EQ(vehicle.LastManual().throttleNorm, -0.2f);
    EXPECT_EQ(err, "custom avoidance hold");
}

TEST_F(Px4UdpHooksTest, UsesInjectedHoverPluginForOffboardHold)
{
    FakeVehicleControlPort vehicle;
    HoldAvoidancePlugin avoidancePlugin;
    CustomHoverPlugin hoverPlugin;
    Px4UdpHooks hooks(Px4UdpHooksConfig{
        vehicle,
        nullptr,
        {},
        [](AvoidanceSnapshot &snapshot) {
            snapshot = MakeSnapshot(std::vector<float>{});
            return true;
        },
        {},
        {},
        &avoidancePlugin,
        &hoverPlugin});

    std::string err;
    EXPECT_TRUE(hooks.EnterGuidedControl(&err));
    EXPECT_FALSE(hooks.ApplyMoveGoal(MoveGoal{}, &err));

    EXPECT_EQ(avoidancePlugin.EvaluateCount(), 1);
    EXPECT_EQ(hoverPlugin.OffboardHoldCount(), 2);
    EXPECT_EQ(hoverPlugin.LastHoldReason(),
              AvoidanceHoldReason::ObstacleNear);
    EXPECT_EQ(err, "custom avoidance hold");
}

TEST_F(Px4UdpHooksTest, ControlProfileUsesFreshPx4LocalPosition)
{
    FakeVehicleControlPort vehicle;
    Px4UdpHooksConfig config{vehicle};
    config.requireVisualLocalization = false;
    Px4UdpHooks hooks(std::move(config));

    RuntimeCommandGate gate = hooks.ReadCommandGate();
    EXPECT_TRUE(gate.localizationReady);
    EXPECT_TRUE(gate.guidedControlReady);

    vehicle.SetLocalPositionAvailable(false);
    gate = hooks.ReadCommandGate();
    EXPECT_FALSE(gate.localizationReady);
    EXPECT_FALSE(gate.guidedControlReady);
}

TEST_F(Px4UdpHooksTest, WarmsSetpointsBeforeRequestingOffboard)
{
    FakeVehicleControlPort vehicle;
    Px4UdpHooksConfig config{vehicle};
    config.offboardWarmupMs = 20;
    Px4UdpHooks hooks(std::move(config));

    ASSERT_TRUE(hooks.EnterGuidedControl(nullptr));
    hooks.StepManualControl();
    EXPECT_EQ(vehicle.OffboardModeCount(), 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    hooks.StepManualControl();
    EXPECT_EQ(vehicle.OffboardModeCount(), 1);
}

TEST_F(Px4UdpHooksTest, VisualLossTriggersControlledLand)
{
    FakeVehicleControlPort vehicle;
    bool visualReady = true;
    int invalidPublishCount = 0;
    Px4UdpHooksConfig config{vehicle};
    config.readRuntimeGate = [&visualReady](RuntimeGateSnapshot &snapshot) {
        snapshot.runtimeMode = RUNTIME_MODE_SLAM;
        snapshot.poseValid = visualReady;
        snapshot.trackingState = visualReady
                                     ? SmartDrone::Core::Ports::SLAM_TRACKING_OK
                                     : SmartDrone::Core::Ports::SLAM_TRACKING_LOST;
        snapshot.poseQuality = visualReady ? LivePoseQuality::Good
                                           : LivePoseQuality::Lost;
        snapshot.poseUpdateUs = MonoTimeUs();
        return true;
    };
    config.publishVisualLoss = [&invalidPublishCount]() {
        ++invalidPublishCount;
    };
    config.requireVisualLocalization = true;
    config.visualLossLandTimeoutMs = 100;
    Px4UdpHooks hooks(std::move(config));
    ASSERT_TRUE(hooks.EnterGuidedControl(nullptr));
    vehicle.SetFlightMode({6, 0, true});

    visualReady = false;
    hooks.StepManualControl();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    hooks.StepManualControl();
    hooks.StepManualControl();

    EXPECT_EQ(vehicle.LandCount(), 1);
    EXPECT_GT(invalidPublishCount, 0);
}

TEST_F(Px4UdpHooksTest, VisualFailsafeStopsRetryAfterLandingConfirmation)
{
    FakeVehicleControlPort vehicle;
    Px4UdpHooksConfig config{vehicle};
    config.readRuntimeGate = [](RuntimeGateSnapshot &snapshot) {
        snapshot.runtimeMode = RUNTIME_MODE_SLAM;
        snapshot.poseValid = false;
        snapshot.trackingState = SmartDrone::Core::Ports::SLAM_TRACKING_LOST;
        snapshot.poseQuality = LivePoseQuality::Lost;
        snapshot.poseUpdateUs = MonoTimeUs();
        return true;
    };
    config.visualLossLandTimeoutMs = 100;
    Px4UdpHooks hooks(std::move(config));
    ASSERT_TRUE(hooks.EnterGuidedControl(nullptr));
    vehicle.SetFlightMode({6, 0, true});

    hooks.StepManualControl();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    hooks.StepManualControl();
    ASSERT_EQ(vehicle.LandCount(), 1);

    vehicle.SetLandingModeConfirmed(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(1050));
    hooks.StepManualControl();
    EXPECT_EQ(vehicle.LandCount(), 1);

    vehicle.SetLandingModeConfirmed(false);
    hooks.StepManualControl();
    EXPECT_EQ(vehicle.LandCount(), 2);
}

TEST_F(Px4UdpHooksTest, StaleVisualPoseTriggersControlledLand)
{
    FakeVehicleControlPort vehicle;
    const uint64_t poseUpdateUs = MonoTimeUs();
    Px4UdpHooksConfig config{vehicle};
    config.readRuntimeGate = [poseUpdateUs](RuntimeGateSnapshot &snapshot) {
        snapshot.runtimeMode = RUNTIME_MODE_SLAM;
        snapshot.poseValid = true;
        snapshot.trackingState = SmartDrone::Core::Ports::SLAM_TRACKING_OK;
        snapshot.poseQuality = LivePoseQuality::Good;
        snapshot.poseUpdateUs = poseUpdateUs;
        return true;
    };
    config.visualLossLandTimeoutMs = 100;
    Px4UdpHooks hooks(std::move(config));
    ASSERT_TRUE(hooks.EnterGuidedControl(nullptr));
    vehicle.SetFlightMode({6, 0, true});

    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    hooks.StepManualControl();

    EXPECT_EQ(vehicle.LandCount(), 1);
}

TEST_F(Px4UdpHooksTest, VisualFreshnessWindowCanScaleForSlowSimulation)
{
    FakeVehicleControlPort vehicle;
    Px4UdpHooksConfig config{vehicle};
    config.readRuntimeGate = [](RuntimeGateSnapshot &snapshot) {
        snapshot.runtimeMode = RUNTIME_MODE_SLAM;
        snapshot.poseValid = true;
        snapshot.trackingState =
            SmartDrone::Core::Ports::SLAM_TRACKING_OK;
        snapshot.poseQuality = LivePoseQuality::Good;
        snapshot.poseUpdateUs = MonoTimeUs() - 150000ULL;
        return true;
    };
    config.visualPoseMaxAgeMs = 200;
    Px4UdpHooks hooks(std::move(config));

    const RuntimeCommandGate gate = hooks.ReadCommandGate();

    EXPECT_TRUE(gate.localizationReady);
    EXPECT_TRUE(gate.guidedControlReady);
}

TEST_F(Px4UdpHooksTest, UpdatedUsablePosePreventsContinuousLossFailsafe)
{
    FakeVehicleControlPort vehicle;
    Px4UdpHooksConfig config{vehicle};
    config.readRuntimeGate = [](RuntimeGateSnapshot &snapshot) {
        snapshot.runtimeMode = RUNTIME_MODE_SLAM;
        snapshot.poseValid = true;
        snapshot.trackingState = SmartDrone::Core::Ports::SLAM_TRACKING_OK;
        snapshot.poseQuality = LivePoseQuality::Good;
        snapshot.poseUpdateUs = MonoTimeUs() - 120000ULL;
        return true;
    };
    config.visualLossLandTimeoutMs = 300;
    Px4UdpHooks hooks(std::move(config));
    ASSERT_TRUE(hooks.EnterGuidedControl(nullptr));
    vehicle.SetFlightMode({6, 0, true});

    hooks.StepManualControl();
    std::this_thread::sleep_for(std::chrono::milliseconds(160));
    hooks.StepManualControl();
    std::this_thread::sleep_for(std::chrono::milliseconds(160));
    hooks.StepManualControl();

    EXPECT_EQ(vehicle.LandCount(), 0);
}

TEST_F(Px4UdpHooksTest, CachedArmedOffboardModeProtectsDuringTelemetryGap)
{
    FakeVehicleControlPort vehicle;
    bool visualReady = true;
    Px4UdpHooksConfig config{vehicle};
    config.readRuntimeGate = [&visualReady](RuntimeGateSnapshot &snapshot) {
        snapshot.runtimeMode = RUNTIME_MODE_SLAM;
        snapshot.poseValid = visualReady;
        snapshot.trackingState = visualReady
                                     ? SmartDrone::Core::Ports::SLAM_TRACKING_OK
                                     : SmartDrone::Core::Ports::SLAM_TRACKING_LOST;
        snapshot.poseQuality = visualReady ? LivePoseQuality::Good
                                           : LivePoseQuality::Lost;
        snapshot.poseUpdateUs = MonoTimeUs();
        return true;
    };
    config.visualLossLandTimeoutMs = 100;
    Px4UdpHooks hooks(std::move(config));
    ASSERT_TRUE(hooks.EnterGuidedControl(nullptr));
    vehicle.SetFlightMode({6, 0, true});
    hooks.StepManualControl();

    vehicle.SetFlightModeAvailable(false);
    visualReady = false;
    hooks.StepManualControl();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    hooks.StepManualControl();

    EXPECT_EQ(vehicle.LandCount(), 1);
}

TEST_F(Px4UdpHooksTest, ActualArmedOffboardModeProtectsAfterRuntimeRestart)
{
    FakeVehicleControlPort vehicle;
    Px4UdpHooksConfig config{vehicle};
    config.readRuntimeGate = [](RuntimeGateSnapshot &snapshot) {
        snapshot.runtimeMode = RUNTIME_MODE_SLAM;
        snapshot.poseValid = false;
        snapshot.trackingState = SmartDrone::Core::Ports::SLAM_TRACKING_LOST;
        snapshot.poseQuality = LivePoseQuality::Lost;
        return true;
    };
    config.visualLossLandTimeoutMs = 100;
    Px4UdpHooks hooks(std::move(config));
    vehicle.SetFlightMode({6, 0, true});

    hooks.StepManualControl();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    hooks.StepManualControl();

    EXPECT_EQ(vehicle.LandCount(), 1);
}

TEST_F(Px4UdpHooksTest, CachedInactiveModeDoesNotLandDuringTelemetryGap)
{
    FakeVehicleControlPort vehicle;
    Px4UdpHooksConfig config{vehicle};
    config.readRuntimeGate = [](RuntimeGateSnapshot &snapshot) {
        snapshot.runtimeMode = RUNTIME_MODE_SLAM;
        snapshot.poseValid = false;
        snapshot.trackingState = SmartDrone::Core::Ports::SLAM_TRACKING_LOST;
        snapshot.poseQuality = LivePoseQuality::Lost;
        snapshot.poseUpdateUs = MonoTimeUs();
        return true;
    };
    config.visualLossLandTimeoutMs = 100;
    Px4UdpHooks hooks(std::move(config));
    ASSERT_TRUE(hooks.EnterGuidedControl(nullptr));

    vehicle.SetFlightMode({6, 0, false});
    hooks.StepManualControl();
    vehicle.SetFlightModeAvailable(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    hooks.StepManualControl();
    EXPECT_EQ(vehicle.LandCount(), 0);

    vehicle.SetFlightMode({3, 0, true});
    hooks.StepManualControl();
    vehicle.SetFlightModeAvailable(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    hooks.StepManualControl();
    EXPECT_EQ(vehicle.LandCount(), 0);
}

TEST_F(Px4UdpHooksTest, PublishesFlightStateValidityForEveryStep)
{
    FakeVehicleControlPort vehicle;
    vehicle.SetFlightMode({6, 4, true});
    std::vector<bool> validSamples;
    std::vector<VehicleFlightMode> samples;
    Px4UdpHooksConfig config{vehicle};
    config.publishVehicleFlightState =
        [&validSamples, &samples](bool valid, bool armed, uint8_t mainMode,
                                  uint8_t subMode) {
            validSamples.push_back(valid);
            samples.push_back({mainMode, subMode, armed});
        };
    Px4UdpHooks hooks(std::move(config));

    hooks.StepManualControl();
    vehicle.SetFlightModeAvailable(false);
    hooks.StepManualControl();

    ASSERT_EQ(validSamples.size(), 2U);
    EXPECT_TRUE(validSamples[0]);
    EXPECT_TRUE(samples[0].armed);
    EXPECT_EQ(samples[0].mainMode, 6);
    EXPECT_EQ(samples[0].subMode, 4);
    EXPECT_FALSE(validSamples[1]);
}

} // namespace
