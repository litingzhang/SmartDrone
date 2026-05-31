#include "core/application/runtime/obstacle_avoidance_config.h"
#include "core/application/runtime/obstacle_avoidance_policy.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "common/environment.h"
#include "common/time_utils.h"

namespace {

using SmartDrone::Core::Application::AvoidanceSnapshot;
using SmartDrone::Core::Application::ObstacleAvoidanceLookaheadForSpeed;
using SmartDrone::Core::Application::ObstacleAvoidanceMaxPointCloudAgeUs;
using SmartDrone::Core::Application::ObstacleAvoidancePolicy;
using SmartDrone::Core::Application::ReadObstacleAvoidanceConfig;

AvoidanceSnapshot MakeSnapshot(std::vector<float> cloud)
{
    AvoidanceSnapshot snapshot{};
    snapshot.poseValid = true;
    snapshot.pointCloudXyz =
        std::make_shared<const std::vector<float>>(std::move(cloud));
    snapshot.pointCloudSeq = 1;
    snapshot.pointCloudUpdateUs = MonoTimeUs();
    return snapshot;
}

MoveGoal MakePositionGoal(float x)
{
    MoveGoal goal{};
    goal.x = x;
    goal.y = 0.0f;
    goal.z = 0.0f;
    goal.maxV = 1.0f;
    return goal;
}

class ObstacleAvoidancePolicyTest : public testing::Test {
  protected:
    void SetUp() override
    {
        SmartDrone::Common::SetEnvVar("SMART_DRONE_AVOIDANCE_ENABLE", "1");
        SmartDrone::Common::SetEnvVar("SMART_DRONE_AVOIDANCE_RADIUS_M",
                                      "0.75");
        SmartDrone::Common::SetEnvVar("SMART_DRONE_AVOIDANCE_LOOKAHEAD_M",
                                      "2.0");
        SmartDrone::Common::UnsetEnvVar(
            "SMART_DRONE_AVOIDANCE_SPEED_LOOKAHEAD_S");
        SmartDrone::Common::UnsetEnvVar(
            "SMART_DRONE_AVOIDANCE_NEAR_FIELD_RADIUS_M");
        SmartDrone::Common::SetEnvVar(
            "SMART_DRONE_AVOIDANCE_MAX_POINT_AGE_MS", "600");
        SmartDrone::Common::UnsetEnvVar(
            "SMART_DRONE_AVOIDANCE_MIN_CLOUD_POINTS");
        SmartDrone::Common::UnsetEnvVar(
            "SMART_DRONE_AVOIDANCE_MIN_BLOCKING_POINTS");
        SmartDrone::Common::UnsetEnvVar(
            "SMART_DRONE_AVOIDANCE_HOLD_ON_STALE_CLOUD");
    }

    void TearDown() override
    {
        SmartDrone::Common::UnsetEnvVar("SMART_DRONE_AVOIDANCE_ENABLE");
        SmartDrone::Common::UnsetEnvVar("SMART_DRONE_AVOIDANCE_RADIUS_M");
        SmartDrone::Common::UnsetEnvVar("SMART_DRONE_AVOIDANCE_LOOKAHEAD_M");
        SmartDrone::Common::UnsetEnvVar(
            "SMART_DRONE_AVOIDANCE_SPEED_LOOKAHEAD_S");
        SmartDrone::Common::UnsetEnvVar(
            "SMART_DRONE_AVOIDANCE_NEAR_FIELD_RADIUS_M");
        SmartDrone::Common::UnsetEnvVar(
            "SMART_DRONE_AVOIDANCE_MAX_POINT_AGE_MS");
        SmartDrone::Common::UnsetEnvVar(
            "SMART_DRONE_AVOIDANCE_MIN_CLOUD_POINTS");
        SmartDrone::Common::UnsetEnvVar(
            "SMART_DRONE_AVOIDANCE_MIN_BLOCKING_POINTS");
        SmartDrone::Common::UnsetEnvVar(
            "SMART_DRONE_AVOIDANCE_HOLD_ON_STALE_CLOUD");
    }
};

TEST_F(ObstacleAvoidancePolicyTest, HoldsForObstacleAhead)
{
    const AvoidanceSnapshot snapshot = MakeSnapshot({1.0f, 0.1f, 0.0f});
    const auto decision =
        ObstacleAvoidancePolicy{}.EvaluateMoveGoal(MakePositionGoal(2.0f),
                                                   snapshot);

    EXPECT_TRUE(decision.shouldHold);
    EXPECT_NE(decision.reason.find("obstacle"), std::string::npos);
    EXPECT_EQ(decision.holdReason,
              SmartDrone::Core::Application::AvoidanceHoldReason::ObstacleAhead);
    EXPECT_NEAR(decision.nearestObstacleM, 1.0f, 0.001f);
}

TEST_F(ObstacleAvoidancePolicyTest, ReadsDefaultAvoidanceConfig)
{
    const auto config = ReadObstacleAvoidanceConfig();

    EXPECT_TRUE(config.enabled);
    EXPECT_FALSE(config.holdOnStaleCloud);
    EXPECT_FLOAT_EQ(config.radiusM, 0.75f);
    EXPECT_FLOAT_EQ(config.lookaheadM, 2.0f);
    EXPECT_FLOAT_EQ(config.speedLookaheadS, 0.0f);
    EXPECT_FLOAT_EQ(config.nearFieldRadiusM, 0.0f);
    EXPECT_EQ(config.maxPointCloudAgeMs, 600);
    EXPECT_EQ(config.minCloudPoints, 1);
    EXPECT_EQ(config.minBlockingPoints, 1);
    EXPECT_EQ(ObstacleAvoidanceMaxPointCloudAgeUs(config), 600000ULL);
}

TEST_F(ObstacleAvoidancePolicyTest, ClampsAvoidanceConfigEnvironment)
{
    SmartDrone::Common::SetEnvVar("SMART_DRONE_AVOIDANCE_ENABLE", "0");
    SmartDrone::Common::SetEnvVar(
        "SMART_DRONE_AVOIDANCE_HOLD_ON_STALE_CLOUD", "1");
    SmartDrone::Common::SetEnvVar("SMART_DRONE_AVOIDANCE_RADIUS_M", "-1");
    SmartDrone::Common::SetEnvVar("SMART_DRONE_AVOIDANCE_LOOKAHEAD_M", "99");
    SmartDrone::Common::SetEnvVar(
        "SMART_DRONE_AVOIDANCE_SPEED_LOOKAHEAD_S", "9");
    SmartDrone::Common::SetEnvVar(
        "SMART_DRONE_AVOIDANCE_NEAR_FIELD_RADIUS_M", "9");
    SmartDrone::Common::SetEnvVar(
        "SMART_DRONE_AVOIDANCE_MAX_POINT_AGE_MS", "12");
    SmartDrone::Common::SetEnvVar(
        "SMART_DRONE_AVOIDANCE_MIN_CLOUD_POINTS", "9999");
    SmartDrone::Common::SetEnvVar(
        "SMART_DRONE_AVOIDANCE_MIN_BLOCKING_POINTS", "99");

    const auto config = ReadObstacleAvoidanceConfig();

    EXPECT_FALSE(config.enabled);
    EXPECT_TRUE(config.holdOnStaleCloud);
    EXPECT_FLOAT_EQ(config.radiusM, 0.2f);
    EXPECT_FLOAT_EQ(config.lookaheadM, 8.0f);
    EXPECT_FLOAT_EQ(config.speedLookaheadS, 5.0f);
    EXPECT_FLOAT_EQ(config.nearFieldRadiusM, 3.0f);
    EXPECT_EQ(config.maxPointCloudAgeMs, 50);
    EXPECT_EQ(config.minCloudPoints, 5000);
    EXPECT_EQ(config.minBlockingPoints, 50);
}

TEST_F(ObstacleAvoidancePolicyTest, ComputesDynamicLookaheadFromSpeed)
{
    auto config = ReadObstacleAvoidanceConfig();
    config.lookaheadM = 1.5f;
    config.speedLookaheadS = 2.0f;

    EXPECT_FLOAT_EQ(ObstacleAvoidanceLookaheadForSpeed(config, -1.0f), 1.5f);
    EXPECT_FLOAT_EQ(ObstacleAvoidanceLookaheadForSpeed(config, 1.0f), 2.0f);
    EXPECT_FLOAT_EQ(ObstacleAvoidanceLookaheadForSpeed(config, 9.0f), 8.0f);
}

TEST_F(ObstacleAvoidancePolicyTest, AllowsObstacleOutsideSafetyRadius)
{
    const AvoidanceSnapshot snapshot = MakeSnapshot({1.0f, 1.0f, 0.0f});
    const auto decision =
        ObstacleAvoidancePolicy{}.EvaluateMoveGoal(MakePositionGoal(2.0f),
                                                   snapshot);

    EXPECT_FALSE(decision.shouldHold);
}

TEST_F(ObstacleAvoidancePolicyTest, AllowsStalePointCloud)
{
    AvoidanceSnapshot snapshot = MakeSnapshot({1.0f, 0.1f, 0.0f});
    snapshot.pointCloudUpdateUs = 1;
    const auto decision =
        ObstacleAvoidancePolicy{}.EvaluateMoveGoal(MakePositionGoal(2.0f),
                                                   snapshot);

    EXPECT_FALSE(decision.shouldHold);
}

TEST_F(ObstacleAvoidancePolicyTest, HoldsOnStalePointCloudWhenConfigured)
{
    SmartDrone::Common::SetEnvVar(
        "SMART_DRONE_AVOIDANCE_HOLD_ON_STALE_CLOUD", "1");
    AvoidanceSnapshot snapshot = MakeSnapshot({1.0f, 0.1f, 0.0f});
    snapshot.pointCloudUpdateUs = 1;
    const auto decision =
        ObstacleAvoidancePolicy{}.EvaluateMoveGoal(MakePositionGoal(2.0f),
                                                   snapshot);

    EXPECT_TRUE(decision.shouldHold);
    EXPECT_NE(decision.reason.find("stale"), std::string::npos);
    EXPECT_EQ(decision.holdReason,
              SmartDrone::Core::Application::AvoidanceHoldReason::PointCloudStale);
}

TEST_F(ObstacleAvoidancePolicyTest, HoldsOnSparsePointCloudWhenConfigured)
{
    SmartDrone::Common::SetEnvVar(
        "SMART_DRONE_AVOIDANCE_HOLD_ON_STALE_CLOUD", "1");
    SmartDrone::Common::SetEnvVar(
        "SMART_DRONE_AVOIDANCE_MIN_CLOUD_POINTS", "2");
    const auto decision =
        ObstacleAvoidancePolicy{}.EvaluateMoveGoal(
            MakePositionGoal(2.0f), MakeSnapshot({1.0f, 0.1f, 0.0f}));

    EXPECT_TRUE(decision.shouldHold);
    EXPECT_NE(decision.reason.find("sparse"), std::string::npos);
    EXPECT_EQ(decision.holdReason,
              SmartDrone::Core::Application::AvoidanceHoldReason::PointCloudSparse);
}

TEST_F(ObstacleAvoidancePolicyTest, RequiresConfiguredBlockingPointCount)
{
    SmartDrone::Common::SetEnvVar(
        "SMART_DRONE_AVOIDANCE_MIN_BLOCKING_POINTS", "2");
    const auto onePointDecision =
        ObstacleAvoidancePolicy{}.EvaluateMoveGoal(
            MakePositionGoal(2.0f), MakeSnapshot({1.0f, 0.1f, 0.0f}));
    const auto twoPointDecision =
        ObstacleAvoidancePolicy{}.EvaluateMoveGoal(
            MakePositionGoal(2.0f),
            MakeSnapshot({1.0f, 0.1f, 0.0f, 1.2f, 0.2f, 0.0f}));

    EXPECT_FALSE(onePointDecision.shouldHold);
    EXPECT_TRUE(twoPointDecision.shouldHold);
}

TEST_F(ObstacleAvoidancePolicyTest, CountsOccupiedVoxelsForBlockingThreshold)
{
    SmartDrone::Common::SetEnvVar(
        "SMART_DRONE_AVOIDANCE_MIN_BLOCKING_POINTS", "2");
    const auto duplicateVoxelDecision =
        ObstacleAvoidancePolicy{}.EvaluateMoveGoal(
            MakePositionGoal(2.0f),
            MakeSnapshot({1.0f, 0.1f, 0.0f, 1.02f, 0.12f, 0.0f}));
    const auto twoVoxelDecision =
        ObstacleAvoidancePolicy{}.EvaluateMoveGoal(
            MakePositionGoal(2.0f),
            MakeSnapshot({1.0f, 0.1f, 0.0f, 1.4f, 0.1f, 0.0f}));

    EXPECT_FALSE(duplicateVoxelDecision.shouldHold);
    EXPECT_TRUE(twoVoxelDecision.shouldHold);
}

TEST_F(ObstacleAvoidancePolicyTest, ExtendsVelocityLookaheadBySpeed)
{
    SmartDrone::Common::SetEnvVar(
        "SMART_DRONE_AVOIDANCE_SPEED_LOOKAHEAD_S", "2.0");
    MoveGoal goal{};
    goal.isVelocity = true;
    goal.vx = 2.0f;
    const auto decision =
        ObstacleAvoidancePolicy{}.EvaluateMoveGoal(
            goal, MakeSnapshot({3.5f, 0.1f, 0.0f}));

    EXPECT_TRUE(decision.shouldHold);
    EXPECT_NEAR(decision.nearestObstacleM, 3.5f, 0.001f);
}

TEST_F(ObstacleAvoidancePolicyTest, HoldsForRcJoystickForwardObstacle)
{
    MoveGoal goal{};
    goal.isRcJoystick = true;
    goal.pitchNorm = 1.0f;
    goal.maxV = 1.0f;
    const auto decision =
        ObstacleAvoidancePolicy{}.EvaluateMoveGoal(
            goal, MakeSnapshot({1.0f, 0.1f, 0.0f}));

    EXPECT_TRUE(decision.shouldHold);
    EXPECT_EQ(decision.holdReason,
              SmartDrone::Core::Application::AvoidanceHoldReason::ObstacleAhead);
    EXPECT_NEAR(decision.nearestObstacleM, 1.0f, 0.001f);
}

TEST_F(ObstacleAvoidancePolicyTest, RotatesRcJoystickProbeByPoseYaw)
{
    MoveGoal goal{};
    goal.isRcJoystick = true;
    goal.pitchNorm = 1.0f;
    goal.maxV = 1.0f;
    AvoidanceSnapshot snapshot = MakeSnapshot({0.0f, 1.0f, 0.0f});
    snapshot.qw = 0.70710677f;
    snapshot.qz = 0.70710677f;

    const auto decision =
        ObstacleAvoidancePolicy{}.EvaluateMoveGoal(goal, snapshot);

    EXPECT_TRUE(decision.shouldHold);
    EXPECT_NEAR(decision.nearestObstacleM, 1.0f, 0.001f);
}

TEST_F(ObstacleAvoidancePolicyTest, HoldsForRcJoystickClimbObstacle)
{
    MoveGoal goal{};
    goal.isRcJoystick = true;
    goal.throttleNorm = 1.0f;
    goal.maxV = 1.0f;

    const auto decision =
        ObstacleAvoidancePolicy{}.EvaluateMoveGoal(
            goal, MakeSnapshot({0.0f, 0.0f, -1.0f}));

    EXPECT_TRUE(decision.shouldHold);
    EXPECT_EQ(decision.holdReason,
              SmartDrone::Core::Application::AvoidanceHoldReason::ObstacleAhead);
    EXPECT_NEAR(decision.nearestObstacleM, 1.0f, 0.001f);
}

TEST_F(ObstacleAvoidancePolicyTest, HoldsForNearFieldObstacleWhenConfigured)
{
    SmartDrone::Common::SetEnvVar(
        "SMART_DRONE_AVOIDANCE_NEAR_FIELD_RADIUS_M", "0.5");
    MoveGoal goal{};
    goal.isVelocity = true;
    const auto decision =
        ObstacleAvoidancePolicy{}.EvaluateMoveGoal(
            goal, MakeSnapshot({0.3f, 0.1f, 0.0f}));

    EXPECT_TRUE(decision.shouldHold);
    EXPECT_EQ(decision.holdReason,
              SmartDrone::Core::Application::AvoidanceHoldReason::ObstacleNear);
    EXPECT_NE(decision.reason.find("near"), std::string::npos);
    EXPECT_NEAR(decision.nearestObstacleM, 0.316f, 0.01f);
}

} // namespace
