#include "common/tlv/goal_cache.h"

#include <gtest/gtest.h>

namespace {

MoveGoal MakeGoal(uint32_t seq, float base)
{
    MoveGoal goal{};
    goal.frame = 2;
    goal.isVelocity = true;
    goal.isRcJoystick = false;
    goal.x = base + 1.0f;
    goal.y = base + 2.0f;
    goal.z = base + 3.0f;
    goal.yaw = base + 4.0f;
    goal.vx = base + 5.0f;
    goal.vy = base + 6.0f;
    goal.vz = base + 7.0f;
    goal.yawRate = base + 8.0f;
    goal.throttleNorm = base + 9.0f;
    goal.yawNorm = base + 10.0f;
    goal.pitchNorm = base + 11.0f;
    goal.rollNorm = base + 12.0f;
    goal.maxV = base + 13.0f;
    goal.seq = seq;
    return goal;
}

void ExpectGoalEq(const MoveGoal &actual, const MoveGoal &expected)
{
    EXPECT_EQ(actual.frame, expected.frame);
    EXPECT_EQ(actual.isVelocity, expected.isVelocity);
    EXPECT_EQ(actual.isRcJoystick, expected.isRcJoystick);
    EXPECT_FLOAT_EQ(actual.x, expected.x);
    EXPECT_FLOAT_EQ(actual.y, expected.y);
    EXPECT_FLOAT_EQ(actual.z, expected.z);
    EXPECT_FLOAT_EQ(actual.yaw, expected.yaw);
    EXPECT_FLOAT_EQ(actual.vx, expected.vx);
    EXPECT_FLOAT_EQ(actual.vy, expected.vy);
    EXPECT_FLOAT_EQ(actual.vz, expected.vz);
    EXPECT_FLOAT_EQ(actual.yawRate, expected.yawRate);
    EXPECT_FLOAT_EQ(actual.throttleNorm, expected.throttleNorm);
    EXPECT_FLOAT_EQ(actual.yawNorm, expected.yawNorm);
    EXPECT_FLOAT_EQ(actual.pitchNorm, expected.pitchNorm);
    EXPECT_FLOAT_EQ(actual.rollNorm, expected.rollNorm);
    EXPECT_FLOAT_EQ(actual.maxV, expected.maxV);
    EXPECT_EQ(actual.seq, expected.seq);
}

TEST(GoalCacheTest, ReturnsFalseBeforeGoalSet)
{
    GoalCache cache;

    MoveGoal goal{};
    EXPECT_FALSE(cache.Get(&goal));
}

TEST(GoalCacheTest, ReturnsFalseForNullOutput)
{
    GoalCache cache;

    EXPECT_FALSE(cache.Get(nullptr));
}

TEST(GoalCacheTest, ReturnsStoredGoal)
{
    GoalCache cache;
    const MoveGoal expected = MakeGoal(7, 1.0f);

    cache.Set(expected);

    MoveGoal actual{};
    ASSERT_TRUE(cache.Get(&actual));
    ExpectGoalEq(actual, expected);
}

TEST(GoalCacheTest, ReturnsLatestGoalAfterOverwrite)
{
    GoalCache cache;
    const MoveGoal first = MakeGoal(7, 1.0f);
    const MoveGoal second = MakeGoal(8, -3.0f);

    cache.Set(first);
    cache.Set(second);

    MoveGoal actual{};
    ASSERT_TRUE(cache.Get(&actual));
    ExpectGoalEq(actual, second);
}

} // namespace
