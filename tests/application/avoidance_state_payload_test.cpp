#include "core/application/runtime/avoidance_state_payload.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "common/time_utils.h"
#include "common/tlv/tlv_pack.h"

namespace {

using SmartDrone::Core::Application::AVOIDANCE_STATE_PAYLOAD_LEN;
using SmartDrone::Core::Application::AvoidanceHoldReason;
using SmartDrone::Core::Application::BuildAvoidanceStatePayload;
using SmartDrone::Core::Application::UdpRuntimeStateSnapshot;

TEST(AvoidanceStatePayloadTest, PacksFixedLittleEndianPayload)
{
    UdpRuntimeStateSnapshot snapshot{};
    snapshot.avoidance.enabled = true;
    snapshot.avoidance.activeGoal = true;
    snapshot.avoidance.holding = true;
    snapshot.avoidance.holdReason = AvoidanceHoldReason::PointCloudSparse;
    snapshot.avoidance.nearestObstacleM = 1.25f;
    snapshot.avoidance.holdCount = 7;
    snapshot.avoidance.pointCloudAgeMs = 42;
    snapshot.pointCloudSeq = 99;
    snapshot.pointCloudXyz =
        std::make_shared<const std::vector<float>>(std::vector<float>{
            1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});

    const std::vector<uint8_t> payload = BuildAvoidanceStatePayload(snapshot);

    ASSERT_EQ(payload.size(), AVOIDANCE_STATE_PAYLOAD_LEN);
    EXPECT_EQ(payload[0], 1);
    EXPECT_EQ(payload[1], 1);
    EXPECT_EQ(payload[2], 1);
    EXPECT_EQ(payload[3],
              static_cast<uint8_t>(AvoidanceHoldReason::PointCloudSparse));
    EXPECT_FLOAT_EQ(ReadF32Le(&payload[4]), 1.25f);
    EXPECT_EQ(ReadU32Le(&payload[8]), 7U);
    EXPECT_EQ(ReadU32Le(&payload[12]), 42U);
    EXPECT_EQ(ReadU32Le(&payload[16]), 99U);
    EXPECT_EQ(ReadU32Le(&payload[20]), 2U);
}

TEST(AvoidanceStatePayloadTest, ComputesPointCloudAgeFromUpdateTimestamp)
{
    UdpRuntimeStateSnapshot snapshot{};
    snapshot.avoidance.pointCloudAgeMs = 1;
    snapshot.pointCloudSeq = 5;
    snapshot.pointCloudUpdateUs = MonoTimeUs() - 250000ULL;

    const std::vector<uint8_t> payload = BuildAvoidanceStatePayload(snapshot);

    ASSERT_EQ(payload.size(), AVOIDANCE_STATE_PAYLOAD_LEN);
    EXPECT_GE(ReadU32Le(&payload[12]), 200U);
    EXPECT_LE(ReadU32Le(&payload[12]), 1000U);
    EXPECT_EQ(ReadU32Le(&payload[16]), 5U);
}

TEST(AvoidanceStatePayloadTest, UsesTelemetryAgeWhenUpdateTimestampMissing)
{
    UdpRuntimeStateSnapshot snapshot{};
    snapshot.avoidance.pointCloudAgeMs = 42;

    const std::vector<uint8_t> payload = BuildAvoidanceStatePayload(snapshot);

    ASSERT_EQ(payload.size(), AVOIDANCE_STATE_PAYLOAD_LEN);
    EXPECT_EQ(ReadU32Le(&payload[12]), 42U);
}

TEST(AvoidanceStatePayloadTest, ClearsFlagsWhenTelemetryInactive)
{
    UdpRuntimeStateSnapshot snapshot{};

    const std::vector<uint8_t> payload = BuildAvoidanceStatePayload(snapshot);

    ASSERT_EQ(payload.size(), AVOIDANCE_STATE_PAYLOAD_LEN);
    EXPECT_EQ(payload[0], 0);
    EXPECT_EQ(payload[1], 0);
    EXPECT_EQ(payload[2], 0);
    EXPECT_EQ(payload[3], 0);
}

} // namespace
