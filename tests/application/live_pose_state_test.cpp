#include "core/application/state/live_pose_state.h"

#include <arpa/inet.h>

#include <gtest/gtest.h>

namespace {

using SmartDrone::Core::Application::LivePoseQuality;
using SmartDrone::Core::Application::LivePoseState;
using SmartDrone::Core::Application::LivePoseUpdate;

UdpPeer MakePeer(uint16_t port)
{
    UdpPeer peer{};
    peer.addr.sin_family = AF_INET;
    peer.addr.sin_port = htons(port);
    peer.addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    peer.valid = true;
    return peer;
}

LivePoseUpdate MakePoseUpdate()
{
    LivePoseUpdate update{};
    update.runtimeMode = RUNTIME_MODE_SLAM;
    update.trackingState = 2;
    update.resetCounter = 3;
    update.resetMapCount = 4;
    update.pose.x = 1.0f;
    update.pose.y = 2.0f;
    update.pose.z = 3.0f;
    update.pose.qw = 0.9f;
    update.pose.qx = 0.1f;
    update.pose.qy = 0.2f;
    update.pose.qz = 0.3f;
    update.quality = LivePoseQuality::Good;
    update.poseValid = true;
    return update;
}

TEST(LivePoseStateTest, ReadSnapshotRequiresValidPeer)
{
    LivePoseState state;
    LivePoseState::Snapshot snapshot{};

    EXPECT_FALSE(state.ReadSnapshot(snapshot));
    state.UpdatePeer(UdpPeer{});

    EXPECT_FALSE(state.ReadSnapshot(snapshot));
}

TEST(LivePoseStateTest, PeerUpdateDoesNotMakeSnapshotDirty)
{
    LivePoseState state;
    LivePoseState::Snapshot snapshot{};

    state.UpdatePeer(MakePeer(14550));

    ASSERT_TRUE(state.ReadSnapshot(snapshot));
    EXPECT_TRUE(snapshot.hasPeer);
    EXPECT_TRUE(snapshot.peer.valid);
    EXPECT_EQ(snapshot.seq, 1U);
    EXPECT_FALSE(state.ConsumeSnapshot(snapshot));
}

TEST(LivePoseStateTest, ConsumesDirtyRuntimeModeOnce)
{
    LivePoseState state;
    state.UpdatePeer(MakePeer(14550));
    state.SetRuntimeMode(RUNTIME_MODE_CALIB);

    LivePoseState::Snapshot snapshot{};
    ASSERT_TRUE(state.ConsumeSnapshot(snapshot));
    EXPECT_EQ(snapshot.runtimeMode, RUNTIME_MODE_CALIB);
    EXPECT_EQ(snapshot.seq, 2U);
    EXPECT_FALSE(snapshot.poseValid);

    EXPECT_FALSE(state.ConsumeSnapshot(snapshot));
    ASSERT_TRUE(state.ReadSnapshot(snapshot));
    EXPECT_EQ(snapshot.seq, 2U);
}

TEST(LivePoseStateTest, ReadSnapshotDoesNotClearDirtyState)
{
    LivePoseState state;
    state.UpdatePeer(MakePeer(14550));
    state.SetVehicleFlightState(true, 3, 4);

    LivePoseState::Snapshot snapshot{};
    ASSERT_TRUE(state.ReadSnapshot(snapshot));
    EXPECT_TRUE(snapshot.armed);
    EXPECT_EQ(snapshot.px4MainMode, 3);
    EXPECT_EQ(snapshot.px4SubMode, 4);

    ASSERT_TRUE(state.ConsumeSnapshot(snapshot));
    EXPECT_TRUE(snapshot.armed);
    EXPECT_EQ(snapshot.seq, 2U);
}

TEST(LivePoseStateTest, PublishesPoseAndPointCloud)
{
    LivePoseState state;
    state.UpdatePeer(MakePeer(14550));
    state.UpdatePose(MakePoseUpdate());
    state.UpdatePointCloud({1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});

    LivePoseState::Snapshot snapshot{};
    ASSERT_TRUE(state.ConsumeSnapshot(snapshot));
    EXPECT_TRUE(snapshot.poseValid);
    EXPECT_EQ(snapshot.runtimeMode, RUNTIME_MODE_SLAM);
    EXPECT_EQ(snapshot.trackingState, 2);
    EXPECT_EQ(snapshot.poseQuality, LivePoseQuality::Good);
    EXPECT_FLOAT_EQ(snapshot.x, 1.0f);
    EXPECT_FLOAT_EQ(snapshot.qz, 0.3f);
    ASSERT_TRUE(snapshot.pointCloudXyz);
    EXPECT_EQ(snapshot.pointCloudXyz->size(), 6U);
    EXPECT_FLOAT_EQ((*snapshot.pointCloudXyz)[4], 5.0f);
    EXPECT_EQ(snapshot.pointCloudSeq, 1U);
}

TEST(LivePoseStateTest, LeavingSlamInvalidatesPose)
{
    LivePoseState state;
    state.UpdatePeer(MakePeer(14550));
    state.UpdatePose(MakePoseUpdate());
    state.SetSlamMode(RUNTIME_SLAM_MODE_RELOCALIZATION);
    state.SetRuntimeMode(RUNTIME_MODE_IDLE);

    LivePoseState::Snapshot snapshot{};
    ASSERT_TRUE(state.ConsumeSnapshot(snapshot));
    EXPECT_EQ(snapshot.runtimeMode, RUNTIME_MODE_IDLE);
    EXPECT_EQ(snapshot.slamMode, RUNTIME_SLAM_MODE_MAPPING);
    EXPECT_EQ(snapshot.trackingState, 0xFF);
    EXPECT_EQ(snapshot.poseQuality, LivePoseQuality::Lost);
    EXPECT_FALSE(snapshot.poseValid);
}

} // namespace
