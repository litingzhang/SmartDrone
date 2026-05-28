#include "core/application/runtime/live_pose_runtime_snapshots.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

namespace {

using SmartDrone::Core::Application::BuildRuntimeGateSnapshot;
using SmartDrone::Core::Application::BuildUdpRuntimeStateSnapshot;
using SmartDrone::Core::Application::LivePoseQuality;
using SmartDrone::Core::Application::LivePoseState;
using SmartDrone::Core::Application::MakeCommandPeerUpdater;
using SmartDrone::Core::Application::MakeRuntimeGateReader;
using SmartDrone::Core::Application::MakeRuntimeModePublisher;
using SmartDrone::Core::Application::MakeUdpRuntimeStateReader;
using SmartDrone::Core::Application::MakeVehicleFlightStatePublisher;

LivePoseState::Snapshot MakeLivePoseSnapshot()
{
    LivePoseState::Snapshot snapshot{};
    snapshot.hasPeer = true;
    snapshot.peer.valid = true;
    snapshot.poseValid = true;
    snapshot.runtimeMode = RUNTIME_MODE_SLAM;
    snapshot.slamMode = RUNTIME_SLAM_MODE_RELOCALIZATION;
    snapshot.trackingState = 2;
    snapshot.armed = true;
    snapshot.px4MainMode = 3;
    snapshot.px4SubMode = 4;
    snapshot.poseQuality = LivePoseQuality::Good;
    snapshot.resetCounter = 5;
    snapshot.resetMapCount = 6;
    snapshot.x = 1.0f;
    snapshot.y = 2.0f;
    snapshot.z = 3.0f;
    snapshot.qw = 0.9f;
    snapshot.qx = 0.1f;
    snapshot.qy = 0.2f;
    snapshot.qz = 0.3f;
    snapshot.seq = 7;
    snapshot.pointCloudXyz =
        std::make_shared<const std::vector<float>>(std::vector<float>{
            1.0f, 2.0f, 3.0f});
    snapshot.pointCloudSeq = 8;
    return snapshot;
}

TEST(LivePoseRuntimeSnapshotsTest, BuildsRuntimeGateSnapshot)
{
    const LivePoseState::Snapshot input = MakeLivePoseSnapshot();
    const auto output = BuildRuntimeGateSnapshot(input);

    EXPECT_EQ(output.runtimeMode, RUNTIME_MODE_SLAM);
    EXPECT_TRUE(output.poseValid);
    EXPECT_EQ(output.trackingState, 2);
    EXPECT_EQ(output.poseQuality, LivePoseQuality::Good);
}

TEST(LivePoseRuntimeSnapshotsTest, BuildsUdpRuntimeStateSnapshot)
{
    const LivePoseState::Snapshot input = MakeLivePoseSnapshot();
    const auto output = BuildUdpRuntimeStateSnapshot(input);

    EXPECT_TRUE(output.hasPeer);
    EXPECT_TRUE(output.peer.valid);
    EXPECT_EQ(output.runtimeMode, RUNTIME_MODE_SLAM);
    EXPECT_EQ(output.slamMode, RUNTIME_SLAM_MODE_RELOCALIZATION);
    EXPECT_EQ(output.trackingState, 2);
    EXPECT_TRUE(output.armed);
    EXPECT_EQ(output.px4MainMode, 3);
    EXPECT_EQ(output.px4SubMode, 4);
    EXPECT_EQ(output.resetCounter, 5);
    EXPECT_EQ(output.resetMapCount, 6);
    EXPECT_FLOAT_EQ(output.x, 1.0f);
    EXPECT_FLOAT_EQ(output.y, 2.0f);
    EXPECT_FLOAT_EQ(output.z, 3.0f);
    EXPECT_FLOAT_EQ(output.qw, 0.9f);
    EXPECT_FLOAT_EQ(output.qx, 0.1f);
    EXPECT_FLOAT_EQ(output.qy, 0.2f);
    EXPECT_FLOAT_EQ(output.qz, 0.3f);
    EXPECT_EQ(output.seq, 7U);
    ASSERT_TRUE(output.pointCloudXyz);
    EXPECT_EQ(output.pointCloudXyz->size(), 3U);
    EXPECT_EQ(output.pointCloudSeq, 8U);
}

TEST(LivePoseRuntimeSnapshotsTest, BuildsLivePoseCallbacks)
{
    LivePoseState livePose;
    auto updatePeer = MakeCommandPeerUpdater(livePose);
    updatePeer(MakeLivePoseSnapshot().peer);
    livePose.UpdatePose({});
    auto publishRuntimeMode = MakeRuntimeModePublisher(livePose);
    publishRuntimeMode(SmartDrone::Core::Domain::RuntimeMode::Slam);
    auto publishFlightState = MakeVehicleFlightStatePublisher(livePose);
    publishFlightState(true, 3, 4);

    auto readGate = MakeRuntimeGateReader(livePose);
    SmartDrone::Core::Application::RuntimeGateSnapshot gate{};
    ASSERT_TRUE(readGate(gate));
    EXPECT_EQ(gate.runtimeMode, RUNTIME_MODE_SLAM);

    auto readState = MakeUdpRuntimeStateReader(livePose);
    SmartDrone::Core::Application::UdpRuntimeStateSnapshot state{};
    ASSERT_TRUE(readState(state));
    EXPECT_TRUE(state.hasPeer);
    EXPECT_EQ(state.runtimeMode, RUNTIME_MODE_SLAM);
    EXPECT_TRUE(state.armed);
    EXPECT_EQ(state.px4MainMode, 3);
    EXPECT_EQ(state.px4SubMode, 4);
}

} // namespace
