#include "core/application/runtime/live_pose_runtime_snapshots.h"

namespace SmartDrone::Core::Application {

RuntimeGateSnapshot BuildRuntimeGateSnapshot(
    const LivePoseState::Snapshot &input)
{
    RuntimeGateSnapshot output{};
    output.runtimeMode = input.runtimeMode;
    output.poseValid = input.poseValid;
    output.trackingState = input.trackingState;
    output.poseQuality = input.poseQuality;
    return output;
}

UdpRuntimeStateSnapshot BuildUdpRuntimeStateSnapshot(
    const LivePoseState::Snapshot &input)
{
    UdpRuntimeStateSnapshot output{};
    output.hasPeer = input.hasPeer;
    output.peer = input.peer;
    output.runtimeMode = input.runtimeMode;
    output.slamMode = input.slamMode;
    output.trackingState = input.trackingState;
    output.armed = input.armed;
    output.px4MainMode = input.px4MainMode;
    output.px4SubMode = input.px4SubMode;
    output.resetCounter = input.resetCounter;
    output.resetMapCount = input.resetMapCount;
    output.x = input.x;
    output.y = input.y;
    output.z = input.z;
    output.qw = input.qw;
    output.qx = input.qx;
    output.qy = input.qy;
    output.qz = input.qz;
    output.seq = input.seq;
    output.pointCloudXyz = input.pointCloudXyz;
    output.pointCloudSeq = input.pointCloudSeq;
    output.pointCloudUpdateUs = input.pointCloudUpdateUs;
    output.avoidance = input.avoidance;
    return output;
}

AvoidanceSnapshot BuildAvoidanceSnapshot(const LivePoseState::Snapshot &input)
{
    AvoidanceSnapshot output{};
    output.poseValid = input.poseValid;
    output.x = input.x;
    output.y = input.y;
    output.z = input.z;
    output.qw = input.qw;
    output.qx = input.qx;
    output.qy = input.qy;
    output.qz = input.qz;
    output.pointCloudXyz = input.pointCloudXyz;
    output.pointCloudSeq = input.pointCloudSeq;
    output.pointCloudUpdateUs = input.pointCloudUpdateUs;
    return output;
}

bool ReadRuntimeGateSnapshot(const LivePoseState &livePose,
                             RuntimeGateSnapshot &snapshot)
{
    LivePoseState::Snapshot liveSnapshot{};
    if (!livePose.ReadSnapshot(liveSnapshot)) {
        return false;
    }
    snapshot = BuildRuntimeGateSnapshot(liveSnapshot);
    return true;
}

bool ReadUdpRuntimeStateSnapshot(const LivePoseState &livePose,
                                 UdpRuntimeStateSnapshot &snapshot)
{
    LivePoseState::Snapshot liveSnapshot{};
    if (!livePose.ReadSnapshot(liveSnapshot)) {
        return false;
    }
    snapshot = BuildUdpRuntimeStateSnapshot(liveSnapshot);
    return true;
}

bool ReadAvoidanceSnapshot(const LivePoseState &livePose,
                           AvoidanceSnapshot &snapshot)
{
    LivePoseState::Snapshot liveSnapshot{};
    if (!livePose.ReadSnapshot(liveSnapshot)) {
        return false;
    }
    snapshot = BuildAvoidanceSnapshot(liveSnapshot);
    return true;
}

ReadRuntimeGateFn MakeRuntimeGateReader(const LivePoseState &livePose)
{
    return [&livePose](RuntimeGateSnapshot &snapshot) {
        return ReadRuntimeGateSnapshot(livePose, snapshot);
    };
}

ReadAvoidanceSnapshotFn MakeAvoidanceSnapshotReader(
    const LivePoseState &livePose)
{
    return [&livePose](AvoidanceSnapshot &snapshot) {
        return ReadAvoidanceSnapshot(livePose, snapshot);
    };
}

UpdateCommandPeerFn MakeCommandPeerUpdater(LivePoseState &livePose)
{
    return [&livePose](const UdpPeer &peer) {
        livePose.UpdatePeer(peer);
    };
}

PublishRuntimeModeFn MakeRuntimeModePublisher(LivePoseState &livePose)
{
    return [&livePose](Domain::RuntimeMode mode) {
        livePose.SetRuntimeMode(static_cast<uint8_t>(mode));
    };
}

PublishVehicleFlightStateFn MakeVehicleFlightStatePublisher(
    LivePoseState &livePose)
{
    return [&livePose](bool armed, uint8_t mainMode, uint8_t subMode) {
        livePose.SetVehicleFlightState(armed, mainMode, subMode);
    };
}

PublishAvoidanceTelemetryFn MakeAvoidanceTelemetryPublisher(
    LivePoseState &livePose)
{
    return [&livePose](const AvoidanceTelemetry &telemetry) {
        livePose.SetAvoidanceTelemetry(telemetry);
    };
}

ReadRuntimeStateFn MakeUdpRuntimeStateReader(const LivePoseState &livePose)
{
    return [&livePose](UdpRuntimeStateSnapshot &snapshot) {
        return ReadUdpRuntimeStateSnapshot(livePose, snapshot);
    };
}

} // namespace SmartDrone::Core::Application
