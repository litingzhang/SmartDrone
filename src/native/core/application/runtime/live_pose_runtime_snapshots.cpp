#include "core/application/runtime/live_pose_runtime_snapshots.h"

namespace SmartDrone::Core::Application {
namespace {

LivePoseQuality ToLivePoseQuality(
    SmartDrone::Core::Ports::PoseQuality quality)
{
    if (quality == SmartDrone::Core::Ports::PoseQuality::Good) {
        return LivePoseQuality::Good;
    }
    if (quality == SmartDrone::Core::Ports::PoseQuality::Weak) {
        return LivePoseQuality::Weak;
    }
    return LivePoseQuality::Lost;
}

SmartDrone::Core::Ports::PosePublishRequest MakeLostPoseRequest(
    const LivePoseState::Snapshot &snapshot)
{
    SmartDrone::Core::Ports::PosePublishRequest request{};
    request.pose = {true, snapshot.x, snapshot.y, snapshot.z, snapshot.qw,
                    snapshot.qx, snapshot.qy, snapshot.qz};
    request.referenceFrame = snapshot.referenceFrame;
    request.resetCounter = static_cast<uint8_t>(snapshot.resetCounter);
    request.trackingState = snapshot.trackingState;
    request.quality = SmartDrone::Core::Ports::PoseQuality::Lost;
    return request;
}

void PublishLastPoseAsLost(
    LivePoseState &livePose,
    SmartDrone::Core::Ports::IPosePublisher &publisher)
{
    LivePoseState::Snapshot snapshot{};
    if (!livePose.ReadSnapshot(snapshot) || snapshot.poseUpdateUs == 0) {
        return;
    }
    publisher.PublishPose(MakeLostPoseRequest(snapshot));
}

} // namespace

RuntimeGateSnapshot BuildRuntimeGateSnapshot(
    const LivePoseState::Snapshot &input)
{
    RuntimeGateSnapshot output{};
    output.runtimeMode = input.runtimeMode;
    output.poseValid = input.poseValid;
    output.trackingState = input.trackingState;
    output.poseQuality = input.poseQuality;
    output.poseUpdateUs = input.poseUpdateUs;
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
    output.px4FlightStateValid = input.px4FlightStateValid;
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
    return [&livePose](bool valid, bool armed, uint8_t mainMode,
                       uint8_t subMode) {
        livePose.SetVehicleFlightState(valid, armed, mainMode, subMode);
    };
}

PublishAvoidanceTelemetryFn MakeAvoidanceTelemetryPublisher(
    LivePoseState &livePose)
{
    return [&livePose](const AvoidanceTelemetry &telemetry) {
        livePose.SetAvoidanceTelemetry(telemetry);
    };
}

PublishVisualLossFn MakeVisualLossPublisher(
    LivePoseState &livePose, SmartDrone::Core::Ports::IPosePublisher &publisher)
{
    return [&livePose, &publisher]() {
        PublishLastPoseAsLost(livePose, publisher);
    };
}

void PublishExternalPose(
    const SmartDrone::Core::Ports::PosePublishRequest &request,
    SmartDrone::Core::Ports::IPosePublisher &publisher,
    LivePoseState &livePose)
{
    publisher.PublishPose(request);
    LivePoseUpdate update{};
    update.runtimeMode = RUNTIME_MODE_IDLE;
    update.trackingState = static_cast<uint8_t>(request.trackingState);
    update.resetCounter = request.resetCounter;
    update.resetMapCount = request.resetMapCount;
    update.pose = {request.pose.x, request.pose.y, request.pose.z,
                   request.pose.qw, request.pose.qx, request.pose.qy,
                   request.pose.qz};
    update.referenceFrame = request.referenceFrame;
    update.quality = ToLivePoseQuality(request.quality);
    update.poseValid = request.pose.valid &&
                       request.quality != SmartDrone::Core::Ports::PoseQuality::Lost;
    livePose.UpdatePose(update);
}

ReadRuntimeStateFn MakeUdpRuntimeStateReader(const LivePoseState &livePose)
{
    return [&livePose](UdpRuntimeStateSnapshot &snapshot) {
        return ReadUdpRuntimeStateSnapshot(livePose, snapshot);
    };
}

} // namespace SmartDrone::Core::Application
