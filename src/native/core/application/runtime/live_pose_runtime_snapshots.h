#pragma once

#include "core/application/runtime/obstacle_avoidance_policy.h"
#include "core/application/runtime/px4_udp_hooks.h"
#include "core/application/runtime/runtime_controller.h"
#include "core/application/runtime/udp_command_runtime_config.h"
#include "core/application/state/live_pose_state.h"
#include "core/ports/pose_publisher.h"

namespace SmartDrone::Core::Application {

RuntimeGateSnapshot BuildRuntimeGateSnapshot(
    const LivePoseState::Snapshot &input);
UdpRuntimeStateSnapshot BuildUdpRuntimeStateSnapshot(
    const LivePoseState::Snapshot &input);
AvoidanceSnapshot BuildAvoidanceSnapshot(const LivePoseState::Snapshot &input);
bool ReadRuntimeGateSnapshot(const LivePoseState &livePose,
                             RuntimeGateSnapshot &snapshot);
bool ReadUdpRuntimeStateSnapshot(const LivePoseState &livePose,
                                 UdpRuntimeStateSnapshot &snapshot);
bool ReadAvoidanceSnapshot(const LivePoseState &livePose,
                           AvoidanceSnapshot &snapshot);
ReadRuntimeGateFn MakeRuntimeGateReader(const LivePoseState &livePose);
ReadAvoidanceSnapshotFn MakeAvoidanceSnapshotReader(
    const LivePoseState &livePose);
UpdateCommandPeerFn MakeCommandPeerUpdater(LivePoseState &livePose);
PublishRuntimeModeFn MakeRuntimeModePublisher(LivePoseState &livePose);
PublishVehicleFlightStateFn MakeVehicleFlightStatePublisher(
    LivePoseState &livePose);
PublishAvoidanceTelemetryFn MakeAvoidanceTelemetryPublisher(
    LivePoseState &livePose);
PublishVisualLossFn MakeVisualLossPublisher(
    LivePoseState &livePose, SmartDrone::Core::Ports::IPosePublisher &publisher);
void PublishExternalPose(
    const SmartDrone::Core::Ports::PosePublishRequest &request,
    SmartDrone::Core::Ports::IPosePublisher &publisher,
    LivePoseState &livePose);
ReadRuntimeStateFn MakeUdpRuntimeStateReader(const LivePoseState &livePose);

} // namespace SmartDrone::Core::Application
