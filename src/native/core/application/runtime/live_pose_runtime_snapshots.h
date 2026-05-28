#pragma once

#include "core/application/runtime/px4_udp_hooks.h"
#include "core/application/runtime/runtime_controller.h"
#include "core/application/runtime/udp_command_runtime_config.h"
#include "core/application/state/live_pose_state.h"

namespace SmartDrone::Core::Application {

RuntimeGateSnapshot BuildRuntimeGateSnapshot(
    const LivePoseState::Snapshot &input);
UdpRuntimeStateSnapshot BuildUdpRuntimeStateSnapshot(
    const LivePoseState::Snapshot &input);
bool ReadRuntimeGateSnapshot(const LivePoseState &livePose,
                             RuntimeGateSnapshot &snapshot);
bool ReadUdpRuntimeStateSnapshot(const LivePoseState &livePose,
                                 UdpRuntimeStateSnapshot &snapshot);
ReadRuntimeGateFn MakeRuntimeGateReader(const LivePoseState &livePose);
UpdateCommandPeerFn MakeCommandPeerUpdater(LivePoseState &livePose);
PublishRuntimeModeFn MakeRuntimeModePublisher(LivePoseState &livePose);
PublishVehicleFlightStateFn MakeVehicleFlightStatePublisher(
    LivePoseState &livePose);
ReadRuntimeStateFn MakeUdpRuntimeStateReader(const LivePoseState &livePose);

} // namespace SmartDrone::Core::Application
