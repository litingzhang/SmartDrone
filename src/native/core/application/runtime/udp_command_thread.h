#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "common/tlv/tlv_protocol.h"
#include "common/tlv/udp_server.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/px4_udp_hooks.h"
#include "core/application/runtime/runtime_controller.h"
#include "core/application/state/live_pose_state.h"

namespace smartdrone::core::application {

constexpr uint8_t kCmdPointCloud = 0xF2;
constexpr uint16_t kPointCloudHeaderLen = 4;
constexpr size_t kMaxTlvPayloadLen = 0xFFFFu;
constexpr size_t kPointCloudPointStrideBytes = 12u;
constexpr size_t kMaxPointCloudPointsPerFrame =
    (kMaxTlvPayloadLen - kPointCloudHeaderLen) / kPointCloudPointStrideBytes;

using BuildCapabilitiesPayloadFn = std::function<std::vector<uint8_t>()>;
using BuildConfigPayloadFn =
    std::function<std::vector<uint8_t>(const UnifiedConfig &, smartdrone::core::domain::RuntimeMode)>;
using PeerToIpStringFn = std::function<std::string(const UdpPeer &)>;

std::thread StartUdpCommandThread(int port, Px4UdpHooks &hooks, UnifiedRuntimeController &controller,
                                  LivePoseState &livePose, std::atomic<bool> &runningFlag,
                                  BuildCapabilitiesPayloadFn buildCapabilitiesPayload,
                                  BuildConfigPayloadFn buildConfigPayload, PeerToIpStringFn peerToIpString);

} // namespace smartdrone::core::application
