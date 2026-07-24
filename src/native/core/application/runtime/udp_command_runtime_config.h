#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "common/tlv/runtime_command_hooks.h"
#include "common/tlv/tlv_protocol.h"
#include "common/tlv/udp_server.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/runtime_command_service.h"
#include "core/application/state/live_pose_types.h"
#include "core/domain/runtime_mode.h"

namespace SmartDrone::Core::Application {

constexpr uint8_t CMD_POINT_CLOUD = 0xF2;
constexpr uint8_t CMD_AVOIDANCE_STATE = 0xF6;
constexpr uint8_t CMD_HEARTBEAT_RUNTIME = CMD_HEARTBEAT;
constexpr uint16_t POINT_CLOUD_HEADER_LEN = 4;
constexpr uint16_t AVOIDANCE_STATE_PAYLOAD_LEN = 24;
constexpr size_t MAX_TLV_PAYLOAD_LEN = 0xFFFFu;
constexpr size_t POINT_CLOUD_POINT_STRIDE_BYTES = 12u;
constexpr size_t MAX_POINT_CLOUD_POINTS_PER_FRAME =
    (MAX_TLV_PAYLOAD_LEN - POINT_CLOUD_HEADER_LEN) /
    POINT_CLOUD_POINT_STRIDE_BYTES;

using BuildCapabilitiesPayloadFn = std::function<std::vector<uint8_t>()>;
using BuildConfigPayloadFn =
    std::function<std::vector<uint8_t>(
        const UnifiedConfig &, SmartDrone::Core::Domain::RuntimeMode)>;
using PeerToIpStringFn = std::function<std::string(const UdpPeer &)>;
using CurrentConfigFn = std::function<UnifiedConfig()>;
using CurrentRuntimeModeFn =
    std::function<SmartDrone::Core::Domain::RuntimeMode()>;
using UpdateCommandPeerFn = std::function<void(const UdpPeer &)>;

struct UdpRuntimeStateSnapshot {
    bool hasPeer{false};
    UdpPeer peer{};
    uint8_t runtimeMode{RUNTIME_MODE_IDLE};
    uint8_t slamMode{RUNTIME_SLAM_MODE_MAPPING};
    uint8_t trackingState{0xFF};
    bool px4FlightStateValid{false};
    bool armed{false};
    uint8_t px4MainMode{0};
    uint8_t px4SubMode{0};
    uint16_t resetCounter{0};
    uint16_t resetMapCount{0};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float qw{1.0f};
    float qx{0.0f};
    float qy{0.0f};
    float qz{0.0f};
    uint32_t seq{0};
    std::shared_ptr<const std::vector<float>> pointCloudXyz;
    uint32_t pointCloudSeq{0};
    uint64_t pointCloudUpdateUs{0};
    AvoidanceTelemetry avoidance{};
};

using ReadRuntimeStateFn = std::function<bool(UdpRuntimeStateSnapshot &)>;

struct UdpCommandRuntimeConfig {
    int port{0};
    uint32_t heartbeatTimeoutMs{3000};
    uint32_t heartbeatLandRetryMs{1000};
    RuntimeCommandHook *commandHook{nullptr};
    RuntimeCommandHook *hooks{nullptr};
    IRuntimeCommandTarget *commandTarget{nullptr};
    CurrentConfigFn currentConfig;
    CurrentRuntimeModeFn currentRuntimeMode;
    UpdateCommandPeerFn updateCommandPeer;
    ReadRuntimeStateFn readRuntimeState;
    BuildCapabilitiesPayloadFn buildCapabilitiesPayload;
    BuildConfigPayloadFn buildConfigPayload;
    PeerToIpStringFn peerToIpString;
};

bool UdpCommandRuntimeConfigValid(const UdpCommandRuntimeConfig &config);

} // namespace SmartDrone::Core::Application
