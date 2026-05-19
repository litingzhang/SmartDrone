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
#include "core/domain/runtime_mode.h"

namespace smartdrone::core::application {

constexpr uint8_t kCmdPointCloud = 0xF2;
constexpr uint8_t kCmdHeartbeat = CMD_HEARTBEAT;
constexpr uint16_t kPointCloudHeaderLen = 4;
constexpr size_t kMaxTlvPayloadLen = 0xFFFFu;
constexpr size_t kPointCloudPointStrideBytes = 12u;
constexpr size_t kMaxPointCloudPointsPerFrame =
    (kMaxTlvPayloadLen - kPointCloudHeaderLen) / kPointCloudPointStrideBytes;

using BuildCapabilitiesPayloadFn = std::function<std::vector<uint8_t>()>;
using BuildConfigPayloadFn =
    std::function<std::vector<uint8_t>(const UnifiedConfig &, smartdrone::core::domain::RuntimeMode)>;
using PeerToIpStringFn = std::function<std::string(const UdpPeer &)>;
using CurrentConfigFn = std::function<UnifiedConfig()>;
using CurrentRuntimeModeFn = std::function<smartdrone::core::domain::RuntimeMode()>;
using UpdateCommandPeerFn = std::function<void(const UdpPeer &)>;

struct UdpRuntimeStateSnapshot {
    bool hasPeer{false};
    UdpPeer peer{};
    uint8_t runtimeMode{RUNTIME_MODE_IDLE};
    uint8_t slamMode{RUNTIME_SLAM_MODE_MAPPING};
    uint8_t trackingState{0xFF};
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
};

using ReadRuntimeStateFn = std::function<bool(UdpRuntimeStateSnapshot &)>;

struct UdpCommandRuntimeConfig {
    int port{0};
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

class UdpCommandRuntime final {
  public:
    explicit UdpCommandRuntime(UdpCommandRuntimeConfig config);
    ~UdpCommandRuntime();

    bool Start();
    void Stop();
    void Step();

  private:
    class Impl;

    std::unique_ptr<Impl> m_impl;
};

} // namespace smartdrone::core::application
