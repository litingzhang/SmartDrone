#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "common/tlv/tlv_cmd_router.hpp"
#include "common/tlv/tlv_pack.hpp"
#include "common/tlv/tlv_parser.hpp"
#include "common/tlv/tlv_protocol.hpp"
#include "common/tlv/udp_server.hpp"
#include "common/time_utils.hpp"
#include "core/application/config_registry.hpp"
#include "core/application/live_pose_state.hpp"
#include "core/application/px4_udp_hooks.hpp"
#include "core/application/runtime_app_types.hpp"
#include "core/application/runtime_command_service.hpp"
#include "core/application/runtime_controller.hpp"

namespace smartdrone::core::application {

constexpr uint8_t kCmdPointCloud = 0xF2;
constexpr uint16_t kPointCloudHeaderLen = 4;

using BuildCapabilitiesPayloadFn = std::function<std::vector<uint8_t>()>;
using BuildConfigPayloadFn = std::function<std::vector<uint8_t>(const UnifiedConfig&, smartdrone::core::domain::RuntimeMode)>;
using PeerToIpStringFn = std::function<std::string(const UdpPeer&)>;

inline SensorMode ParseRuntimeSensorMode(uint8_t value)
{
    switch (value) {
        case RUNTIME_SENSOR_STEREO_IMU: return SensorMode::StereoImu;
        case RUNTIME_SENSOR_MONO: return SensorMode::Mono;
        case RUNTIME_SENSOR_MONO_IMU: return SensorMode::MonoImu;
        case RUNTIME_SENSOR_STEREO:
        default: return SensorMode::Stereo;
    }
}

inline RouteResult HandleRuntimeModeFrame(const TlvFrame& frame, UnifiedRuntimeController& controller)
{
    using ControllerMode = smartdrone::core::domain::RuntimeMode;
    if (frame.len != RUNTIME_MODE_PAYLOAD_LEN) return {ACK_E_BAD_LEN, "bad runtime mode len"};
    RuntimeAction action{};
    if (frame.payload[0] == RUNTIME_MODE_SLAM) {
        action.type = RuntimeAction::Type::StartRuntime;
        action.selection.runtimeMode = ControllerMode::Slam;
    } else if (frame.payload[0] == RUNTIME_MODE_CALIB) {
        action.type = RuntimeAction::Type::StartRuntime;
        action.selection.runtimeMode = ControllerMode::Calib;
    } else if (frame.payload[0] == RUNTIME_MODE_IDLE) {
        action.type = RuntimeAction::Type::StopRuntime;
        action.selection.runtimeMode = ControllerMode::Idle;
    } else {
        return {ACK_E_BAD_ARGS, "bad runtime mode"};
    }
    RuntimeCommandService service(controller);
    const auto result = service.ExecuteAction(action);
    return {result.ok ? ACK_OK : ACK_E_BAD_STATE, result.message};
}

inline RouteResult HandleRuntimeConfigFrame(
    const TlvFrame& frame,
    const UdpPeer& peer,
    UnifiedRuntimeController& controller,
    const PeerToIpStringFn& peerToIpString)
{
    if (frame.len != RUNTIME_CONFIG_PAYLOAD_LEN && frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_LEGACY) {
        return {ACK_E_BAD_LEN, "bad runtime cfg len"};
    }
    const uint8_t* p = frame.payload.data();
    const UnifiedConfig currentCfg = controller.CurrentConfig();
    RemoteRuntimeConfig r{};
    r.exposureUs = static_cast<int>(ReadU32Le(&p[0]));
    r.gain = ReadF32Le(&p[4]);
    r.pairMs = currentCfg.app.camera.pairMs > 0 ? currentCfg.app.camera.pairMs : 1;
    r.slamInputFps = currentCfg.app.runtime.slamInputFps;
    if (r.exposureUs <= 0 || !std::isfinite(r.gain)) {
        return {ACK_E_BAD_ARGS, "bad runtime cfg args"};
    }
    r.sensorMode = ParseRuntimeSensorMode(p[8]);
    const uint8_t streamFlags = p[9];
    if (streamFlags == 0) {
        r.sendImage = true;
        r.sendFeature = false;
        r.sendMap = false;
    } else {
        r.sendImage = (streamFlags & RUNTIME_CFG_FLAG_SEND_IMAGE) != 0;
        r.sendFeature = (streamFlags & RUNTIME_CFG_FLAG_SEND_FEATURE) != 0;
        r.sendMap = (streamFlags & RUNTIME_CFG_FLAG_SEND_MAP) != 0;
    }
    size_t ipOffset = 10;
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN) {
        const int pairMs = static_cast<int>(ReadU16Le(&p[RUNTIME_CONFIG_PAIR_MS_OFFSET]));
        if (pairMs > 0) r.pairMs = pairMs;
        r.slamInputFps = static_cast<int>(ReadU16Le(&p[RUNTIME_CONFIG_SLAM_FPS_OFFSET]));
        ipOffset = RUNTIME_CONFIG_IP_OFFSET;
    }
    const char* ipChars = reinterpret_cast<const char*>(&p[ipOffset]);
    size_t ipLen = 0;
    while (ipLen < RUNTIME_CONFIG_IP_LEN && ipChars[ipLen] != '\0') {
        ++ipLen;
    }
    r.udpIp.assign(ipChars, ipLen);
    r.udpEnabled = !r.udpIp.empty();
    const std::string peerIp = peerToIpString ? peerToIpString(peer) : std::string{};
    if (!peerIp.empty()) {
        r.udpIp = peerIp;
        r.udpEnabled = true;
    }

    ConfigUpdate update{};
    update.values[std::string(ConfigRegistry::kCameraExposureUs)] = static_cast<int64_t>(r.exposureUs);
    update.values[std::string(ConfigRegistry::kCameraGain)] = static_cast<double>(r.gain);
    update.values[std::string(ConfigRegistry::kCameraPairWindowMs)] = static_cast<int64_t>(r.pairMs);
    update.values[std::string(ConfigRegistry::kSlamInputFps)] = static_cast<int64_t>(r.slamInputFps);
    update.values[std::string(ConfigRegistry::kSlamPerceptionMode)] =
        std::string(ToSensorModeText(r.sensorMode));
    update.values[std::string(ConfigRegistry::kStreamUdpEnabled)] = r.udpEnabled;
    update.values[std::string(ConfigRegistry::kStreamUdpIp)] = r.udpIp;
    update.values[std::string(ConfigRegistry::kStreamSendImage)] = r.sendImage;
    update.values[std::string(ConfigRegistry::kStreamSendFeature)] = r.sendFeature;
    update.values[std::string(ConfigRegistry::kStreamSendMap)] = r.sendMap;

    RuntimeCommandService service(controller);
    const auto result = service.ApplyConfig(update);
    if (!result.ok) {
        return {ACK_E_BAD_ARGS, result.message};
    }
    return {ACK_OK, result.message + " udp=" + r.udpIp +
                        " settings=" + std::string(DefaultSettingsForSensorMode(r.sensorMode)) +
                        " img=" + (r.sendImage ? "on" : "off") +
                        " feat=" + (r.sendFeature ? "on" : "off") +
                        " map=" + (r.sendMap ? "on" : "off")};
}

inline RouteResult HandleCalibCleanFrame(const TlvFrame& frame, UnifiedRuntimeController& controller)
{
    if (frame.len != 0) {
        return {ACK_E_BAD_LEN, "bad calib clean len"};
    }
    RuntimeAction action{};
    action.type = RuntimeAction::Type::CleanCalibration;
    RuntimeCommandService service(controller);
    const auto result = service.ExecuteAction(action);
    return {result.ok ? ACK_OK : ACK_E_BAD_STATE, result.message};
}

inline RouteResult HandleGetCapabilitiesFrame(const TlvFrame& frame, const BuildCapabilitiesPayloadFn& buildCapabilitiesPayload)
{
    if (frame.len != 0) {
        return {ACK_E_BAD_LEN, "bad capabilities query len"};
    }
    RouteResult out{};
    out.status = ACK_OK;
    out.msg = "capabilities queried";
    out.responseCmd = CMD_CAPABILITIES;
    out.responsePayload = buildCapabilitiesPayload ? buildCapabilitiesPayload() : std::vector<uint8_t>{};
    return out;
}

inline RouteResult HandleGetConfigFrame(
    const TlvFrame& frame,
    UnifiedRuntimeController& controller,
    const BuildConfigPayloadFn& buildConfigPayload)
{
    if (frame.len != 0) {
        return {ACK_E_BAD_LEN, "bad config query len"};
    }
    RouteResult out{};
    out.status = ACK_OK;
    out.msg = "config queried";
    out.responseCmd = CMD_CONFIG;
    out.responsePayload = buildConfigPayload
        ? buildConfigPayload(controller.CurrentConfig(), controller.CurrentDesiredMode())
        : std::vector<uint8_t>{};
    return out;
}

class CommandPeerGate {
public:
    static constexpr auto kPeerTimeout = std::chrono::seconds(5);

    bool Accept(const UdpPeer& peer, const std::chrono::steady_clock::time_point& now)
    {
        if (!peer.valid) {
            return false;
        }
        if (!m_lockedPeer.valid || (now - m_lastSeen) > kPeerTimeout) {
            m_lockedPeer = peer;
            m_lastSeen = now;
            return true;
        }
        if (SamePeer(peer, m_lockedPeer)) {
            m_lastSeen = now;
            return true;
        }
        return false;
    }

private:
    static bool SamePeer(const UdpPeer& a, const UdpPeer& b)
    {
        return a.valid && b.valid &&
               a.addr.sin_family == b.addr.sin_family &&
               a.addr.sin_port == b.addr.sin_port &&
               a.addr.sin_addr.s_addr == b.addr.sin_addr.s_addr;
    }

    UdpPeer m_lockedPeer{};
    std::chrono::steady_clock::time_point m_lastSeen{};
};

inline std::thread StartUdpCommandThread(
    int port,
    Px4UdpHooks& hooks,
    UnifiedRuntimeController& controller,
    LivePoseState& livePose,
    std::atomic<bool>& runningFlag,
    BuildCapabilitiesPayloadFn buildCapabilitiesPayload,
    BuildConfigPayloadFn buildConfigPayload,
    PeerToIpStringFn peerToIpString)
{
    return std::thread([port,
                        &hooks,
                        &controller,
                        &livePose,
                        &runningFlag,
                        buildCapabilitiesPayload = std::move(buildCapabilitiesPayload),
                        buildConfigPayload = std::move(buildConfigPayload),
                        peerToIpString = std::move(peerToIpString)]() {
        UdpServer server;
        if (!server.Open(static_cast<uint16_t>(port))) {
            std::cerr << "[udp_cmd] open failed on 0.0.0.0:" << port << "\n";
            return;
        }
        TlvCmdRouter router(hooks);
        router.RegisterDefaults();
        TlvParser parser;
        CommandPeerGate peerGate;
        uint8_t rx[2048]{};
        auto lastStateTx = std::chrono::steady_clock::now();
        auto lastRejectedPeerLog = std::chrono::steady_clock::time_point{};
        while (runningFlag.load()) {
            UdpPeer peer{};
            const int n = server.Recv(rx, sizeof(rx), &peer);
            if (n <= 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            } else {
                const auto now = std::chrono::steady_clock::now();
                if (!peerGate.Accept(peer, now)) {
                    if (now - lastRejectedPeerLog >= std::chrono::seconds(1)) {
                        lastRejectedPeerLog = now;
                        std::cerr << "[udp_cmd] rejected packet from non-active peer ip="
                                  << (peerToIpString ? peerToIpString(peer) : std::string{})
                                  << " port=" << ntohs(peer.addr.sin_port) << "\n";
                    }
                    continue;
                }
                livePose.UpdatePeer(peer);
                parser.Push(rx, static_cast<size_t>(n));
                while (auto frame = parser.TryPop()) {
                    RouteResult rr{};
                    if (frame->cmd == CMD_RUNTIME_MODE) rr = HandleRuntimeModeFrame(*frame, controller);
                    else if (frame->cmd == CMD_RUNTIME_CONFIG) rr = HandleRuntimeConfigFrame(*frame, peer, controller, peerToIpString);
                    else if (frame->cmd == CMD_CALIB_CLEAN) rr = HandleCalibCleanFrame(*frame, controller);
                    else if (frame->cmd == CMD_GET_CAPABILITIES) rr = HandleGetCapabilitiesFrame(*frame, buildCapabilitiesPayload);
                    else if (frame->cmd == CMD_GET_CONFIG) rr = HandleGetConfigFrame(*frame, controller, buildConfigPayload);
                    else rr = router.Handle(*frame);

                    std::vector<uint8_t> ack = MakeAckFrame(frame->seq, frame->tMs, frame->cmd, frame->seq, rr.status);
                    server.SendTo(peer, ack.data(), ack.size());
                    if (rr.status == ACK_OK && rr.responseCmd != 0) {
                        const uint8_t* payload = rr.responsePayload.empty() ? nullptr : rr.responsePayload.data();
                        std::vector<uint8_t> responseFrame =
                            MakeFrame(TLV_VER, rr.responseCmd, 0, frame->seq, MonoTimeMs32(),
                                      payload, static_cast<uint16_t>(rr.responsePayload.size()));
                        server.SendTo(peer, responseFrame.data(), responseFrame.size());
                    }
                    if (!rr.msg.empty()) std::cerr << "[udp_cmd] " << rr.msg << "\n";
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - lastStateTx >= std::chrono::milliseconds(100)) {
                lastStateTx = now;
                LivePoseState::Snapshot snap{};
                if (livePose.ConsumeSnapshot(snap) && snap.hasPeer) {
                    const UnifiedConfig currentCfg = controller.CurrentConfig();
                    if (currentCfg.app.udp.sendMap) {
                        std::vector<uint8_t> payload;
                        payload.reserve(STATE_POSE_PAYLOAD_LEN);
                        payload.push_back(snap.runtimeMode);
                        payload.push_back(snap.trackingState);
                        WriteU16Le(payload, snap.resetCounter);
                        WriteU16Le(payload, snap.resetMapCount);
                        WriteF32Le(payload, snap.x);
                        WriteF32Le(payload, snap.y);
                        WriteF32Le(payload, snap.z);
                        WriteF32Le(payload, snap.qw);
                        WriteF32Le(payload, snap.qx);
                        WriteF32Le(payload, snap.qy);
                        WriteF32Le(payload, snap.qz);
                        std::vector<uint8_t> stateFrame =
                            MakeFrame(TLV_VER, CMD_STATE, 0, snap.seq, MonoTimeMs32(),
                                      payload.data(), static_cast<uint16_t>(payload.size()));
                        server.SendTo(snap.peer, stateFrame.data(), stateFrame.size());

                        const size_t pointCount = snap.pointCloudXyz.size() / 3;
                        if (pointCount > 0) {
                            std::vector<uint8_t> cloudPayload;
                            cloudPayload.reserve(kPointCloudHeaderLen + pointCount * 12);
                            WriteU16Le(cloudPayload, static_cast<uint16_t>(std::min<size_t>(pointCount, 0xFFFF)));
                            WriteU16Le(cloudPayload, static_cast<uint16_t>(snap.pointCloudSeq & 0xFFFFu));
                            for (size_t i = 0; i < pointCount * 3; ++i) {
                                WriteF32Le(cloudPayload, snap.pointCloudXyz[i]);
                            }
                            std::vector<uint8_t> cloudFrame =
                                MakeFrame(TLV_VER, kCmdPointCloud, 0, snap.seq, MonoTimeMs32(),
                                          cloudPayload.data(), static_cast<uint16_t>(cloudPayload.size()));
                            server.SendTo(snap.peer, cloudFrame.data(), cloudFrame.size());
                        }
                    }
                }
            }
        }
    });
}

}  // namespace smartdrone::core::application
