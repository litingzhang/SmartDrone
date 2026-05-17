#include "core/application/runtime/udp_command_thread.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "common/thread_launch.h"
#include "common/time_utils.h"
#include "common/tlv/tlv_cmd_router.h"
#include "common/tlv/tlv_pack.h"
#include "common/tlv/tlv_parser.h"
#include "core/application/config/config_registry.h"
#include "core/application/runtime/runtime_command_service.h"

namespace smartdrone::core::application {
namespace {

constexpr auto kHeartbeatPeriod = std::chrono::milliseconds(500);
constexpr auto kHeartbeatTimeout = std::chrono::seconds(3);

int EnvIntValueClamped(const char *name, int fallback, int minValue, int maxValue)
{
    const char *value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    char *end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value) {
        return fallback;
    }
    return std::clamp(static_cast<int>(parsed), minValue, maxValue);
}

SensorMode ParseRuntimeSensorMode(uint8_t value)
{
    switch (value) {
    case RUNTIME_SENSOR_STEREO_IMU:
        return SensorMode::StereoImu;
    case RUNTIME_SENSOR_MONO:
        return SensorMode::Mono;
    case RUNTIME_SENSOR_MONO_IMU:
        return SensorMode::MonoImu;
    case RUNTIME_SENSOR_STEREO:
    default:
        return SensorMode::Stereo;
    }
}

FeatureFrontend ParseRuntimeFeatureFrontend(uint8_t value)
{
    switch (value) {
    case RUNTIME_FEATURE_FRONTEND_XFEAT_LIGHTGLUE:
        return FeatureFrontend::XFeatLightGlue;
    case RUNTIME_FEATURE_FRONTEND_SUPERPOINT_LIGHTGLUE:
        return FeatureFrontend::SuperPointLightGlue;
    case RUNTIME_FEATURE_FRONTEND_LK_GFTT_PER_FRAME:
        return FeatureFrontend::LkGfttPerFrame;
    case RUNTIME_FEATURE_FRONTEND_LK:
        return FeatureFrontend::LK;
    case RUNTIME_FEATURE_FRONTEND_ORB:
        return FeatureFrontend::Orb;
    case RUNTIME_FEATURE_FRONTEND_RESERVED_SUPERPOINT:
    case RUNTIME_FEATURE_FRONTEND_RESERVED_LEGACY:
    default:
        return FeatureFrontend::LkGfttPerFrame;
    }
}

std::string ParseRuntimeLkPerFrameAcceleration(uint8_t value)
{
    switch (value) {
    case RUNTIME_LK_PER_FRAME_ACCEL_VPI_CUDA:
        return "vpi-cuda";
    case RUNTIME_LK_PER_FRAME_ACCEL_CPU:
    default:
        return "cpu";
    }
}

std::string ParseRuntimeOrbAcceleration(uint8_t value)
{
    switch (value) {
    case RUNTIME_ORB_ACCEL_OPENCV_CUDA:
        return "cuda";
    case RUNTIME_ORB_ACCEL_VPI_REMAP:
        return "vpi-remap";
    case RUNTIME_ORB_ACCEL_CPU:
    default:
        return "cpu";
    }
}

SlamBackend ParseRuntimeSlamBackend(uint8_t value)
{
    switch (value) {
    case RUNTIME_SLAM_BACKEND_KLT:
        return SlamBackend::Klt;
    case RUNTIME_SLAM_BACKEND_DPVO_TENSORRT:
        return SlamBackend::DpvoTensorRt;
    case RUNTIME_SLAM_BACKEND_ORBSLAM3:
#if defined(SMART_DRONE_ENABLE_ORB_SLAM3)
        return SlamBackend::OrbSlam3;
#else
        return SlamBackend::Klt;
#endif
    default:
        return SlamBackend::Klt;
    }
}

smartdrone::core::domain::SlamOperationMode ParseRuntimeSlamMode(uint8_t value)
{
    using smartdrone::core::domain::SlamOperationMode;

    switch (value) {
    case RUNTIME_SLAM_MODE_LOCALIZATION:
        return SlamOperationMode::Localization;
    case RUNTIME_SLAM_MODE_RELOCALIZATION:
        return SlamOperationMode::Relocalization;
    case RUNTIME_SLAM_MODE_TRACKING_ONLY:
        return SlamOperationMode::TrackingOnly;
    case RUNTIME_SLAM_MODE_AUTO:
        return SlamOperationMode::Auto;
    case RUNTIME_SLAM_MODE_MAPPING:
    default:
        return SlamOperationMode::Mapping;
    }
}

RouteResult HandleRuntimeModeFrame(const TlvFrame &frame, UnifiedRuntimeController &controller)
{
    using ControllerMode = smartdrone::core::domain::RuntimeMode;
    if (frame.len != RUNTIME_MODE_PAYLOAD_LEN)
        return {ACK_E_BAD_LEN, "bad runtime mode len"};
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

RouteResult HandleRuntimeConfigFrame(const TlvFrame &frame, const UdpPeer &peer, UnifiedRuntimeController &controller,
                                     const PeerToIpStringFn &peerToIpString)
{
    if (frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_V14 &&
        frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_V13 &&
        frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_V12 &&
        frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_V11 &&
        frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_V10 && frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_V9 &&
        frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_V8 &&
        frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_V7 &&
        frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_V6 &&
        frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_V5 &&
        frame.len != RUNTIME_CONFIG_PAYLOAD_LEN &&
        frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_V3 &&
        frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_V2 &&
        frame.len != RUNTIME_CONFIG_PAYLOAD_LEN_LEGACY) {
        return {ACK_E_BAD_LEN, "bad runtime cfg len"};
    }
    const uint8_t *p = frame.payload.data();
    const UnifiedConfig currentCfg = controller.CurrentConfig();
    RemoteRuntimeConfig r{};
    r.exposureUs = static_cast<int>(ReadU32Le(&p[0]));
    r.gain = ReadF32Le(&p[4]);
    r.pairMs = currentCfg.app.camera.pairMs > 0 ? currentCfg.app.camera.pairMs : 1;
    r.autoExposureEnabled = !currentCfg.app.camera.aeDisable;
    r.slamInputFps = currentCfg.app.runtime.slamInputFps;
    r.slamOperationMode = currentCfg.app.runtime.slamOperationMode;
    r.slamBackend = currentCfg.app.runtime.slamBackend;
    r.featureFrontend = currentCfg.app.runtime.featureFrontend;
    r.useCustomTbc = currentCfg.app.runtime.useCustomTbc;
    r.tbcTx = currentCfg.app.runtime.tbcTx;
    r.tbcTy = currentCfg.app.runtime.tbcTy;
    r.tbcTz = currentCfg.app.runtime.tbcTz;
    r.tbcRollDeg = currentCfg.app.runtime.tbcRollDeg;
    r.tbcPitchDeg = currentCfg.app.runtime.tbcPitchDeg;
    r.tbcYawDeg = currentCfg.app.runtime.tbcYawDeg;
    r.orbNFeatures = currentCfg.app.runtime.orbNFeatures;
    r.orbScaleFactor = currentCfg.app.runtime.orbScaleFactor;
    r.orbNLevels = currentCfg.app.runtime.orbNLevels;
    r.orbIniThFAST = currentCfg.app.runtime.orbIniThFAST;
    r.orbMinThFAST = currentCfg.app.runtime.orbMinThFAST;
    r.superpointTopK = currentCfg.app.runtime.superpointTopK;
    r.superpointMaxPoints = currentCfg.app.runtime.superpointMaxPoints;
    r.superpointInputMaxWidth = currentCfg.app.runtime.superpointInputMaxWidth;
    r.superpointInputMaxHeight = currentCfg.app.runtime.superpointInputMaxHeight;
    r.lkPerFrameAcceleration = currentCfg.app.runtime.lkPerFrameAcceleration;
    r.orbAcceleration = currentCfg.app.runtime.orbAcceleration;
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
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V2) {
        const int pairMs = static_cast<int>(ReadU16Le(&p[RUNTIME_CONFIG_PAIR_MS_OFFSET]));
        if (pairMs > 0)
            r.pairMs = pairMs;
        r.slamInputFps = static_cast<int>(ReadU16Le(&p[RUNTIME_CONFIG_SLAM_FPS_OFFSET]));
        ipOffset = RUNTIME_CONFIG_IP_OFFSET;
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN) {
        r.autoExposureEnabled = p[RUNTIME_CONFIG_AE_OFFSET] != 0;
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V3) {
        r.slamOperationMode = ParseRuntimeSlamMode(p[RUNTIME_CONFIG_SLAM_MODE_OFFSET]);
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V5) {
        r.useCustomTbc = p[RUNTIME_CONFIG_TBC_OVERRIDE_ENABLE_OFFSET] != 0;
        r.tbcTx = ReadF32Le(&p[RUNTIME_CONFIG_TBC_TX_OFFSET]);
        r.tbcTy = ReadF32Le(&p[RUNTIME_CONFIG_TBC_TY_OFFSET]);
        r.tbcTz = ReadF32Le(&p[RUNTIME_CONFIG_TBC_TZ_OFFSET]);
        r.tbcPitchDeg = ReadF32Le(&p[RUNTIME_CONFIG_TBC_PITCH_DEG_OFFSET]);
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V6) {
        r.tbcRollDeg = ReadF32Le(&p[RUNTIME_CONFIG_TBC_ROLL_DEG_OFFSET]);
        r.tbcYawDeg = ReadF32Le(&p[RUNTIME_CONFIG_TBC_YAW_DEG_OFFSET]);
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V7) {
        r.orbNFeatures = static_cast<int>(std::lround(ReadF32Le(&p[RUNTIME_CONFIG_ORB_NFEATURES_OFFSET])));
        r.orbScaleFactor = ReadF32Le(&p[RUNTIME_CONFIG_ORB_SCALE_FACTOR_OFFSET]);
        r.orbNLevels = static_cast<int>(std::lround(ReadF32Le(&p[RUNTIME_CONFIG_ORB_NLEVELS_OFFSET])));
        r.orbIniThFAST = static_cast<int>(std::lround(ReadF32Le(&p[RUNTIME_CONFIG_ORB_INI_TH_FAST_OFFSET])));
        r.orbMinThFAST = static_cast<int>(std::lround(ReadF32Le(&p[RUNTIME_CONFIG_ORB_MIN_TH_FAST_OFFSET])));
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V8) {
        r.featureFrontend = ParseRuntimeFeatureFrontend(p[RUNTIME_CONFIG_FEATURE_FRONTEND_OFFSET]);
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V9) {
        r.superpointTopK = static_cast<int>(std::lround(ReadF32Le(&p[RUNTIME_CONFIG_SUPERPOINT_TOP_K_OFFSET])));
        r.superpointMaxPoints = static_cast<int>(std::lround(ReadF32Le(&p[RUNTIME_CONFIG_SUPERPOINT_MAX_POINTS_OFFSET])));
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V10) {
        r.superpointInputMaxWidth =
            static_cast<int>(std::lround(ReadF32Le(&p[RUNTIME_CONFIG_SUPERPOINT_INPUT_MAX_WIDTH_OFFSET])));
        r.superpointInputMaxHeight =
            static_cast<int>(std::lround(ReadF32Le(&p[RUNTIME_CONFIG_SUPERPOINT_INPUT_MAX_HEIGHT_OFFSET])));
    }
    // Offset 106 is retained for wire compatibility; LK seeding is always GFTT.
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V12) {
        r.lkPerFrameAcceleration = ParseRuntimeLkPerFrameAcceleration(p[RUNTIME_CONFIG_LK_PER_FRAME_ACCEL_OFFSET]);
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V13) {
        r.orbAcceleration = ParseRuntimeOrbAcceleration(p[RUNTIME_CONFIG_ORB_ACCEL_OFFSET]);
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V14) {
        r.slamBackend = ParseRuntimeSlamBackend(p[RUNTIME_CONFIG_SLAM_BACKEND_OFFSET]);
    }
    const char *ipChars = reinterpret_cast<const char *>(&p[ipOffset]);
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
    update.values[std::string(ConfigRegistry::kCameraAutoExposure)] = r.autoExposureEnabled;
    update.values[std::string(ConfigRegistry::kCameraPairWindowMs)] = static_cast<int64_t>(r.pairMs);
    update.values[std::string(ConfigRegistry::kSlamInputFps)] = static_cast<int64_t>(r.slamInputFps);
    update.values[std::string(ConfigRegistry::kSlamBackend)] = std::string(ToSlamBackendText(r.slamBackend));
    update.values[std::string(ConfigRegistry::kSlamFeatureFrontend)] = std::string(ToFeatureFrontendText(r.featureFrontend));
    update.values[std::string(ConfigRegistry::kSlamOperationMode)] =
        std::string(smartdrone::core::domain::ToString(r.slamOperationMode));
    update.values[std::string(ConfigRegistry::kSlamPerceptionMode)] = std::string(ToSensorModeText(r.sensorMode));
    update.values[std::string(ConfigRegistry::kStreamUdpEnabled)] = r.udpEnabled;
    update.values[std::string(ConfigRegistry::kStreamUdpIp)] = r.udpIp;
    update.values[std::string(ConfigRegistry::kStreamSendImage)] = r.sendImage;
    update.values[std::string(ConfigRegistry::kStreamSendFeature)] = r.sendFeature;
    update.values[std::string(ConfigRegistry::kStreamSendMap)] = r.sendMap;
    update.values[std::string(ConfigRegistry::kSlamUseCustomTbc)] = r.useCustomTbc;
    update.values[std::string(ConfigRegistry::kSlamTbcTx)] = static_cast<double>(r.tbcTx);
    update.values[std::string(ConfigRegistry::kSlamTbcTy)] = static_cast<double>(r.tbcTy);
    update.values[std::string(ConfigRegistry::kSlamTbcTz)] = static_cast<double>(r.tbcTz);
    update.values[std::string(ConfigRegistry::kSlamTbcRollDeg)] = static_cast<double>(r.tbcRollDeg);
    update.values[std::string(ConfigRegistry::kSlamTbcPitchDeg)] = static_cast<double>(r.tbcPitchDeg);
    update.values[std::string(ConfigRegistry::kSlamTbcYawDeg)] = static_cast<double>(r.tbcYawDeg);
    update.values[std::string(ConfigRegistry::kSlamOrbNFeatures)] = static_cast<int64_t>(r.orbNFeatures);
    update.values[std::string(ConfigRegistry::kSlamOrbScaleFactor)] = static_cast<double>(r.orbScaleFactor);
    update.values[std::string(ConfigRegistry::kSlamOrbNLevels)] = static_cast<int64_t>(r.orbNLevels);
    update.values[std::string(ConfigRegistry::kSlamOrbIniThFast)] = static_cast<int64_t>(r.orbIniThFAST);
    update.values[std::string(ConfigRegistry::kSlamOrbMinThFast)] = static_cast<int64_t>(r.orbMinThFAST);
    update.values[std::string(ConfigRegistry::kSlamSuperPointTopK)] = static_cast<int64_t>(r.superpointTopK);
    update.values[std::string(ConfigRegistry::kSlamSuperPointMaxPoints)] = static_cast<int64_t>(r.superpointMaxPoints);
    update.values[std::string(ConfigRegistry::kSlamSuperPointInputMaxWidth)] =
        static_cast<int64_t>(r.superpointInputMaxWidth);
    update.values[std::string(ConfigRegistry::kSlamSuperPointInputMaxHeight)] =
        static_cast<int64_t>(r.superpointInputMaxHeight);
    update.values[std::string(ConfigRegistry::kSlamLkSuperPointSeeding)] = false;
    update.values[std::string(ConfigRegistry::kSlamLkPerFrameAcceleration)] = r.lkPerFrameAcceleration;
    update.values[std::string(ConfigRegistry::kSlamOrbAcceleration)] = r.orbAcceleration;

    RuntimeCommandService service(controller);
    const auto result = service.ApplyConfig(update);
    if (!result.ok) {
        return {ACK_E_BAD_ARGS, result.message};
    }
    return {ACK_OK, result.message + " udp=" + r.udpIp +
                        " settings=" + std::string(DefaultSettingsForSensorMode(r.sensorMode)) +
                        " backend=" + std::string(ToSlamBackendText(r.slamBackend)) +
                        " frontend=" + std::string(ToFeatureFrontendText(r.featureFrontend)) +
                        " slam_mode=" + std::string(smartdrone::core::domain::ToString(r.slamOperationMode)) +
                        " img=" + (r.sendImage ? "on" : "off") + " feat=" + (r.sendFeature ? "on" : "off") +
                        " map=" + (r.sendMap ? "on" : "off") + " ae=" + (r.autoExposureEnabled ? "on" : "off") +
                        " tbc_override=" + (r.useCustomTbc ? "on" : "off") +
                        " lk_seed=gftt" +
                        " lk_accel=" + r.lkPerFrameAcceleration +
                        " orb_accel=" + r.orbAcceleration};
}

RouteResult HandleCalibCleanFrame(const TlvFrame &frame, UnifiedRuntimeController &controller)
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

RouteResult HandleForceRestartFrame(const TlvFrame &frame, UnifiedRuntimeController &controller)
{
    if (frame.len != 0) {
        return {ACK_E_BAD_LEN, "bad force restart len"};
    }
    RuntimeAction action{};
    action.type = RuntimeAction::Type::ForceRestart;
    RuntimeCommandService service(controller);
    const auto result = service.ExecuteAction(action);
    return {result.ok ? ACK_OK : ACK_E_BAD_STATE, result.message};
}

RouteResult HandleGetCapabilitiesFrame(const TlvFrame &frame,
                                       const BuildCapabilitiesPayloadFn &buildCapabilitiesPayload)
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

RouteResult HandleGetConfigFrame(const TlvFrame &frame, UnifiedRuntimeController &controller,
                                 const BuildConfigPayloadFn &buildConfigPayload)
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

    bool Accept(const UdpPeer &peer, const std::chrono::steady_clock::time_point &now)
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
    static bool SamePeer(const UdpPeer &a, const UdpPeer &b)
    {
        return a.valid && b.valid && a.addr.sin_family == b.addr.sin_family && a.addr.sin_port == b.addr.sin_port &&
               a.addr.sin_addr.s_addr == b.addr.sin_addr.s_addr;
    }

    UdpPeer m_lockedPeer{};
    std::chrono::steady_clock::time_point m_lastSeen{};
};

} // namespace

std::thread StartUdpCommandThread(int port, Px4UdpHooks &hooks, UnifiedRuntimeController &controller,
                                  LivePoseState &livePose, std::atomic<bool> &runningFlag,
                                  BuildCapabilitiesPayloadFn buildCapabilitiesPayload,
                                  BuildConfigPayloadFn buildConfigPayload, PeerToIpStringFn peerToIpString)
{
    return smartdrone::common::StartThread(
        smartdrone::common::MakeThreadLaunchInfo(smartdrone::common::ThreadRole::UdpCommand, "UdpCommandThread"),
        [port, &hooks, &controller, &livePose, &runningFlag,
         buildCapabilitiesPayload = std::move(buildCapabilitiesPayload),
         buildConfigPayload = std::move(buildConfigPayload), peerToIpString = std::move(peerToIpString)]() {
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
            const int statePeriodMs =
                EnvIntValueClamped("SMART_DRONE_UDP_STATE_PERIOD_MS", 100, 20, 1000);
            auto lastHeartbeatTx = std::chrono::steady_clock::time_point{};
            auto lastHeartbeatRx = std::chrono::steady_clock::time_point{};
            auto lastRejectedPeerLog = std::chrono::steady_clock::time_point{};
            uint32_t lastSentPointCloudSeq = 0;
            UdpPeer activePeer{};
            bool haveHeartbeatPeer = false;
            bool heartbeatLandTriggered = false;
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
                    activePeer = peer;
                    livePose.UpdatePeer(peer);
                    parser.Push(rx, static_cast<size_t>(n));
                    while (auto frame = parser.TryPop()) {
                        if (frame->cmd == CMD_HEARTBEAT) {
                            haveHeartbeatPeer = true;
                            heartbeatLandTriggered = false;
                            lastHeartbeatRx = now;
                            continue;
                        }
                        RouteResult rr{};
                        if (frame->cmd == CMD_RUNTIME_MODE)
                            rr = HandleRuntimeModeFrame(*frame, controller);
                        else if (frame->cmd == CMD_RUNTIME_CONFIG)
                            rr = HandleRuntimeConfigFrame(*frame, peer, controller, peerToIpString);
                        else if (frame->cmd == CMD_CALIB_CLEAN)
                            rr = HandleCalibCleanFrame(*frame, controller);
                        else if (frame->cmd == CMD_GET_CAPABILITIES)
                            rr = HandleGetCapabilitiesFrame(*frame, buildCapabilitiesPayload);
                        else if (frame->cmd == CMD_GET_CONFIG)
                            rr = HandleGetConfigFrame(*frame, controller, buildConfigPayload);
                        else if (frame->cmd == CMD_FORCE_RESTART)
                            rr = HandleForceRestartFrame(*frame, controller);
                        else
                            rr = router.Handle(*frame);

                        std::vector<uint8_t> ack =
                            MakeAckFrame(frame->seq, frame->tMs, frame->cmd, frame->seq, rr.status);
                        server.SendTo(peer, ack.data(), ack.size());
                        if (rr.status == ACK_OK && rr.responseCmd != 0) {
                            const uint8_t *payload = rr.responsePayload.empty() ? nullptr : rr.responsePayload.data();
                            std::vector<uint8_t> responseFrame =
                                MakeFrame(TLV_VER, rr.responseCmd, 0, frame->seq, MonoTimeMs32(), payload,
                                          static_cast<uint16_t>(rr.responsePayload.size()));
                            server.SendTo(peer, responseFrame.data(), responseFrame.size());
                        }
                        if (!rr.msg.empty())
                            std::cerr << "[udp_cmd] " << rr.msg << "\n";
                    }
                }

                const auto now = std::chrono::steady_clock::now();
                if (activePeer.valid && (lastHeartbeatTx.time_since_epoch().count() == 0 ||
                                         (now - lastHeartbeatTx) >= kHeartbeatPeriod)) {
                    lastHeartbeatTx = now;
                    std::vector<uint8_t> heartbeatFrame =
                        MakeFrame(TLV_VER, kCmdHeartbeat, 0, 0, MonoTimeMs32(), nullptr, 0);
                    server.SendTo(activePeer, heartbeatFrame.data(), heartbeatFrame.size());
                }
                LivePoseState::Snapshot heartbeatSnapshot{};
                const bool vehicleArmed = livePose.ReadSnapshot(heartbeatSnapshot) && heartbeatSnapshot.armed;
                if (haveHeartbeatPeer && vehicleArmed && !heartbeatLandTriggered &&
                    (now - lastHeartbeatRx) > kHeartbeatTimeout) {
                    heartbeatLandTriggered = true;
                    std::string err;
                    if (!hooks.Land(&err)) {
                        std::cerr << "[udp_cmd] heartbeat timeout land failed: " << err << "\n";
                    } else {
                        std::cerr << "[udp_cmd] heartbeat timeout >3s, LAND triggered\n";
                    }
                } else if (!vehicleArmed) {
                    heartbeatLandTriggered = false;
                }
                if (now - lastStateTx >= std::chrono::milliseconds(statePeriodMs)) {
                    lastStateTx = now;
                    LivePoseState::Snapshot snap{};
                    if (livePose.ReadSnapshot(snap) && snap.hasPeer) {
                        const UnifiedConfig currentCfg = controller.CurrentConfig();
                        std::vector<uint8_t> payload;
                        payload.reserve(STATE_POSE_PAYLOAD_LEN);
                        payload.push_back(snap.runtimeMode);
                        payload.push_back(snap.slamMode);
                        payload.push_back(snap.trackingState);
                        payload.push_back(snap.armed ? 1u : 0u);
                        WriteU16Le(payload, snap.resetCounter);
                        WriteU16Le(payload, snap.resetMapCount);
                        WriteF32Le(payload, snap.x);
                        WriteF32Le(payload, snap.y);
                        WriteF32Le(payload, snap.z);
                        WriteF32Le(payload, snap.qw);
                        WriteF32Le(payload, snap.qx);
                        WriteF32Le(payload, snap.qy);
                        WriteF32Le(payload, snap.qz);
                        payload.push_back(snap.px4MainMode);
                        payload.push_back(snap.px4SubMode);
                        std::vector<uint8_t> stateFrame =
                            MakeFrame(TLV_VER, CMD_STATE, 0, snap.seq, MonoTimeMs32(), payload.data(),
                                      static_cast<uint16_t>(payload.size()));
                        server.SendTo(snap.peer, stateFrame.data(), stateFrame.size());

                        if (currentCfg.app.udp.sendMap) {
                            const std::shared_ptr<const std::vector<float>> &pointCloudXyz = snap.pointCloudXyz;
                            const size_t pointCount = pointCloudXyz ? (pointCloudXyz->size() / 3) : 0;
                            if (pointCount > 0 && snap.pointCloudSeq != lastSentPointCloudSeq) {
                                const size_t cappedPointCount = std::min(pointCount, kMaxPointCloudPointsPerFrame);
                                std::vector<uint8_t> cloudPayload;
                                cloudPayload.reserve(kPointCloudHeaderLen +
                                                     cappedPointCount * kPointCloudPointStrideBytes);
                                WriteU16Le(cloudPayload, static_cast<uint16_t>(cappedPointCount));
                                WriteU16Le(cloudPayload, static_cast<uint16_t>(snap.pointCloudSeq & 0xFFFFu));
                                for (size_t i = 0; i < cappedPointCount * 3; ++i) {
                                    WriteF32Le(cloudPayload, (*pointCloudXyz)[i]);
                                }
                                std::vector<uint8_t> cloudFrame =
                                    MakeFrame(TLV_VER, kCmdPointCloud, 0, snap.seq, MonoTimeMs32(), cloudPayload.data(),
                                              static_cast<uint16_t>(cloudPayload.size()));
                                server.SendTo(snap.peer, cloudFrame.data(), cloudFrame.size());
                                lastSentPointCloudSeq = snap.pointCloudSeq;
                                if (cappedPointCount < pointCount) {
                                    std::cerr << "[udp_cmd] point cloud truncated points=" << pointCount
                                              << " sent=" << cappedPointCount << "\n";
                                }
                            }
                        }
                    }
                }
            }
        });
}

} // namespace smartdrone::core::application
