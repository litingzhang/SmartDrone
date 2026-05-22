#include "core/application/runtime/udp_command_runtime.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>

#include "common/time_utils.h"
#include "common/tlv/tlv_cmd_router.h"
#include "common/tlv/tlv_pack.h"
#include "common/tlv/tlv_parser.h"
#include "core/application/runtime/runtime_config_frame_codec.h"
#include "core/application/runtime/runtime_command_service.h"

namespace SmartDrone::Core::Application {
namespace {

constexpr auto kHeartbeatPeriod = std::chrono::milliseconds(500);
constexpr auto kHeartbeatTimeout = std::chrono::seconds(3);
constexpr auto kOpenRetryPeriod = std::chrono::seconds(1);

int EnvIntValueClamped(const char *name, int fallback, int minValue,
                       int maxValue)
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

RouteResult HandleRuntimeModeFrame(const TlvFrame &frame,
                                   IRuntimeCommandTarget &commandTarget)
{
    using ControllerMode = SmartDrone::Core::Domain::RuntimeMode;
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
    RuntimeCommandService service(commandTarget);
    const auto result = service.ExecuteAction(action);
    return {result.ok ? ACK_OK : ACK_E_BAD_STATE, result.message};
}

RouteResult HandleRuntimeConfigFrame(const TlvFrame &frame, const UdpPeer &peer,
                                     IRuntimeCommandTarget &commandTarget,
                                     const CurrentConfigFn &currentConfig,
                                     const PeerToIpStringFn &peerToIpString)
{
    if (!RuntimeConfigPayloadLengthValid(frame.len)) {
        return {ACK_E_BAD_LEN, "bad runtime cfg len"};
    }

    RemoteRuntimeConfig remote =
        ParseRuntimeConfigFrame(frame, currentConfig());
    if (remote.exposureUs <= 0 || !std::isfinite(remote.gain)) {
        return {ACK_E_BAD_ARGS, "bad runtime cfg args"};
    }
    ApplyPeerIp(remote, peer, peerToIpString);

    RuntimeCommandService service(commandTarget);
    const ConfigUpdate update = BuildRuntimeConfigUpdate(remote);
    const auto result = service.ApplyConfig(update);
    if (!result.ok) {
        return {ACK_E_BAD_ARGS, result.message};
    }
    return {ACK_OK, BuildRuntimeConfigAckMessage(result.message, remote)};
}

RouteResult HandleCalibCleanFrame(const TlvFrame &frame,
                                  IRuntimeCommandTarget &commandTarget)
{
    if (frame.len != 0) {
        return {ACK_E_BAD_LEN, "bad calib clean len"};
    }
    RuntimeAction action{};
    action.type = RuntimeAction::Type::CleanCalibration;
    RuntimeCommandService service(commandTarget);
    const auto result = service.ExecuteAction(action);
    return {result.ok ? ACK_OK : ACK_E_BAD_STATE, result.message};
}

RouteResult HandleForceRestartFrame(const TlvFrame &frame,
                                    IRuntimeCommandTarget &commandTarget)
{
    if (frame.len != 0) {
        return {ACK_E_BAD_LEN, "bad force restart len"};
    }
    RuntimeAction action{};
    action.type = RuntimeAction::Type::ForceRestart;
    RuntimeCommandService service(commandTarget);
    const auto result = service.ExecuteAction(action);
    return {result.ok ? ACK_OK : ACK_E_BAD_STATE, result.message};
}

RouteResult HandleGetCapabilitiesFrame(
    const TlvFrame &frame,
    const BuildCapabilitiesPayloadFn &buildCapabilitiesPayload)
{
    if (frame.len != 0) {
        return {ACK_E_BAD_LEN, "bad capabilities query len"};
    }
    RouteResult out{};
    out.status = ACK_OK;
    out.msg = "capabilities queried";
    out.responseCmd = CMD_CAPABILITIES;
    out.responsePayload = buildCapabilitiesPayload ? buildCapabilitiesPayload()
                                                   : std::vector<uint8_t>{};
    return out;
}

RouteResult
HandleGetConfigFrame(const TlvFrame &frame,
                     const CurrentConfigFn &currentConfig,
                     const CurrentRuntimeModeFn &currentRuntimeMode,
                     const BuildConfigPayloadFn &buildConfigPayload)
{
    if (frame.len != 0) {
        return {ACK_E_BAD_LEN, "bad config query len"};
    }
    RouteResult out{};
    out.status = ACK_OK;
    out.msg = "config queried";
    out.responseCmd = CMD_CONFIG;
    out.responsePayload =
        buildConfigPayload ? buildConfigPayload(currentConfig(),
                                                currentRuntimeMode())
                           : std::vector<uint8_t>{};
    return out;
}

RuntimeCommandHook *ResolveCommandHook(const UdpCommandRuntimeConfig &config)
{
    return config.commandHook != nullptr ? config.commandHook : config.hooks;
}

class CommandPeerGate {
  public:
    static constexpr auto kPeerTimeout = std::chrono::seconds(5);

    bool Accept(const UdpPeer &peer,
                const std::chrono::steady_clock::time_point &now)
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
        return a.valid && b.valid && a.addr.sin_family == b.addr.sin_family &&
               a.addr.sin_port == b.addr.sin_port &&
               a.addr.sin_addr.s_addr == b.addr.sin_addr.s_addr;
    }

    UdpPeer m_lockedPeer{};
    std::chrono::steady_clock::time_point m_lastSeen{};
};

} // namespace

bool UdpCommandRuntimeConfigValid(const UdpCommandRuntimeConfig &config)
{
    return config.port > 0 && ResolveCommandHook(config) != nullptr && config.commandTarget != nullptr && config.currentConfig &&
           config.currentRuntimeMode && config.updateCommandPeer && config.readRuntimeState;
}

class UdpCommandRuntime::Impl final {
  public:
    explicit Impl(UdpCommandRuntimeConfig config)
        : m_config(std::move(config)),
          m_statePeriodMs(EnvIntValueClamped("SMART_DRONE_UDP_STATE_PERIOD_MS",
                                             100, 20, 1000))
    {
    }

    ~Impl()
    {
        Stop();
    }

    bool Start()
    {
        if (m_started) {
            return true;
        }
        const auto now = std::chrono::steady_clock::now();
        if (!CanRetryOpen(now)) {
            return false;
        }
        if (!ConfigValid()) {
            return false;
        }
        EnsureRouter();
        if (!m_server.Open(static_cast<uint16_t>(m_config.port))) {
            m_nextOpenAttempt = now + kOpenRetryPeriod;
            std::cerr << "[udp_cmd] open failed on 0.0.0.0:" << m_config.port
                      << "\n";
            return false;
        }
        m_nextOpenAttempt = {};
        m_started = true;
        return true;
    }

    void Stop()
    {
        m_server.Close();
        m_started = false;
    }

    void Step()
    {
        if (!m_started && !Start()) {
            return;
        }
        ReceiveOneDatagram();
        SendPeriodicOutputs();
    }

  private:
    bool ConfigValid() const
    {
        return UdpCommandRuntimeConfigValid(m_config);
    }

    bool CanRetryOpen(std::chrono::steady_clock::time_point now) const
    {
        return m_nextOpenAttempt.time_since_epoch().count() == 0 ||
               now >= m_nextOpenAttempt;
    }

    void EnsureRouter()
    {
        if (m_router) {
            return;
        }
        m_router = std::make_unique<TlvCmdRouter>(*ResolveCommandHook(m_config));
        m_router->RegisterDefaults();
    }

    void ReceiveOneDatagram()
    {
        UdpPeer peer{};
        const int received = m_server.Recv(m_rxBuffer.data(), m_rxBuffer.size(), &peer);
        if (received <= 0) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (!m_peerGate.Accept(peer, now)) {
            LogRejectedPeer(peer, now);
            return;
        }
        m_activePeer = peer;
        m_config.updateCommandPeer(peer);
        m_parser.Push(m_rxBuffer.data(), static_cast<size_t>(received));
        while (auto frame = m_parser.TryPop()) {
            HandleFrame(*frame, peer, now);
        }
    }

    void LogRejectedPeer(const UdpPeer &peer,
                         std::chrono::steady_clock::time_point now)
    {
        if (now - m_lastRejectedPeerLog < std::chrono::seconds(1)) {
            return;
        }
        m_lastRejectedPeerLog = now;
        std::cerr << "[udp_cmd] rejected packet from non-active peer ip="
                  << PeerText(peer) << " port=" << ntohs(peer.addr.sin_port)
                  << "\n";
    }

    void HandleFrame(const TlvFrame &frame, const UdpPeer &peer,
                     std::chrono::steady_clock::time_point now)
    {
        if (frame.cmd == CMD_HEARTBEAT) {
            m_haveHeartbeatPeer = true;
            m_heartbeatLandTriggered = false;
            m_lastHeartbeatRx = now;
            return;
        }
        RouteResult routeResult = RouteFrame(frame, peer);
        SendRouteResponse(frame, peer, routeResult);
        if (!routeResult.msg.empty()) {
            std::cerr << "[udp_cmd] " << routeResult.msg << "\n";
        }
    }

    RouteResult RouteFrame(const TlvFrame &frame, const UdpPeer &peer)
    {
        if (frame.cmd == CMD_RUNTIME_MODE) {
            return HandleRuntimeModeFrame(frame, *m_config.commandTarget);
        }
        if (frame.cmd == CMD_RUNTIME_CONFIG) {
            return HandleRuntimeConfigFrame(frame, peer, *m_config.commandTarget,
                                            m_config.currentConfig,
                                            m_config.peerToIpString);
        }
        if (frame.cmd == CMD_CALIB_CLEAN) {
            return HandleCalibCleanFrame(frame, *m_config.commandTarget);
        }
        if (frame.cmd == CMD_GET_CAPABILITIES) {
            return HandleGetCapabilitiesFrame(frame, m_config.buildCapabilitiesPayload);
        }
        if (frame.cmd == CMD_GET_CONFIG) {
            return HandleGetConfigFrame(frame, m_config.currentConfig,
                                        m_config.currentRuntimeMode,
                                        m_config.buildConfigPayload);
        }
        if (frame.cmd == CMD_FORCE_RESTART) {
            return HandleForceRestartFrame(frame, *m_config.commandTarget);
        }
        if (!m_router) {
            return {ACK_E_BAD_STATE, "command router not ready"};
        }
        return m_router->Handle(frame);
    }

    void SendRouteResponse(const TlvFrame &frame, const UdpPeer &peer,
                           const RouteResult &routeResult)
    {
        std::vector<uint8_t> ack =
            MakeAckFrame(frame.seq, frame.tMs, frame.cmd, frame.seq,
                         routeResult.status);
        m_server.SendTo(peer, ack.data(), ack.size());
        if (routeResult.status != ACK_OK || routeResult.responseCmd == 0) {
            return;
        }
        const uint8_t *payload =
            routeResult.responsePayload.empty() ? nullptr
                                                : routeResult.responsePayload.data();
        const TlvFrameBuildRequest responseRequest{
            TLV_VER,
            routeResult.responseCmd,
            0,
            frame.seq,
            MonoTimeMs32(),
            payload,
            static_cast<uint16_t>(routeResult.responsePayload.size())};
        std::vector<uint8_t> responseFrame = MakeFrame(responseRequest);
        m_server.SendTo(peer, responseFrame.data(), responseFrame.size());
    }

    void SendPeriodicOutputs()
    {
        const auto now = std::chrono::steady_clock::now();
        SendHeartbeat(now);
        HandleHeartbeatTimeout(now);
        SendStateIfDue(now);
    }

    void SendHeartbeat(std::chrono::steady_clock::time_point now)
    {
        if (!m_activePeer.valid) {
            return;
        }
        if (m_lastHeartbeatTx.time_since_epoch().count() != 0 &&
            now - m_lastHeartbeatTx < kHeartbeatPeriod) {
            return;
        }
        m_lastHeartbeatTx = now;
        const TlvFrameBuildRequest heartbeatRequest{
            TLV_VER, kCmdHeartbeat, 0, 0, MonoTimeMs32(), nullptr, 0};
        std::vector<uint8_t> heartbeatFrame = MakeFrame(heartbeatRequest);
        m_server.SendTo(m_activePeer, heartbeatFrame.data(), heartbeatFrame.size());
    }

    void HandleHeartbeatTimeout(std::chrono::steady_clock::time_point now)
    {
        UdpRuntimeStateSnapshot snapshot{};
        const bool vehicleArmed =
            m_config.readRuntimeState(snapshot) && snapshot.armed;
        if (!m_haveHeartbeatPeer || !vehicleArmed || m_heartbeatLandTriggered ||
            now - m_lastHeartbeatRx <= kHeartbeatTimeout) {
            if (!vehicleArmed) {
                m_heartbeatLandTriggered = false;
            }
            return;
        }
        m_heartbeatLandTriggered = true;
        std::string err;
        if (!ResolveCommandHook(m_config)->LandVehicle(&err)) {
            std::cerr << "[udp_cmd] heartbeat timeout land failed: " << err << "\n";
            return;
        }
        std::cerr << "[udp_cmd] heartbeat timeout >3s, LAND triggered\n";
    }

    void SendStateIfDue(std::chrono::steady_clock::time_point now)
    {
        if (now - m_lastStateTx < std::chrono::milliseconds(m_statePeriodMs)) {
            return;
        }
        m_lastStateTx = now;
        UdpRuntimeStateSnapshot snapshot{};
        if (!m_config.readRuntimeState(snapshot) || !snapshot.hasPeer) {
            return;
        }
        SendState(snapshot);
        SendPointCloudIfNeeded(snapshot);
    }

    void SendState(const UdpRuntimeStateSnapshot &snapshot)
    {
        std::vector<uint8_t> payload;
        payload.reserve(STATE_POSE_PAYLOAD_LEN);
        payload.push_back(snapshot.runtimeMode);
        payload.push_back(snapshot.slamMode);
        payload.push_back(snapshot.trackingState);
        payload.push_back(snapshot.armed ? 1u : 0u);
        WriteU16Le(payload, snapshot.resetCounter);
        WriteU16Le(payload, snapshot.resetMapCount);
        WriteF32Le(payload, snapshot.x);
        WriteF32Le(payload, snapshot.y);
        WriteF32Le(payload, snapshot.z);
        WriteF32Le(payload, snapshot.qw);
        WriteF32Le(payload, snapshot.qx);
        WriteF32Le(payload, snapshot.qy);
        WriteF32Le(payload, snapshot.qz);
        payload.push_back(snapshot.px4MainMode);
        payload.push_back(snapshot.px4SubMode);
        const TlvFrameBuildRequest stateRequest{
            TLV_VER,
            CMD_STATE,
            0,
            snapshot.seq,
            MonoTimeMs32(),
            payload.data(),
            static_cast<uint16_t>(payload.size())};
        std::vector<uint8_t> stateFrame = MakeFrame(stateRequest);
        m_server.SendTo(snapshot.peer, stateFrame.data(), stateFrame.size());
    }

    void SendPointCloudIfNeeded(const UdpRuntimeStateSnapshot &snapshot)
    {
        const UnifiedConfig currentConfig = m_config.currentConfig();
        if (!currentConfig.app.udp.sendMap) {
            return;
        }
        const std::shared_ptr<const std::vector<float>> &pointCloudXyz =
            snapshot.pointCloudXyz;
        const size_t pointCount = pointCloudXyz ? (pointCloudXyz->size() / 3) : 0;
        if (pointCount == 0 || snapshot.pointCloudSeq == m_lastSentPointCloudSeq) {
            return;
        }
        SendPointCloud(snapshot, *pointCloudXyz, pointCount);
    }

    void SendPointCloud(const UdpRuntimeStateSnapshot &snapshot,
                        const std::vector<float> &pointCloudXyz,
                        size_t pointCount)
    {
        const size_t cappedPointCount =
            std::min(pointCount, kMaxPointCloudPointsPerFrame);
        std::vector<uint8_t> payload;
        payload.reserve(kPointCloudHeaderLen +
                        cappedPointCount * kPointCloudPointStrideBytes);
        WriteU16Le(payload, static_cast<uint16_t>(cappedPointCount));
        WriteU16Le(payload, static_cast<uint16_t>(snapshot.pointCloudSeq & 0xFFFFu));
        for (size_t index = 0; index < cappedPointCount * 3; ++index) {
            WriteF32Le(payload, pointCloudXyz[index]);
        }
        const TlvFrameBuildRequest cloudRequest{
            TLV_VER,
            kCmdPointCloud,
            0,
            snapshot.seq,
            MonoTimeMs32(),
            payload.data(),
            static_cast<uint16_t>(payload.size())};
        std::vector<uint8_t> cloudFrame = MakeFrame(cloudRequest);
        m_server.SendTo(snapshot.peer, cloudFrame.data(), cloudFrame.size());
        m_lastSentPointCloudSeq = snapshot.pointCloudSeq;
        if (cappedPointCount < pointCount) {
            std::cerr << "[udp_cmd] point cloud truncated points=" << pointCount
                      << " sent=" << cappedPointCount << "\n";
        }
    }

    std::string PeerText(const UdpPeer &peer) const
    {
        return m_config.peerToIpString ? m_config.peerToIpString(peer) : std::string{};
    }

    UdpCommandRuntimeConfig m_config;
    UdpServer m_server;
    std::unique_ptr<TlvCmdRouter> m_router;
    TlvParser m_parser;
    CommandPeerGate m_peerGate;
    std::array<uint8_t, 2048> m_rxBuffer{};
    std::chrono::steady_clock::time_point m_lastStateTx{
        std::chrono::steady_clock::now()};
    std::chrono::steady_clock::time_point m_lastHeartbeatTx{};
    std::chrono::steady_clock::time_point m_lastHeartbeatRx{};
    std::chrono::steady_clock::time_point m_lastRejectedPeerLog{};
    std::chrono::steady_clock::time_point m_nextOpenAttempt{};
    uint32_t m_lastSentPointCloudSeq{0};
    UdpPeer m_activePeer{};
    int m_statePeriodMs{100};
    bool m_haveHeartbeatPeer{false};
    bool m_heartbeatLandTriggered{false};
    bool m_started{false};
};

UdpCommandRuntime::UdpCommandRuntime(UdpCommandRuntimeConfig config)
    : m_impl(new Impl(std::move(config)))
{
}

UdpCommandRuntime::~UdpCommandRuntime() = default;

bool UdpCommandRuntime::Start()
{
    return m_impl->Start();
}

void UdpCommandRuntime::Stop()
{
    m_impl->Stop();
}

void UdpCommandRuntime::OnGraphTick()
{
    m_impl->Step();
}

void UdpCommandRuntime::Step()
{
    OnGraphTick();
}

} // namespace SmartDrone::Core::Application
