#include "adapters/telemetry/px4_mavlink_gateway.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

#include <time.h>

#include "common/time_utils.h"

namespace {

constexpr uint64_t kOdomTsLogEveryNFrames = 30;
constexpr std::size_t kMaxQueuedTxMessages = 64;

struct CovarianceDiagonal {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
};

void FillCovDiag21(float cov[21], const CovarianceDiagonal &diag, bool fillOffdiagZero = true)
{
    if (fillOffdiagZero) {
        for (int i = 0; i < 21; i++) {
            cov[i] = 0.0f;
        }
    }
    cov[0] = diag.x;
    cov[6] = diag.y;
    cov[11] = diag.z;
    cov[15] = diag.roll;
    cov[18] = diag.pitch;
    cov[20] = diag.yaw;
}

void FillNanCov21(float cov[21])
{
    for (int i = 0; i < 21; i++) {
        cov[i] = NAN;
    }
}

double NsDeltaToMs(uint64_t endNs, uint64_t startNs)
{
    if (startNs == 0 || endNs < startNs) {
        return -1.0;
    }
    return static_cast<double>(endNs - startNs) * 1e-6;
}

uint64_t ClockMonotonicNs()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

} // namespace

Px4MavlinkGateway::Px4MavlinkGateway(const std::string &dev, int baud, uint8_t sysid, uint8_t compid)
    : m_transport(dev, baud), m_sysid(sysid), m_compid(compid)
{
}

Px4MavlinkGateway::~Px4MavlinkGateway()
{
    StopSetpointStream();
}

void Px4MavlinkGateway::SetJsonDiagnostics(bool enabled)
{
    m_jsonDiagnostics.store(enabled, std::memory_order_relaxed);
}

void Px4MavlinkGateway::SetFrameTimingTracker(SmartDrone::core::application::FrameTimingTracker *tracker)
{
    std::lock_guard<std::mutex> lk(m_frameTimingTrackerMtx);
    m_frameTimingTracker = tracker;
}

int Px4MavlinkGateway::PollRxOnce(int timeoutMs)
{
    StepTx();
    const int pollResult = m_transport.PollReadable(timeoutMs);
    if (pollResult <= 0) {
        return 0;
    }

    uint8_t buffer[512];
    const ssize_t readLength = m_transport.Read(buffer, sizeof(buffer));
    if (readLength <= 0) {
        return 0;
    }
    int parsedCount = 0;
    for (ssize_t index = 0; index < readLength; ++index) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[index], &m_rxMessage, &m_rxStatus)) {
            HandleMavlinkMessage(m_rxMessage);
            ++parsedCount;
        }
    }
    return parsedCount;
}

void Px4MavlinkGateway::StepTx()
{
    std::lock_guard<std::mutex> txLock(m_txMtx);
    while (!m_txQueue.empty()) {
        auto &front = m_txQueue.front();
        const ssize_t written = m_transport.WriteSome(front.data() + m_txOffset, front.size() - m_txOffset);
        if (written > 0) {
            m_txOffset += static_cast<std::size_t>(written);
            if (m_txOffset >= front.size()) {
                m_txQueue.pop_front();
                m_txOffset = 0;
            }
            continue;
        }
        if (written == 0) {
            return;
        }
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            return;
        }
        printf("[mav] write failed len=%zu errno=%d\n", front.size(), errno);
        m_txQueue.pop_front();
        m_txOffset = 0;
        return;
    }
}

uint8_t Px4MavlinkGateway::GetTargetSystem() const
{
    return m_px4Sysid.load();
}

uint8_t Px4MavlinkGateway::GetTargetComponent() const
{
    return m_px4Compid.load();
}

bool Px4MavlinkGateway::GetLocalPositionNed(LocalPositionNed &out, uint64_t maxAgeUs) const
{
    std::lock_guard<std::mutex> lk(m_localPosMtx);
    if (!m_haveLocalPosNed) {
        return false;
    }
    if (maxAgeUs > 0 && (MonoTimeUs() - m_localPosNed.receivedUs) > maxAgeUs) {
        return false;
    }
    out = m_localPosNed;
    return true;
}

bool Px4MavlinkGateway::GetDownwardDistanceSensor(DownwardDistanceSensor &out, uint64_t maxAgeUs) const
{
    std::lock_guard<std::mutex> lk(m_distanceSensorMtx);
    if (!m_haveDownwardDistanceSensor) {
        return false;
    }
    if (maxAgeUs > 0 && (MonoTimeUs() - m_downwardDistanceSensor.receivedUs) > maxAgeUs) {
        return false;
    }
    out = m_downwardDistanceSensor;
    return true;
}

bool Px4MavlinkGateway::TryConsumeCommandAck(uint16_t command, uint8_t &outResult)
{
    std::lock_guard<std::mutex> lk(m_ackMtx);
    const auto it = m_ackMap.find(command);
    if (it == m_ackMap.end()) {
        return false;
    }
    outResult = it->second.result;
    m_ackMap.erase(it);
    return true;
}

bool Px4MavlinkGateway::GetFlightModeInfo(FlightModeInfo &out, uint64_t maxAgeUs) const
{
    std::lock_guard<std::mutex> lk(m_flightModeMtx);
    if (!m_haveFlightModeInfo) {
        return false;
    }
    if (maxAgeUs > 0 && (MonoTimeUs() - m_flightModeInfo.receivedUs) > maxAgeUs) {
        return false;
    }
    out = m_flightModeInfo;
    return true;
}

bool Px4MavlinkGateway::SendCommandLong(const CommandLongRequest &request)
{
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(m_sysid, m_compid, &msg, request.targetSystem, request.targetComponent,
                                  request.command, request.confirmation, request.params[0], request.params[1],
                                  request.params[2], request.params[3], request.params[4], request.params[5],
                                  request.params[6]);
    return QueueMessage(msg);
}

bool Px4MavlinkGateway::BeginCommandLong(const CommandLongRequest &request)
{
    const auto resolved = ResolveCommandTargets(request);
    {
        std::lock_guard<std::mutex> lk(m_ackMtx);
        m_ackMap.erase(resolved.command);
    }
    return SendCommandLong(resolved);
}

bool Px4MavlinkGateway::BeginSetModePx4Main(uint8_t mainMode, uint8_t targetSystem, uint8_t targetComponent)
{
    CommandLongRequest request{};
    request.command = MAV_CMD_DO_SET_MODE;
    request.params[0] = static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
    request.params[1] = static_cast<float>(mainMode);
    request.targetSystem = targetSystem;
    request.targetComponent = targetComponent;
    return BeginCommandLong(request);
}

bool Px4MavlinkGateway::BeginSetModeOffboard(uint8_t targetSystem, uint8_t targetComponent)
{
    return BeginSetModePx4Main(PX4_CUSTOM_MAIN_MODE_OFFBOARD, targetSystem, targetComponent);
}

bool Px4MavlinkGateway::BeginSetModePosition(uint8_t targetSystem, uint8_t targetComponent)
{
    return BeginSetModePx4Main(PX4_CUSTOM_MAIN_MODE_POSCTL, targetSystem, targetComponent);
}

bool Px4MavlinkGateway::BeginArm(bool doArm, uint8_t targetSystem, uint8_t targetComponent)
{
    CommandLongRequest request{};
    request.command = MAV_CMD_COMPONENT_ARM_DISARM;
    request.params[0] = doArm ? 1.0f : 0.0f;
    request.targetSystem = targetSystem;
    request.targetComponent = targetComponent;
    return BeginCommandLong(request);
}

bool Px4MavlinkGateway::BeginEmergencyStop(uint8_t targetSystem, uint8_t targetComponent)
{
    CommandLongRequest request{};
    request.command = MAV_CMD_COMPONENT_ARM_DISARM;
    request.params[1] = 21196.0f;
    request.targetSystem = targetSystem;
    request.targetComponent = targetComponent;
    return BeginCommandLong(request);
}

void Px4MavlinkGateway::SendSetPositionTargetLocalNed(uint32_t timeBootMs, const SetpointLocalNED &sp,
                                                      uint8_t coordinateFrame)
{
    uint16_t typeMask = 0;
    auto ign = [&](int bit) { typeMask |= (1u << bit); };
    if (!std::isfinite(sp.x))
        ign(0);
    if (!std::isfinite(sp.y))
        ign(1);
    if (!std::isfinite(sp.z))
        ign(2);
    if (!std::isfinite(sp.vx))
        ign(3);
    if (!std::isfinite(sp.vy))
        ign(4);
    if (!std::isfinite(sp.vz))
        ign(5);
    if (!std::isfinite(sp.ax))
        ign(6);
    if (!std::isfinite(sp.ay))
        ign(7);
    if (!std::isfinite(sp.az))
        ign(8);
    ign(9);
    if (!std::isfinite(sp.yaw))
        ign(10);
    if (!std::isfinite(sp.yawspeed))
        ign(11);

    mavlink_message_t msg{};
    const uint8_t targetSystem = 1;
    const uint8_t targetComponent = 1;
    mavlink_msg_set_position_target_local_ned_pack(
        m_sysid, m_compid, &msg, timeBootMs, targetSystem, targetComponent, coordinateFrame, typeMask,
        std::isfinite(sp.x) ? sp.x : 0.0f, std::isfinite(sp.y) ? sp.y : 0.0f, std::isfinite(sp.z) ? sp.z : 0.0f,
        std::isfinite(sp.vx) ? sp.vx : 0.0f, std::isfinite(sp.vy) ? sp.vy : 0.0f, std::isfinite(sp.vz) ? sp.vz : 0.0f,
        std::isfinite(sp.ax) ? sp.ax : 0.0f, std::isfinite(sp.ay) ? sp.ay : 0.0f, std::isfinite(sp.az) ? sp.az : 0.0f,
        std::isfinite(sp.yaw) ? sp.yaw : 0.0f, std::isfinite(sp.yawspeed) ? sp.yawspeed : 0.0f);
    QueueMessage(msg);
}

void Px4MavlinkGateway::StartSetpointStreamHz(double hz)
{
    if (hz <= 0.0) {
        hz = 20.0;
    }
    m_streamPeriodUs.store(static_cast<uint64_t>(1e6 / hz), std::memory_order_relaxed);
    m_lastStreamTxUs.store(0, std::memory_order_relaxed);
    m_streaming.store(true);
}

void Px4MavlinkGateway::StopSetpointStream()
{
    m_streaming.store(false);
    m_lastStreamTxUs.store(0, std::memory_order_relaxed);
}

void Px4MavlinkGateway::StepSetpointStream()
{
    if (!m_streaming.load(std::memory_order_relaxed)) {
        return;
    }
    const uint64_t nowUs = MonoTimeUs();
    const uint64_t lastTxUs = m_lastStreamTxUs.load(std::memory_order_relaxed);
    if (lastTxUs != 0 && nowUs - lastTxUs < m_streamPeriodUs.load(std::memory_order_relaxed)) {
        return;
    }
    SetpointLocalNED setpoint;
    {
        std::lock_guard<std::mutex> lock(m_spMtx);
        setpoint = m_spCurrent;
    }
    m_lastStreamTxUs.store(nowUs, std::memory_order_relaxed);
    SendSetPositionTargetLocalNed(MonoTimeMs32(), setpoint, MAV_FRAME_LOCAL_NED);
    StepTx();
}

void Px4MavlinkGateway::UpdateStreamSetpoint(const SetpointLocalNED &spNed)
{
    std::lock_guard<std::mutex> lk(m_spMtx);
    m_spCurrent = spNed;
}

void Px4MavlinkGateway::UpdateStreamPosition(float xN, float yE, float zD, float yawRad)
{
    SetpointLocalNED sp{};
    sp.x = xN;
    sp.y = yE;
    sp.z = zD;
    sp.yaw = yawRad;
    UpdateStreamSetpoint(sp);
}

void Px4MavlinkGateway::SendManualControl(const ManualControlInput &input, uint16_t buttons, uint16_t buttons2,
                                          uint8_t targetSystem)
{
    if (targetSystem == 0) {
        targetSystem = GetTargetSystem();
    }
    auto toAxis = [](float value) -> int16_t {
        const float clamped = std::max(-1.0f, std::min(1.0f, value));
        return static_cast<int16_t>(std::lround(clamped * 1000.0f));
    };
    auto toThrottleAxis = [](float value) -> int16_t {
        const float clamped = std::max(-1.0f, std::min(1.0f, value));
        return static_cast<int16_t>(std::lround((clamped + 1.0f) * 500.0f));
    };

    mavlink_message_t msg{};
    mavlink_msg_manual_control_pack(m_sysid, m_compid, &msg, targetSystem, toAxis(input.pitchNorm),
                                    toAxis(input.rollNorm), toThrottleAxis(input.throttleNorm), toAxis(input.yawNorm),
                                    buttons, buttons2, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    QueueMessage(msg);
}

bool Px4MavlinkGateway::BeginLand(uint8_t targetSystem, uint8_t targetComponent)
{
    CommandLongRequest request{};
    request.command = MAV_CMD_NAV_LAND;
    request.params = {NAN, NAN, NAN, NAN, NAN, NAN, NAN};
    request.targetSystem = targetSystem;
    request.targetComponent = targetComponent;
    return BeginCommandLong(request);
}

void Px4MavlinkGateway::SendOdometry(const OdometryRequest &request)
{
    const OdometryPacketFields fields = BuildOdometryPacketFields(request);
    OdometryTiming timing{};
    if (!PrepareOdometryTiming(request.frameId, timing)) {
        return;
    }

    mavlink_message_t msg{};
    PackOdometryMessage(request, fields, timing.tCamNs / 1000ULL, msg);
    QueueMessage(msg);

    OdometryTimingLog log{};
    log.frameId = request.frameId;
    log.resetCounter = request.resetCounter;
    log.quality = fields.quality;
    log.haveTiming = true;
    log.timing = timing;
    LogOdometryTiming(log);
}

Px4MavlinkGateway::OdometryPacketFields
Px4MavlinkGateway::BuildOdometryPacketFields(const OdometryRequest &request) const
{
    OdometryPacketFields fields{};
    const bool haveVelocity = std::isfinite(request.velocityNed.x) && std::isfinite(request.velocityNed.y) &&
                              std::isfinite(request.velocityNed.z);
    if (request.qualityMode == OdomQualityMode::GOOD) {
        FillCovDiag21(fields.poseCov, {0.04f, 0.04f, 0.36f, 0.03f, 0.03f, 0.03f});
        fields.quality = 100;
    } else if (request.qualityMode == OdomQualityMode::WEAK) {
        FillCovDiag21(fields.poseCov, {2.25f, 2.25f, 9.0f, 0.25f, 0.25f, 0.25f});
        fields.quality = 20;
    } else {
        FillCovDiag21(fields.poseCov, {1e4f, 1e4f, 1e6f, 10.0f, 10.0f, 10.0f});
        fields.quality = 0;
    }
    if (!haveVelocity) {
        FillNanCov21(fields.velocityCov);
        return fields;
    }
    if (request.qualityMode == OdomQualityMode::GOOD) {
        FillCovDiag21(fields.velocityCov, {0.16f, 0.16f, 0.64f, 1.0f, 1.0f, 1.0f});
    } else if (request.qualityMode == OdomQualityMode::WEAK) {
        FillCovDiag21(fields.velocityCov, {4.0f, 4.0f, 16.0f, 1.0f, 1.0f, 1.0f});
    } else {
        FillCovDiag21(fields.velocityCov, {1e2f, 1e2f, 1e4f, 1e2f, 1e2f, 1e2f});
    }
    return fields;
}

bool Px4MavlinkGateway::PrepareOdometryTiming(uint64_t frameId, OdometryTiming &out)
{
    SmartDrone::core::application::FrameTimingRecord timing{};
    const bool haveTiming = LookupFrameTiming(frameId, timing);
    if (!haveTiming || timing.tCamNs == 0) {
        printf("[odom_warn] frame=%llu missing capture timestamp; skipping odometry publish\n",
               static_cast<unsigned long long>(frameId));
        return false;
    }
    out.tCamNs = timing.tCamNs;
    out.tCbNs = timing.tCbNs;
    out.tSlamInNs = timing.tSlamInNs;
    out.tSlamOutNs = timing.tSlamOutNs;
    out.tMavTxNs = ClockMonotonicNs();
    MarkFrameMavTx(frameId, out.tMavTxNs);
    m_lastSentOdomFrameId = frameId;
    return true;
}

void Px4MavlinkGateway::PackOdometryMessage(const OdometryRequest &request, const OdometryPacketFields &fields,
                                            uint64_t captureTimeUs, mavlink_message_t &msg) const
{
    float q[4] = {request.poseNed.qw, request.poseNed.qx, request.poseNed.qy, request.poseNed.qz};
    mavlink_msg_odometry_pack(m_sysid, m_compid, &msg, captureTimeUs, request.mavFrameId, request.childFrameId,
                              request.poseNed.x, request.poseNed.y, request.poseNed.z, q, request.velocityNed.x,
                              request.velocityNed.y, request.velocityNed.z, NAN, NAN, NAN, fields.poseCov,
                              fields.velocityCov, request.resetCounter, fields.estimatorType, fields.quality);
}

void Px4MavlinkGateway::LogOdometryTiming(const OdometryTimingLog &log) const
{
    const double queueLatencyMs = NsDeltaToMs(log.timing.tSlamInNs, log.timing.tCbNs);
    const double slamLatencyMs = NsDeltaToMs(log.timing.tSlamOutNs, log.timing.tCamNs);
    const double sendLatencyMs = NsDeltaToMs(log.timing.tMavTxNs, log.timing.tSlamOutNs);
    const double totalLatencyMs = NsDeltaToMs(log.timing.tMavTxNs, log.timing.tCamNs);
    const bool periodic = (kOdomTsLogEveryNFrames > 0) && ((log.frameId % kOdomTsLogEveryNFrames) == 0);
    const bool abnormal =
        totalLatencyMs > 120.0 || queueLatencyMs > 50.0 || slamLatencyMs > 80.0 || sendLatencyMs > 30.0;
    if (!periodic && !abnormal) {
        return;
    }
    if (m_jsonDiagnostics.load(std::memory_order_relaxed)) {
        printf("{\"tag\":\"odom_ts\",\"frame\":%llu,\"timing\":%d,\"reset\":%u,\"quality\":%d,\"cam_ns\":%llu,"
               "\"queue_ms\":%.3f,\"slam_ms\":%.3f,\"send_ms\":%.3f,\"total_ms\":%.3f}\n",
               static_cast<unsigned long long>(log.frameId), log.haveTiming ? 1 : 0,
               static_cast<unsigned>(log.resetCounter), static_cast<int>(log.quality),
               static_cast<unsigned long long>(log.timing.tCamNs), queueLatencyMs, slamLatencyMs, sendLatencyMs,
               totalLatencyMs);
        return;
    }
    printf("[odom_ts] frame=%llu timing=%d reset=%u quality=%d cam_ns=%llu "
           "queue_ms=%.3f slam_ms=%.3f send_ms=%.3f total_ms=%.3f\n",
           static_cast<unsigned long long>(log.frameId), log.haveTiming ? 1 : 0,
           static_cast<unsigned>(log.resetCounter), static_cast<int>(log.quality),
           static_cast<unsigned long long>(log.timing.tCamNs), queueLatencyMs, slamLatencyMs, sendLatencyMs,
           totalLatencyMs);
}

Px4MavlinkGateway::Pose Px4MavlinkGateway::EnuToNed(const Pose &pEnu)
{
    Pose out{};
    out.x = pEnu.y;
    out.y = pEnu.x;
    out.z = -pEnu.z;

    const float s = 0.7071067811865476f;
    const float qrW = 0.0f, qrX = s, qrY = s, qrZ = 0.0f;

    auto qmul = [](float aw, float ax, float ay, float az, float bw, float bx, float by, float bz, float &ow, float &ox,
                   float &oy, float &oz) {
        ow = aw * bw - ax * bx - ay * by - az * bz;
        ox = aw * bx + ax * bw + ay * bz - az * by;
        oy = aw * by - ax * bz + ay * bw + az * bx;
        oz = aw * bz + ax * by - ay * bx + az * bw;
    };

    auto qconj = [](float w, float x, float y, float z, float &ow, float &ox, float &oy, float &oz) {
        ow = w;
        ox = -x;
        oy = -y;
        oz = -z;
    };

    float tW, tX, tY, tZ;
    qmul(qrW, qrX, qrY, qrZ, pEnu.qw, pEnu.qx, pEnu.qy, pEnu.qz, tW, tX, tY, tZ);
    float qrcW, qrcX, qrcY, qrcZ;
    qconj(qrW, qrX, qrY, qrZ, qrcW, qrcX, qrcY, qrcZ);
    qmul(tW, tX, tY, tZ, qrcW, qrcX, qrcY, qrcZ, out.qw, out.qx, out.qy, out.qz);
    NormalizeQuat(out.qw, out.qx, out.qy, out.qz);
    return out;
}

void Px4MavlinkGateway::NormalizeQuat(float &w, float &x, float &y, float &z)
{
    const float n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n > 1e-9f) {
        w /= n;
        x /= n;
        y /= n;
        z /= n;
    } else {
        w = 1;
        x = y = z = 0;
    }
}

bool Px4MavlinkGateway::LookupFrameTiming(uint64_t frameId, SmartDrone::core::application::FrameTimingRecord &out) const
{
    std::lock_guard<std::mutex> lk(m_frameTimingTrackerMtx);
    return m_frameTimingTracker && m_frameTimingTracker->Lookup(frameId, out);
}

void Px4MavlinkGateway::MarkFrameMavTx(uint64_t frameId, uint64_t tMavTxNs)
{
    std::lock_guard<std::mutex> lk(m_frameTimingTrackerMtx);
    if (m_frameTimingTracker) {
        m_frameTimingTracker->MarkMavTx(frameId, tMavTxNs);
    }
}

const char *Px4MavlinkGateway::MavResultToStr(uint8_t r)
{
    switch (r) {
    case MAV_RESULT_ACCEPTED:
        return "ACCEPTED";
    case MAV_RESULT_TEMPORARILY_REJECTED:
        return "TEMP_REJECTED";
    case MAV_RESULT_DENIED:
        return "DENIED";
    case MAV_RESULT_UNSUPPORTED:
        return "UNSUPPORTED";
    case MAV_RESULT_FAILED:
        return "FAILED";
    case MAV_RESULT_IN_PROGRESS:
        return "IN_PROGRESS";
    default:
        return "UNKNOWN";
    }
}

Px4MavlinkGateway::CommandLongRequest Px4MavlinkGateway::ResolveCommandTargets(CommandLongRequest request) const
{
    if (request.targetSystem == 0) {
        request.targetSystem = GetTargetSystem();
    }
    if (request.targetComponent == 0) {
        request.targetComponent = GetTargetComponent();
    }
    return request;
}

bool Px4MavlinkGateway::SendMessageIntervalRequest(uint32_t messageId, float intervalUs, uint8_t targetSystem,
                                                   uint8_t targetComponent)
{
    CommandLongRequest request{};
    request.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    request.params[0] = static_cast<float>(messageId);
    request.params[1] = intervalUs;
    request.targetSystem = targetSystem;
    request.targetComponent = targetComponent;
    return SendCommandLong(request);
}

bool Px4MavlinkGateway::QueueMessage(const mavlink_message_t &msg)
{
    std::lock_guard<std::mutex> txLock(m_txMtx);
    if (m_txQueue.size() >= kMaxQueuedTxMessages) {
        printf("[mav] tx queue full, dropping msgid=%u\n", unsigned(msg.msgid));
        return false;
    }
    std::vector<uint8_t> buffer(MAVLINK_MAX_PACKET_LEN);
    const uint16_t len = mavlink_msg_to_send_buffer(buffer.data(), &msg);
    buffer.resize(len);
    m_txQueue.push_back(std::move(buffer));
    return true;
}

void Px4MavlinkGateway::HandleMavlinkMessage(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        HandleHeartbeat(msg);
        return;
    }
    if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
        HandleLocalPositionNed(msg);
        return;
    }
    if (msg.msgid == MAVLINK_MSG_ID_DISTANCE_SENSOR) {
        HandleDistanceSensor(msg);
        return;
    }
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        HandleCommandAck(msg);
        return;
    }
    if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        HandleStatusText(msg);
        return;
    }
}

void Px4MavlinkGateway::HandleHeartbeat(const mavlink_message_t &msg)
{
    mavlink_heartbeat_t heartbeat{};
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    if (heartbeat.autopilot == MAV_AUTOPILOT_INVALID) {
        return;
    }
    FlightModeInfo modeInfo{};
    modeInfo.baseMode = heartbeat.base_mode;
    modeInfo.customMode = heartbeat.custom_mode;
    modeInfo.mainMode = static_cast<uint8_t>((heartbeat.custom_mode >> 16) & 0xFFu);
    modeInfo.subMode = static_cast<uint8_t>((heartbeat.custom_mode >> 24) & 0xFFu);
    modeInfo.armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
    modeInfo.receivedUs = MonoTimeUs();
    {
        std::lock_guard<std::mutex> lock(m_flightModeMtx);
        m_flightModeInfo = modeInfo;
        m_haveFlightModeInfo = true;
    }
    m_havePx4Heartbeat.store(true);
    m_px4Sysid.store(msg.sysid);
    m_px4Compid.store(msg.compid);
    MaybeRequestLocalPositionNedStream(msg.sysid, msg.compid);
    MaybeRequestDistanceSensorStream(msg.sysid, msg.compid);
}

void Px4MavlinkGateway::HandleLocalPositionNed(const mavlink_message_t &msg)
{
    mavlink_local_position_ned_t localPosition{};
    mavlink_msg_local_position_ned_decode(&msg, &localPosition);
    LocalPositionNed sample{};
    sample.x = localPosition.x;
    sample.y = localPosition.y;
    sample.z = localPosition.z;
    sample.vx = localPosition.vx;
    sample.vy = localPosition.vy;
    sample.vz = localPosition.vz;
    sample.timeBootMs = localPosition.time_boot_ms;
    sample.receivedUs = MonoTimeUs();
    std::lock_guard<std::mutex> lock(m_localPosMtx);
    m_localPosNed = sample;
    m_haveLocalPosNed = true;
}

void Px4MavlinkGateway::HandleDistanceSensor(const mavlink_message_t &msg)
{
    mavlink_distance_sensor_t distance{};
    mavlink_msg_distance_sensor_decode(&msg, &distance);
    if (distance.orientation != MAV_SENSOR_ROTATION_PITCH_270) {
        return;
    }
    DownwardDistanceSensor sample{};
    sample.currentDistance = 0.01f * static_cast<float>(distance.current_distance);
    sample.minDistance = 0.01f * static_cast<float>(distance.min_distance);
    sample.maxDistance = 0.01f * static_cast<float>(distance.max_distance);
    sample.signalQuality = distance.signal_quality;
    sample.receivedUs = MonoTimeUs();
    std::lock_guard<std::mutex> lock(m_distanceSensorMtx);
    m_downwardDistanceSensor = sample;
    m_haveDownwardDistanceSensor = true;
}

void Px4MavlinkGateway::HandleCommandAck(const mavlink_message_t &msg)
{
    mavlink_command_ack_t ack{};
    mavlink_msg_command_ack_decode(&msg, &ack);
    AckInfo info;
    info.result = ack.result;
    info.progress = ack.progress;
    info.resultParam2 = ack.result_param2;
    info.t = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(m_ackMtx);
        m_ackMap[ack.command] = info;
    }
    printf("[ACK] sys=%d, comp=%d cmd=%u result= %d(%s) progress=%d param2=%d\n", int(msg.sysid), int(msg.compid),
           ack.command, int(ack.result), MavResultToStr(ack.result), int(ack.progress), ack.result_param2);
}

void Px4MavlinkGateway::HandleStatusText(const mavlink_message_t &msg)
{
    mavlink_statustext_t statusText{};
    mavlink_msg_statustext_decode(&msg, &statusText);
    char text[sizeof(statusText.text) + 1];
    std::memcpy(text, statusText.text, sizeof(statusText.text));
    text[sizeof(statusText.text)] = '\0';
    printf("[STATUSTEXT] sev=%d %s\n", int(statusText.severity), text);
}

void Px4MavlinkGateway::MaybeRequestLocalPositionNedStream(uint8_t targetSystem, uint8_t targetComponent)
{
    bool expected = false;
    if (!m_localPosStreamRequested.compare_exchange_strong(expected, true)) {
        return;
    }
    constexpr float kIntervalUs = 50000.0f;
    SendMessageIntervalRequest(MAVLINK_MSG_ID_LOCAL_POSITION_NED, kIntervalUs, targetSystem, targetComponent);
    printf("[mav] requested LOCAL_POSITION_NED @20Hz from sys=%d comp=%d\n", int(targetSystem), int(targetComponent));
}

void Px4MavlinkGateway::MaybeRequestDistanceSensorStream(uint8_t targetSystem, uint8_t targetComponent)
{
    bool expected = false;
    if (!m_distanceSensorStreamRequested.compare_exchange_strong(expected, true)) {
        return;
    }
    constexpr float kIntervalUs = 50000.0f;
    SendMessageIntervalRequest(MAVLINK_MSG_ID_DISTANCE_SENSOR, kIntervalUs, targetSystem, targetComponent);
    printf("[mav] requested DISTANCE_SENSOR @20Hz from sys=%d comp=%d\n", int(targetSystem), int(targetComponent));
}
