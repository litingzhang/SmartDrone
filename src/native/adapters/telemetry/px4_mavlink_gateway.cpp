#include "adapters/telemetry/px4_mavlink_gateway.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <time.h>

#include "common/time_utils.h"

namespace {

constexpr uint64_t ODOM_TS_LOG_EVERY_N_FRAMES = 30;

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

double NsAbsoluteDeltaToMs(uint64_t lhsNs, uint64_t rhsNs)
{
    if (lhsNs == 0 || rhsNs == 0) {
        return -1.0;
    }
    const uint64_t earlierNs = std::min(lhsNs, rhsNs);
    const uint64_t laterNs = std::max(lhsNs, rhsNs);
    return static_cast<double>(laterNs - earlierNs) * 1e-6;
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
    auto setpoint = std::make_shared<const SetpointLocalNED>(SetpointLocalNED{});
    auto ackSnapshot = std::make_shared<const AckSnapshot>(AckSnapshot{});
    std::atomic_store_explicit(&m_spCurrent, std::move(setpoint),
                               std::memory_order_release);
    std::atomic_store_explicit(&m_ackSnapshot, std::move(ackSnapshot),
                               std::memory_order_release);
}

Px4MavlinkGateway::~Px4MavlinkGateway()
{
    StopSetpointStream();
}

void Px4MavlinkGateway::SetJsonDiagnostics(bool enabled)
{
    m_jsonDiagnostics.store(enabled, std::memory_order_relaxed);
}

void Px4MavlinkGateway::SetFrameTimingTracker(SmartDrone::Core::Ports::IFrameTimingTracker *tracker)
{
    m_frameTimingTracker.store(tracker, std::memory_order_release);
}

void Px4MavlinkGateway::SetMeasurementClock(
    std::shared_ptr<SmartDrone::Core::Ports::IMeasurementClock> clock)
{
    std::atomic_store_explicit(&m_measurementClock, std::move(clock),
                               std::memory_order_release);
}

int Px4MavlinkGateway::PollRxOnce()
{
    StepTx();
    const int pollResult = m_transport.PollReadable();
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
    MaybeSendHeartbeat();
    while (m_txActive || LoadNextTxMessage()) {
        const auto &front = *m_txActive;
        const ssize_t written = m_transport.WriteSome(
            front.bytes.data() + m_txOffset, front.length - m_txOffset);
        if (written > 0) {
            m_txOffset += static_cast<std::size_t>(written);
            if (m_txOffset >= front.length) {
                ReleaseActiveTxMessage();
            }
            continue;
        }
        if (written == 0) {
            return;
        }
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            return;
        }
        std::cerr << "[mav] write failed len=" << front.length
                  << " errno=" << errno << "\n";
        ReleaseActiveTxMessage();
        return;
    }
}

uint8_t Px4MavlinkGateway::GetTargetSystem() const
{
    return static_cast<uint8_t>(
        m_px4Target.load(std::memory_order_acquire) >> 8U);
}

uint8_t Px4MavlinkGateway::GetTargetComponent() const
{
    return static_cast<uint8_t>(
        m_px4Target.load(std::memory_order_acquire) & 0xFFU);
}

bool Px4MavlinkGateway::GetLocalPositionNed(LocalPositionNed &out, uint64_t maxAgeUs) const
{
    auto snapshot = std::atomic_load_explicit(&m_localPosNed,
                                              std::memory_order_acquire);
    if (!snapshot) {
        return false;
    }
    if (maxAgeUs > 0 && (MonoTimeUs() - snapshot->receivedUs) > maxAgeUs) {
        return false;
    }
    out = *snapshot;
    return true;
}

bool Px4MavlinkGateway::GetDownwardDistanceSensor(DownwardDistanceSensor &out, uint64_t maxAgeUs) const
{
    auto snapshot = std::atomic_load_explicit(&m_downwardDistanceSensor,
                                              std::memory_order_acquire);
    if (!snapshot) {
        return false;
    }
    if (maxAgeUs > 0 && (MonoTimeUs() - snapshot->receivedUs) > maxAgeUs) {
        return false;
    }
    out = *snapshot;
    return true;
}

bool Px4MavlinkGateway::TryConsumeCommandAck(uint16_t command, uint8_t &outResult)
{
    auto current = std::atomic_load_explicit(&m_ackSnapshot,
                                             std::memory_order_acquire);
    while (current) {
        AckSnapshot next = *current;
        auto it = std::find_if(next.records.begin(), next.records.end(),
                               [command](const AckInfo &info) {
                                   return info.command == command;
                               });
        if (it == next.records.end()) {
            return false;
        }
        outResult = it->result;
        next.records.erase(it);
        auto updated = std::make_shared<const AckSnapshot>(std::move(next));
        if (std::atomic_compare_exchange_weak_explicit(
                &m_ackSnapshot, &current, updated,
                std::memory_order_acq_rel, std::memory_order_acquire)) {
            return true;
        }
    }
    return false;
}

bool Px4MavlinkGateway::GetFlightModeInfo(FlightModeInfo &out, uint64_t maxAgeUs) const
{
    auto snapshot = std::atomic_load_explicit(&m_flightModeInfo,
                                              std::memory_order_acquire);
    if (!snapshot) {
        return false;
    }
    if (maxAgeUs > 0 && (MonoTimeUs() - snapshot->receivedUs) > maxAgeUs) {
        return false;
    }
    out = *snapshot;
    return true;
}

bool Px4MavlinkGateway::GetExtendedSystemState(ExtendedSystemState &out,
                                                uint64_t maxAgeUs) const
{
    auto snapshot = std::atomic_load_explicit(&m_extendedSystemState,
                                              std::memory_order_acquire);
    if (!snapshot) {
        return false;
    }
    if (maxAgeUs > 0 && (MonoTimeUs() - snapshot->receivedUs) > maxAgeUs) {
        return false;
    }
    out = *snapshot;
    return true;
}

bool Px4MavlinkGateway::GetEstimatorStatus(EstimatorStatus &out,
                                            uint64_t maxAgeUs) const
{
    auto snapshot = std::atomic_load_explicit(&m_estimatorStatus,
                                              std::memory_order_acquire);
    if (!snapshot) {
        return false;
    }
    if (maxAgeUs > 0 && (MonoTimeUs() - snapshot->receivedUs) > maxAgeUs) {
        return false;
    }
    out = *snapshot;
    return true;
}

bool Px4MavlinkGateway::SendCommandLong(const CommandLongRequest &request)
{
    CommandLongRequest resolved{};
    if (!ResolveCommandTargets(request, resolved)) {
        return false;
    }
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(m_sysid, m_compid, &msg,
                                  resolved.targetSystem,
                                  resolved.targetComponent, resolved.command,
                                  resolved.confirmation, resolved.params[0],
                                  resolved.params[1], resolved.params[2],
                                  resolved.params[3], resolved.params[4],
                                  resolved.params[5], resolved.params[6]);
    return QueueMessage(msg);
}

bool Px4MavlinkGateway::BeginCommandLong(const CommandLongRequest &request)
{
    ClearCommandAck(request.command);
    return SendCommandLong(request);
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
    if (!HavePx4Target()) {
        return;
    }
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
    const uint8_t targetSystem = GetTargetSystem();
    const uint8_t targetComponent = GetTargetComponent();
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
    if (!m_streaming.load(std::memory_order_relaxed) || !HavePx4Target()) {
        return;
    }
    const uint64_t nowUs = MonoTimeUs();
    const uint64_t lastTxUs = m_lastStreamTxUs.load(std::memory_order_relaxed);
    if (lastTxUs != 0 && nowUs - lastTxUs < m_streamPeriodUs.load(std::memory_order_relaxed)) {
        return;
    }
    auto setpoint = std::atomic_load_explicit(&m_spCurrent,
                                              std::memory_order_acquire);
    if (!setpoint) {
        return;
    }
    m_lastStreamTxUs.store(nowUs, std::memory_order_relaxed);
    SendSetPositionTargetLocalNed(MonoTimeMs32(), *setpoint,
                                  MAV_FRAME_LOCAL_NED);
}

void Px4MavlinkGateway::UpdateStreamSetpoint(const SetpointLocalNED &spNed)
{
    auto snapshot = std::make_shared<const SetpointLocalNED>(spNed);
    std::atomic_store_explicit(&m_spCurrent, std::move(snapshot),
                               std::memory_order_release);
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
        if (!HavePx4Target()) {
            return;
        }
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
    if (!PrepareOdometryTiming(request, timing)) {
        return;
    }

    mavlink_message_t msg{};
    PackOdometryMessage(request, fields, timing.tCamNs / 1000ULL, msg);
    if (!QueueMessage(msg)) {
        return;
    }
    MarkFrameMavTx(request.frameId, timing.tMavTxNs);

    OdometryTimingLog log{};
    log.frameId = request.frameId;
    log.resetCounter = request.resetCounter;
    log.quality = fields.quality;
    log.haveTiming = timing.complete;
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
        fields.quality = -1;
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

bool Px4MavlinkGateway::PrepareOdometryTiming(
    const OdometryRequest &request, OdometryTiming &out)
{
    SmartDrone::Core::Ports::FrameTimingRecord timing{};
    const bool timingFound = LookupFrameTiming(request.frameId, timing);
    out.tCamNs = request.measurementTimestampNs != 0
                     ? request.measurementTimestampNs
                     : timing.tCamNs;
    if (out.tCamNs == 0) {
        out.tCamNs = MeasurementNowNs();
    }
    if (out.tCamNs == 0) {
        return false;
    }
    out.tCaptureMonotonicNs = request.captureMonotonicNs != 0
                                  ? request.captureMonotonicNs
                                  : timing.tCaptureMonotonicNs;
    if (out.tCaptureMonotonicNs == 0) {
        out.tCaptureMonotonicNs = ClockMonotonicNs();
    }
    out.tLeftArrivalNs = timing.tLeftArrivalNs;
    out.tRightArrivalNs = timing.tRightArrivalNs;
    out.tPairReadyNs = timing.tPairReadyNs;
    out.tSlamInNs = timing.tSlamInNs;
    out.tSlamOutNs = timing.tSlamOutNs;
    out.tMavTxNs = ClockMonotonicNs();
    out.simAgeMs = MeasurementAgeMs(out.tCamNs);
    out.complete = timingFound && IsOdometryTimingComplete(out);
    return true;
}

bool Px4MavlinkGateway::IsOdometryTimingComplete(
    const OdometryTiming &timing)
{
    const bool pairReadyMatchesArrivals =
        timing.tPairReadyNs ==
        std::max(timing.tLeftArrivalNs, timing.tRightArrivalNs);
    return timing.tCaptureMonotonicNs > 0 && timing.tLeftArrivalNs > 0 &&
           timing.tRightArrivalNs > 0 && timing.tPairReadyNs > 0 &&
           timing.tSlamInNs > 0 && timing.tSlamOutNs > 0 &&
           timing.tMavTxNs > 0 && pairReadyMatchesArrivals &&
           timing.tCaptureMonotonicNs <= timing.tPairReadyNs &&
           timing.tPairReadyNs <= timing.tSlamInNs &&
           timing.tSlamInNs <= timing.tSlamOutNs &&
           timing.tSlamOutNs <= timing.tMavTxNs;
}

Px4MavlinkGateway::OdometryTimingMetrics
Px4MavlinkGateway::BuildOdometryTimingMetrics(
    const OdometryTiming &timing)
{
    OdometryTimingMetrics metrics;
    metrics.eyeSkewMs = NsAbsoluteDeltaToMs(
        timing.tLeftArrivalNs, timing.tRightArrivalNs);
    metrics.renderTransportMs = NsDeltaToMs(
        timing.tPairReadyNs, timing.tCaptureMonotonicNs);
    metrics.queueMs = NsDeltaToMs(timing.tSlamInNs, timing.tPairReadyNs);
    metrics.processingMs = NsDeltaToMs(timing.tSlamOutNs, timing.tSlamInNs);
    metrics.sendMs = NsDeltaToMs(timing.tMavTxNs, timing.tSlamOutNs);
    metrics.pairToTxMs = NsDeltaToMs(timing.tMavTxNs, timing.tPairReadyNs);
    metrics.wallTotalMs = NsDeltaToMs(
        timing.tMavTxNs, timing.tCaptureMonotonicNs);
    return metrics;
}

double Px4MavlinkGateway::MeasurementAgeMs(uint64_t measurementNs) const
{
    auto clock = std::atomic_load_explicit(&m_measurementClock,
                                           std::memory_order_acquire);
    if (!clock || !clock->Valid()) {
        return -1.0;
    }
    return NsDeltaToMs(clock->NowNs(), measurementNs);
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
    const OdometryTimingMetrics metrics =
        BuildOdometryTimingMetrics(log.timing);
    const bool jsonDiagnostics =
        m_jsonDiagnostics.load(std::memory_order_relaxed);
    const bool periodic = jsonDiagnostics ||
                          (log.frameId != 0 &&
                           (ODOM_TS_LOG_EVERY_N_FRAMES > 0) &&
                           ((log.frameId % ODOM_TS_LOG_EVERY_N_FRAMES) == 0));
    const bool abnormal =
        metrics.wallTotalMs > 120.0 || metrics.queueMs > 50.0 ||
        metrics.processingMs > 80.0 || metrics.sendMs > 30.0;
    if (!periodic && !abnormal) {
        return;
    }
    if (jsonDiagnostics) {
        LogOdometryTimingJson(log, metrics);
        return;
    }
    LogOdometryTimingText(log, metrics);
}

void Px4MavlinkGateway::LogOdometryTimingJson(
    const OdometryTimingLog &log,
    const OdometryTimingMetrics &metrics) const
{
    std::ostringstream line;
    line << std::fixed << std::setprecision(3)
         << "{\"tag\":\"odom_ts\",\"frame\":" << log.frameId
         << ",\"timing\":" << (log.haveTiming ? 1 : 0)
         << ",\"reset\":" << static_cast<unsigned>(log.resetCounter)
         << ",\"quality\":" << static_cast<int>(log.quality)
         << ",\"cam_ns\":" << log.timing.tCamNs
         << ",\"capture_mono_ns\":" << log.timing.tCaptureMonotonicNs
         << ",\"eye_skew_ms\":" << metrics.eyeSkewMs
         << ",\"render_transport_ms\":" << metrics.renderTransportMs
         << ",\"queue_ms\":" << metrics.queueMs
         << ",\"processing_ms\":" << metrics.processingMs
         << ",\"slam_ms\":" << metrics.processingMs
         << ",\"send_ms\":" << metrics.sendMs
         << ",\"pair_to_tx_ms\":" << metrics.pairToTxMs
         << ",\"wall_total_ms\":" << metrics.wallTotalMs
         << ",\"total_ms\":" << metrics.wallTotalMs
         << ",\"sim_age_ms\":" << log.timing.simAgeMs << "}";
    std::cerr << line.str() << "\n";
}

void Px4MavlinkGateway::LogOdometryTimingText(
    const OdometryTimingLog &log,
    const OdometryTimingMetrics &metrics) const
{
    std::ostringstream line;
    line << std::fixed << std::setprecision(3)
         << "[odom_ts] frame=" << log.frameId
         << " timing=" << (log.haveTiming ? 1 : 0)
         << " reset=" << static_cast<unsigned>(log.resetCounter)
         << " quality=" << static_cast<int>(log.quality)
         << " cam_ns=" << log.timing.tCamNs
         << " eye_skew_ms=" << metrics.eyeSkewMs
         << " render_transport_ms=" << metrics.renderTransportMs
         << " queue_ms=" << metrics.queueMs
         << " processing_ms=" << metrics.processingMs
         << " send_ms=" << metrics.sendMs
         << " pair_to_tx_ms=" << metrics.pairToTxMs
         << " wall_total_ms=" << metrics.wallTotalMs
         << " total_ms=" << metrics.wallTotalMs
         << " sim_age_ms=" << log.timing.simAgeMs;
    std::cerr << line.str() << "\n";
}

Px4MavlinkGateway::Pose Px4MavlinkGateway::EnuToNed(const Pose &pEnu)
{
    Pose out{};
    out.x = pEnu.y;
    out.y = pEnu.x;
    out.z = -pEnu.z;

    const float s = 0.7071067811865476f;
    struct Quaternion {
        float w{1.0f};
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};
    };

    const Quaternion rotation{0.0f, s, s, 0.0f};
    const Quaternion input{pEnu.qw, pEnu.qx, pEnu.qy, pEnu.qz};
    auto multiply = [](const Quaternion &lhs, const Quaternion &rhs) {
        return Quaternion{
            lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
            lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
            lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w};
    };
    auto conjugate = [](const Quaternion &value) {
        return Quaternion{value.w, -value.x, -value.y, -value.z};
    };

    const Quaternion converted =
        multiply(multiply(rotation, input), conjugate(rotation));
    out.qw = converted.w;
    out.qx = converted.x;
    out.qy = converted.y;
    out.qz = converted.z;
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

bool Px4MavlinkGateway::LookupFrameTiming(uint64_t frameId, SmartDrone::Core::Ports::FrameTimingRecord &out) const
{
    auto *tracker = m_frameTimingTracker.load(std::memory_order_acquire);
    return tracker != nullptr && tracker->Lookup(frameId, out);
}

void Px4MavlinkGateway::MarkFrameMavTx(uint64_t frameId, uint64_t tMavTxNs)
{
    auto *tracker = m_frameTimingTracker.load(std::memory_order_acquire);
    if (tracker != nullptr) {
        tracker->MarkMavTx(frameId, tMavTxNs);
    }
}

uint64_t Px4MavlinkGateway::MeasurementNowNs() const
{
    auto clock = std::atomic_load_explicit(&m_measurementClock,
                                           std::memory_order_acquire);
    if (clock && clock->Valid()) {
        return clock->NowNs();
    }
    return MonoTimeUs() * 1000ULL;
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

bool Px4MavlinkGateway::HavePx4Target() const
{
    return m_px4Target.load(std::memory_order_acquire) != 0;
}

bool Px4MavlinkGateway::ResolveCommandTargets(
    const CommandLongRequest &request, CommandLongRequest &out) const
{
    out = request;
    if (out.targetSystem != 0) {
        return true;
    }
    if (!HavePx4Target()) {
        return false;
    }
    out.targetSystem = GetTargetSystem();
    if (out.targetComponent == 0) {
        out.targetComponent = GetTargetComponent();
    }
    return true;
}

void Px4MavlinkGateway::ClearCommandAck(uint16_t command)
{
    uint8_t ignored = 0;
    (void)TryConsumeCommandAck(command, ignored);
}

void Px4MavlinkGateway::StoreCommandAck(const AckInfo &info)
{
    auto current = std::atomic_load_explicit(&m_ackSnapshot,
                                             std::memory_order_acquire);
    while (current) {
        AckSnapshot next = *current;
        auto it = std::find_if(next.records.begin(), next.records.end(),
                               [&info](const AckInfo &record) {
                                   return record.command == info.command;
                               });
        if (it == next.records.end()) {
            next.records.push_back(info);
        } else {
            *it = info;
        }
        auto updated = std::make_shared<const AckSnapshot>(std::move(next));
        if (std::atomic_compare_exchange_weak_explicit(
                &m_ackSnapshot, &current, updated,
                std::memory_order_acq_rel, std::memory_order_acquire)) {
            return;
        }
    }
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

void Px4MavlinkGateway::MaybeSendHeartbeat()
{
    const uint64_t nowUs = MonoTimeUs();
    const uint64_t lastUs =
        m_lastHeartbeatTxUs.load(std::memory_order_relaxed);
    if (lastUs != 0 && nowUs - lastUs < 1000000ULL) {
        return;
    }
    m_lastHeartbeatTxUs.store(nowUs, std::memory_order_relaxed);
    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack(m_sysid, m_compid, &msg,
                               MAV_TYPE_ONBOARD_CONTROLLER,
                               MAV_AUTOPILOT_INVALID, 0, 0,
                               MAV_STATE_ACTIVE);
    QueueMessage(msg);
}

bool Px4MavlinkGateway::LoadNextTxMessage()
{
    auto tail = m_txTail.load(std::memory_order_relaxed);
    if (tail == m_txHead.load(std::memory_order_acquire)) {
        return false;
    }
    const auto index = static_cast<std::size_t>(tail % TX_QUEUE_CAPACITY);
    if (!m_txReady[index].load(std::memory_order_acquire)) {
        return false;
    }
    m_txActive = std::move(m_txSlots[index]);
    m_txSlots[index].reset();
    m_txReady[index].store(false, std::memory_order_release);
    m_txTail.store(tail + 1, std::memory_order_release);
    m_txOffset = 0;
    return m_txActive != nullptr;
}

void Px4MavlinkGateway::ReleaseActiveTxMessage()
{
    m_txActive.reset();
    m_txOffset = 0;
}

bool Px4MavlinkGateway::PushTxMessage(std::shared_ptr<const TxMessage> message)
{
    auto head = m_txHead.load(std::memory_order_relaxed);
    while (true) {
        const auto tail = m_txTail.load(std::memory_order_acquire);
        if (head - tail >= TX_QUEUE_CAPACITY) {
            return false;
        }
        if (m_txHead.compare_exchange_weak(head, head + 1,
                                           std::memory_order_acq_rel,
                                           std::memory_order_relaxed)) {
            break;
        }
    }
    const auto index = static_cast<std::size_t>(head % TX_QUEUE_CAPACITY);
    m_txSlots[index] = std::move(message);
    m_txReady[index].store(true, std::memory_order_release);
    return true;
}

bool Px4MavlinkGateway::QueueMessage(const mavlink_message_t &msg)
{
    auto message = std::make_shared<TxMessage>();
    const uint16_t len = mavlink_msg_to_send_buffer(message->bytes.data(), &msg);
    message->length = len;
    if (!PushTxMessage(std::move(message))) {
        std::cerr << "[mav] tx queue full, dropping msgid="
                  << unsigned(msg.msgid) << "\n";
        return false;
    }
    return true;
}
