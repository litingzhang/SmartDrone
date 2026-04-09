#include "adapters/telemetry/px4_mavlink_gateway.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

#include <time.h>
#include <unistd.h>

#include "common/thread_launch.h"
#include "common/time_utils.h"

namespace {

constexpr uint64_t kOdomTsLogEveryNFrames = 30;

void FillCovDiag21(float cov[21], float varX, float varY, float varZ, float varRoll, float varPitch, float varYaw,
                   bool fillOffdiagZero = true)
{
    if (fillOffdiagZero) {
        for (int i = 0; i < 21; i++) {
            cov[i] = 0.0f;
        }
    }
    cov[0] = varX;
    cov[6] = varY;
    cov[11] = varZ;
    cov[15] = varRoll;
    cov[18] = varPitch;
    cov[20] = varYaw;
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
    StopRx();
    StopSetpointStream();
}

void Px4MavlinkGateway::SetFrameTimingTracker(smartdrone::core::application::FrameTimingTracker *tracker)
{
    std::lock_guard<std::mutex> lk(m_frameTimingTrackerMtx);
    m_frameTimingTracker = tracker;
}

void Px4MavlinkGateway::StartRx()
{
    StopRx();
    {
        std::lock_guard<std::mutex> lk(m_timesyncMtx);
        m_havePendingTimesync = false;
        m_pendingTimesyncTs1Ns = 0;
        m_pendingTimesyncSentNs = 0;
        m_pendingTimesyncTargetSystem = 0;
        m_pendingTimesyncTargetComponent = 0;
        m_timesyncOffsetEstimateNs = 0;
        m_timesyncLastRttUs = 0;
        m_timesyncSampleCount = 0;
        m_timesyncInboundRequestCount = 0;
        m_timesyncInboundResponseCount = 0;
        m_lastTimesyncActivityUs = 0;
        m_lastTimesyncLogUs = 0;
    }
    m_havePx4Heartbeat.store(false);
    m_rxRunning.store(true);
    m_timesyncRunning.store(true);
    m_rxThread = SMARTDRONE_START_THREAD(smartdrone::common::ThreadRole::MavlinkRx, "Px4MavlinkGateway",
                                         [this]() { this->RxLoop(); });
    m_timesyncThread = SMARTDRONE_START_THREAD(smartdrone::common::ThreadRole::MavlinkTimesync, "Px4MavlinkGateway",
                                               [this]() { this->TimesyncLoop(); });
}

void Px4MavlinkGateway::StopRx()
{
    m_rxRunning.store(false);
    m_timesyncRunning.store(false);
    if (m_rxThread.joinable())
        m_rxThread.join();
    if (m_timesyncThread.joinable())
        m_timesyncThread.join();
}

uint8_t Px4MavlinkGateway::GetTargetSystem() const { return m_px4Sysid.load(); }

uint8_t Px4MavlinkGateway::GetTargetComponent() const { return m_px4Compid.load(); }

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

bool Px4MavlinkGateway::WaitCommandAck(uint16_t command, int timeoutMs, uint8_t &outResult)
{
    std::unique_lock<std::mutex> lk(m_ackMtx);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    auto pred = [&]() {
        auto it = m_ackMap.find(command);
        return it != m_ackMap.end();
    };
    if (!m_ackCv.wait_until(lk, deadline, pred))
        return false;
    outResult = m_ackMap[command].result;
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

void Px4MavlinkGateway::SendCommandLong(uint16_t command, float p1, float p2, float p3, float p4, float p5, float p6,
                                        float p7, uint8_t targetSystem, uint8_t targetComponent, uint8_t confirmation)
{
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(m_sysid, m_compid, &msg, targetSystem, targetComponent, command, confirmation, p1, p2,
                                  p3, p4, p5, p6, p7);
    WriteMessage(msg);
}

bool Px4MavlinkGateway::SendCommandLongAndWaitAck(uint16_t command, float p1, float p2, float p3, float p4, float p5,
                                                  float p6, float p7, int timeoutMs, uint8_t targetSystem,
                                                  uint8_t targetComponent, uint8_t *outResult)
{
    if (targetSystem == 0)
        targetSystem = GetTargetSystem();
    if (targetComponent == 0)
        targetComponent = GetTargetComponent();
    {
        std::lock_guard<std::mutex> lk(m_ackMtx);
        m_ackMap.erase(command);
    }
    SendCommandLong(command, p1, p2, p3, p4, p5, p6, p7, targetSystem, targetComponent);
    uint8_t res = 255;
    bool ok = WaitCommandAck(command, timeoutMs, res);
    if (outResult)
        *outResult = res;
    if (!ok) {
        printf("[ACK] TIMEOUT cmd=%u (target sys=%d comp=%d)\n", command, int(targetSystem), int(targetComponent));
    } else {
        printf("[ACK] cmd=%u result=%s\n", command, MavResultToStr(res));
    }
    return ok;
}

bool Px4MavlinkGateway::SetModePx4Main(uint8_t mainMode, int ackTimeoutMs, uint8_t targetSystem,
                                       uint8_t targetComponent)
{
    // PX4 handles MAV_CMD_DO_SET_MODE as:
    // param1 = base mode flags, param2 = PX4 custom main mode, param3 = custom sub mode.
    // Do not pack main/sub mode into a 32-bit heartbeat custom_mode here.
    const float baseMode = static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
    const float customMainMode = static_cast<float>(mainMode);
    const float customSubMode = 0.0f;
    uint8_t res = 255;
    bool got = SendCommandLongAndWaitAck(MAV_CMD_DO_SET_MODE, baseMode, customMainMode, customSubMode, 0, 0, 0, 0,
                                         ackTimeoutMs, targetSystem, targetComponent, &res);
    return got && (res == MAV_RESULT_ACCEPTED);
}

bool Px4MavlinkGateway::SetModeOffboard(int ackTimeoutMs, uint8_t targetSystem, uint8_t targetComponent)
{
    return SetModePx4Main(PX4_CUSTOM_MAIN_MODE_OFFBOARD, ackTimeoutMs, targetSystem, targetComponent);
}

bool Px4MavlinkGateway::SetModePosition(int ackTimeoutMs, uint8_t targetSystem, uint8_t targetComponent)
{
    return SetModePx4Main(PX4_CUSTOM_MAIN_MODE_POSCTL, ackTimeoutMs, targetSystem, targetComponent);
}

bool Px4MavlinkGateway::Arm(bool doArm, int ackTimeoutMs, uint8_t targetSystem, uint8_t targetComponent)
{
    uint8_t res = 255;
    bool got = SendCommandLongAndWaitAck(MAV_CMD_COMPONENT_ARM_DISARM, doArm ? 1.0f : 0.0f, 0, 0, 0, 0, 0, 0,
                                         ackTimeoutMs, targetSystem, targetComponent, &res);
    return got && (res == MAV_RESULT_ACCEPTED);
}

bool Px4MavlinkGateway::EmergencyStop(int ackTimeoutMs, uint8_t targetSystem, uint8_t targetComponent)
{
    uint8_t res = 255;
    bool got = SendCommandLongAndWaitAck(MAV_CMD_COMPONENT_ARM_DISARM, 0.0f, 21196.0f, 0, 0, 0, 0, 0, ackTimeoutMs,
                                         targetSystem, targetComponent, &res);
    return got && (res == MAV_RESULT_ACCEPTED);
}

bool Px4MavlinkGateway::StartOffboardAndArm(double, double, int warmupMs, int ackTimeoutMs, uint8_t targetSystem,
                                            uint8_t targetComponent)
{
    usleep(static_cast<useconds_t>(warmupMs) * 1000);
    if (!SetModeOffboard(ackTimeoutMs, targetSystem, targetComponent)) {
        printf(
            "[px4] OFFBOARD failed. Common causes: setpoints not streaming, estimator not ready, safety/RC checks.\n");
        return false;
    }
    usleep(200000);
    if (!Arm(true, ackTimeoutMs, targetSystem, targetComponent)) {
        printf("[px4] ARM failed. Check STATUSTEXT for reason (EKF, safety switch, RC, arming checks).\n");
        return false;
    }
    printf("[px4] OFFBOARD + ARM OK\n");
    return true;
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
    WriteMessage(msg);
}

void Px4MavlinkGateway::StartSetpointStreamHz(double hz)
{
    if (hz <= 0.0)
        hz = 20.0;
    m_streamPeriodUs = static_cast<uint64_t>(1e6 / hz);
    StopSetpointStream();
    m_streaming.store(true);
    m_streamThread =
        SMARTDRONE_START_THREAD(smartdrone::common::ThreadRole::MavlinkSetpointStream, "Px4MavlinkGateway", [this]() {
            while (this->m_streaming.load()) {
                const uint32_t tMs = MonoTimeMs32();
                SetpointLocalNED sp;
                {
                    std::lock_guard<std::mutex> lk(this->m_spMtx);
                    sp = this->m_spCurrent;
                }
                this->SendSetPositionTargetLocalNed(tMs, sp, MAV_FRAME_LOCAL_NED);
                usleep(static_cast<useconds_t>(this->m_streamPeriodUs));
            }
        });
}

void Px4MavlinkGateway::StopSetpointStream()
{
    m_streaming.store(false);
    if (m_streamThread.joinable())
        m_streamThread.join();
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
    WriteMessage(msg);
}

bool Px4MavlinkGateway::SendLand(int ackTimeoutMs, uint8_t targetSystem, uint8_t targetComponent)
{
    uint8_t res = 255;
    // Use NaN for optional landing target fields so PX4 lands at current location
    // instead of interpreting 0/0/0 as an explicit target coordinate.
    bool got = SendCommandLongAndWaitAck(MAV_CMD_NAV_LAND, NAN, NAN, NAN, NAN, NAN, NAN, NAN, ackTimeoutMs,
                                         targetSystem, targetComponent, &res);
    return got && (res == MAV_RESULT_ACCEPTED);
}

bool Px4MavlinkGateway::GetTimesyncStatus(int64_t &offsetNs, uint32_t &rttUs, uint32_t &sampleCount) const
{
    std::lock_guard<std::mutex> lk(m_timesyncMtx);
    offsetNs = m_timesyncOffsetEstimateNs;
    rttUs = m_timesyncLastRttUs;
    sampleCount = m_timesyncSampleCount;
    return m_timesyncSampleCount != 0;
}

uint64_t Px4MavlinkGateway::EstimatePx4TimeFromCompanionMonotonicNs(uint64_t companionMonoNs, bool *outTimesyncValid,
                                                                    int64_t *outOffsetNs, uint32_t *outRttUs,
                                                                    uint32_t *outSampleCount) const
{
    int64_t offsetNs = 0;
    uint32_t rttUs = 0;
    uint32_t sampleCount = 0;
    const bool valid = GetTimesyncStatus(offsetNs, rttUs, sampleCount);
    if (outTimesyncValid)
        *outTimesyncValid = valid;
    if (outOffsetNs)
        *outOffsetNs = offsetNs;
    if (outRttUs)
        *outRttUs = rttUs;
    if (outSampleCount)
        *outSampleCount = sampleCount;
    if (!valid) {
        return companionMonoNs;
    }
    const int64_t px4NsSigned = static_cast<int64_t>(companionMonoNs) - offsetNs;
    return px4NsSigned > 0 ? static_cast<uint64_t>(px4NsSigned) : 0ULL;
}

void Px4MavlinkGateway::SendOdometry(uint64_t odomFrameId, const Pose &poseNed, const LinearVelocityNed &velNed,
                                     uint8_t mavFrameId, uint8_t childFrameId, uint8_t resetCounter,
                                     OdomQualityMode mode)
{
    mavlink_message_t msg;
    const float vx = velNed.x, vy = velNed.y, vz = velNed.z;
    const float rollspeed = NAN, pitchspeed = NAN, yawspeed = NAN;
    float poseCov[21];
    float velCov[21];
    int8_t quality = 100;
    uint8_t estimatorType = 0;
    float q[4] = {poseNed.qw, poseNed.qx, poseNed.qy, poseNed.qz};
    const bool haveVelocity = std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz);
    auto fillNanCov = [](float cov[21]) {
        for (int i = 0; i < 21; i++) {
            cov[i] = NAN;
        }
    };

    if (mode == OdomQualityMode::GOOD) {
        FillCovDiag21(poseCov, 0.04f, 0.04f, 0.36f, 0.03f, 0.03f, 0.03f);
        if (haveVelocity) {
            FillCovDiag21(velCov, 0.16f, 0.16f, 0.64f, 1.0f, 1.0f, 1.0f);
        } else {
            fillNanCov(velCov);
        }
        quality = 100;
    } else if (mode == OdomQualityMode::WEAK) {
        FillCovDiag21(poseCov, 2.25f, 2.25f, 9.0f, 0.25f, 0.25f, 0.25f);
        if (haveVelocity) {
            FillCovDiag21(velCov, 4.0f, 4.0f, 16.0f, 1.0f, 1.0f, 1.0f);
        } else {
            fillNanCov(velCov);
        }
        quality = 20;
    } else {
        FillCovDiag21(poseCov, 1e4f, 1e4f, 1e6f, 10.0f, 10.0f, 10.0f);
        if (haveVelocity) {
            FillCovDiag21(velCov, 1e2f, 1e2f, 1e4f, 1e2f, 1e2f, 1e2f);
        } else {
            fillNanCov(velCov);
        }
        quality = 0;
    }

    smartdrone::core::application::FrameTimingRecord timing{};
    const bool haveTiming = LookupFrameTiming(odomFrameId, timing);
    OdomTimesyncDiagnostics diagnostics{};
    diagnostics.frameId = odomFrameId;
    diagnostics.tCamNs = timing.tCamNs;
    diagnostics.tCbNs = timing.tCbNs;
    diagnostics.tSlamInNs = timing.tSlamInNs;
    diagnostics.tSlamOutNs = timing.tSlamOutNs;
    diagnostics.tMavTxNs = ClockMonotonicNs();
    if (!haveTiming || diagnostics.tCamNs == 0) {
        printf("[odom_warn] frame=%llu missing capture timestamp; skipping odometry publish\n",
               static_cast<unsigned long long>(odomFrameId));
        return;
    }
    diagnostics.estimatedPx4TimeNs = EstimatePx4TimeFromCompanionMonotonicNs(
        diagnostics.tCamNs, &diagnostics.timesyncValid, &diagnostics.timesyncOffsetNs, &diagnostics.timesyncRttUs);
    MarkFrameMavTx(odomFrameId, diagnostics.tMavTxNs);
    m_lastOdomTimesyncDiagnostics = diagnostics;
    const uint64_t companionCaptureTimeUs = diagnostics.tCamNs / 1000ULL;
    if (diagnostics.estimatedPx4TimeNs == 0) {
        printf("[odom_warn] frame=%llu px4 timestamp mapped to zero cam_ns=%llu sync=%d offset_ns=%lld\n",
               static_cast<unsigned long long>(odomFrameId), static_cast<unsigned long long>(diagnostics.tCamNs),
               diagnostics.timesyncValid ? 1 : 0, static_cast<long long>(diagnostics.timesyncOffsetNs));
    }
    if (m_lastEstimatedPx4OdomTimeNs != 0 && diagnostics.estimatedPx4TimeNs <= m_lastEstimatedPx4OdomTimeNs) {
        printf("[odom_warn] frame=%llu non-monotonic px4 timestamp prev_frame=%llu prev_px4_ns=%llu px4_ns=%llu "
               "cam_ns=%llu sync=%d offset_ms=%.3f\n",
               static_cast<unsigned long long>(odomFrameId), static_cast<unsigned long long>(m_lastSentOdomFrameId),
               static_cast<unsigned long long>(m_lastEstimatedPx4OdomTimeNs),
               static_cast<unsigned long long>(diagnostics.estimatedPx4TimeNs),
               static_cast<unsigned long long>(diagnostics.tCamNs), diagnostics.timesyncValid ? 1 : 0,
               static_cast<double>(diagnostics.timesyncOffsetNs) * 1e-6);
    }
    m_lastSentOdomFrameId = odomFrameId;
    m_lastEstimatedPx4OdomTimeNs = diagnostics.estimatedPx4TimeNs;

    mavlink_msg_odometry_pack(m_sysid, m_compid, &msg, companionCaptureTimeUs, mavFrameId, childFrameId, poseNed.x,
                              poseNed.y, poseNed.z, q, vx, vy, vz, rollspeed, pitchspeed, yawspeed, poseCov, velCov,
                              resetCounter, estimatorType, quality);

    WriteMessage(msg);

    const double queueLatencyMs = (diagnostics.tSlamInNs >= diagnostics.tCbNs && diagnostics.tCbNs != 0)
                                      ? (static_cast<double>(diagnostics.tSlamInNs - diagnostics.tCbNs) * 1e-6)
                                      : -1.0;
    const double slamLatencyMs = (diagnostics.tSlamOutNs >= diagnostics.tCamNs)
                                     ? (static_cast<double>(diagnostics.tSlamOutNs - diagnostics.tCamNs) * 1e-6)
                                     : -1.0;
    const double sendLatencyMs = (diagnostics.tMavTxNs >= diagnostics.tSlamOutNs && diagnostics.tSlamOutNs != 0)
                                     ? (static_cast<double>(diagnostics.tMavTxNs - diagnostics.tSlamOutNs) * 1e-6)
                                     : -1.0;
    const double totalLatencyMs = (diagnostics.tMavTxNs >= diagnostics.tCamNs)
                                      ? (static_cast<double>(diagnostics.tMavTxNs - diagnostics.tCamNs) * 1e-6)
                                      : -1.0;
    const double px4OffsetMs = static_cast<double>(diagnostics.timesyncOffsetNs) * 1e-6;
    const bool odomTsPeriodic = (kOdomTsLogEveryNFrames > 0) && ((diagnostics.frameId % kOdomTsLogEveryNFrames) == 0);
    const bool odomTsAbnormal = !diagnostics.timesyncValid || totalLatencyMs > 120.0 || queueLatencyMs > 50.0 ||
                                slamLatencyMs > 80.0 || sendLatencyMs > 30.0;
    if (odomTsPeriodic || odomTsAbnormal) {
        printf("[odom_ts] frame=%llu timing=%d reset=%u quality=%d cam_ns=%llu px4_est_ns=%llu "
               "sync=%d offset_ms=%.3f rtt_ms=%.3f "
               "queue_ms=%.3f slam_ms=%.3f send_ms=%.3f total_ms=%.3f\n",
               static_cast<unsigned long long>(diagnostics.frameId), haveTiming ? 1 : 0,
               static_cast<unsigned>(resetCounter), static_cast<int>(quality),
               static_cast<unsigned long long>(diagnostics.tCamNs),
               static_cast<unsigned long long>(diagnostics.estimatedPx4TimeNs), diagnostics.timesyncValid ? 1 : 0,
               px4OffsetMs, static_cast<double>(diagnostics.timesyncRttUs) * 1e-3, queueLatencyMs, slamLatencyMs,
               sendLatencyMs, totalLatencyMs);
    }
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

void Px4MavlinkGateway::ResetTimesyncEstimateLocked()
{
    m_timesyncOffsetEstimateNs = 0;
    m_timesyncLastRttUs = 0;
    m_timesyncSampleCount = 0;
}

bool Px4MavlinkGateway::LookupFrameTiming(uint64_t frameId, smartdrone::core::application::FrameTimingRecord &out) const
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

void Px4MavlinkGateway::WriteMessage(const mavlink_message_t &msg)
{
    std::lock_guard<std::mutex> txLock(m_txMtx);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (!m_transport.WriteAll(buf, len, 200)) {
        printf("[mav] write failed or timed out len=%u\n", unsigned(len));
    }
}

void Px4MavlinkGateway::SendTimesyncMessage(int64_t tc1Ns, int64_t ts1Ns, uint8_t targetSystem, uint8_t targetComponent)
{
    mavlink_message_t msg{};
    mavlink_msg_timesync_pack(m_sysid, m_compid, &msg, tc1Ns, ts1Ns, targetSystem, targetComponent);
    WriteMessage(msg);
}

void Px4MavlinkGateway::SendTimesyncRequest(uint8_t targetSystem, uint8_t targetComponent)
{
    const uint64_t nowNs = ClockMonotonicNs();
    {
        std::lock_guard<std::mutex> lk(m_timesyncMtx);
        m_pendingTimesyncTs1Ns = static_cast<int64_t>(nowNs);
        m_pendingTimesyncSentNs = nowNs;
        m_pendingTimesyncTargetSystem = targetSystem;
        m_pendingTimesyncTargetComponent = targetComponent;
        m_havePendingTimesync = true;
    }
    SendTimesyncMessage(0, static_cast<int64_t>(nowNs), targetSystem, targetComponent);
}

void Px4MavlinkGateway::SendTimesyncResponse(int64_t requestTs1Ns, uint8_t targetSystem, uint8_t targetComponent)
{
    const int64_t nowNs = static_cast<int64_t>(ClockMonotonicNs());
    SendTimesyncMessage(nowNs, requestTs1Ns, targetSystem, targetComponent);
}

void Px4MavlinkGateway::SendTimesyncFollowUp(int64_t remoteStampNs, uint8_t targetSystem, uint8_t targetComponent)
{
    const int64_t nowNs = static_cast<int64_t>(ClockMonotonicNs());
    SendTimesyncMessage(nowNs, remoteStampNs, targetSystem, targetComponent);
}

void Px4MavlinkGateway::TimesyncLoop()
{
    while (m_timesyncRunning.load()) {
        if (!m_havePx4Heartbeat.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        const uint8_t targetSystem = m_px4Sysid.load();
        const uint8_t targetComponent = m_px4Compid.load();
        if (targetSystem != 0 && targetComponent != 0) {
            bool shouldSend = false;
            bool timedOut = false;
            bool recentlyActive = false;
            const uint64_t nowNs = ClockMonotonicNs();
            {
                std::lock_guard<std::mutex> lk(m_timesyncMtx);
                const uint64_t nowUs = nowNs / 1000ULL;
                recentlyActive = (m_lastTimesyncActivityUs != 0) && (nowUs <= (m_lastTimesyncActivityUs + 2000000ULL));
                if (!m_havePendingTimesync) {
                    shouldSend = true;
                } else if (nowNs > (m_pendingTimesyncSentNs + 500000000ULL)) {
                    m_havePendingTimesync = false;
                    shouldSend = true;
                    timedOut = !recentlyActive;
                }
            }
            if (timedOut) {
                printf("[timesync] request timed out; retrying\n");
            }
            if (shouldSend) {
                SendTimesyncRequest(targetSystem, targetComponent);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Px4MavlinkGateway::RxLoop()
{
    mavlink_message_t msg{};
    mavlink_status_t status{};
    while (m_rxRunning.load()) {
        int pr = m_transport.PollReadable(200);
        if (pr <= 0)
            continue;

        uint8_t buf[512];
        ssize_t n = m_transport.Read(buf, sizeof(buf));
        if (n <= 0)
            continue;
        for (ssize_t i = 0; i < n; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                HandleMavlinkMessage(msg);
            }
        }
    }
}

void Px4MavlinkGateway::HandleMavlinkMessage(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
        mavlink_timesync_t tsync{};
        mavlink_msg_timesync_decode(&msg, &tsync);
        const bool targetSystemMatch = (tsync.target_system == 0) || (tsync.target_system == m_sysid);
        const bool targetComponentMatch = (tsync.target_component == 0) || (tsync.target_component == m_compid);
        if (!targetSystemMatch || !targetComponentMatch) {
            return;
        }

        if (tsync.tc1 == 0) {
            bool shouldLog = false;
            uint32_t requestCount = 0;
            {
                std::lock_guard<std::mutex> lk(m_timesyncMtx);
                ++m_timesyncInboundRequestCount;
                requestCount = m_timesyncInboundRequestCount;
                const uint64_t nowUs = MonoTimeUs();
                m_lastTimesyncActivityUs = nowUs;
                if (requestCount <= 3 || (requestCount % 50) == 0 || (nowUs > (m_lastTimesyncLogUs + 5000000ULL))) {
                    m_lastTimesyncLogUs = nowUs;
                    shouldLog = true;
                }
            }
            if (shouldLog) {
                printf("[timesync] rx PX4 request count=%u px4=%u/%u\n", requestCount, static_cast<unsigned>(msg.sysid),
                       static_cast<unsigned>(msg.compid));
            }
            SendTimesyncResponse(tsync.ts1, msg.sysid, msg.compid);
            return;
        }

        const uint64_t nowNs = ClockMonotonicNs();
        bool matchedPending = false;
        uint64_t pendingSentNs = 0;
        {
            std::lock_guard<std::mutex> lk(m_timesyncMtx);
            if (m_havePendingTimesync && tsync.ts1 == m_pendingTimesyncTs1Ns &&
                msg.sysid == m_pendingTimesyncTargetSystem && msg.compid == m_pendingTimesyncTargetComponent) {
                matchedPending = true;
                pendingSentNs = m_pendingTimesyncSentNs;
                m_havePendingTimesync = false;
            }
        }

        if (!matchedPending) {
            return;
        }

        const int64_t observedOffsetNs = static_cast<int64_t>((pendingSentNs + nowNs) / 2ULL) - tsync.tc1;
        const uint64_t rttUs64 = (nowNs > pendingSentNs) ? ((nowNs - pendingSentNs) / 1000ULL) : 0ULL;
        const uint32_t rttUs = (rttUs64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFu : static_cast<uint32_t>(rttUs64);

        int64_t filteredOffsetNs = observedOffsetNs;
        uint32_t sampleCount = 0;
        bool shouldLog = false;
        bool rejectedByRtt = false;
        bool resetByJump = false;
        bool jumpWarn = false;
        int64_t jumpNs = 0;
        {
            std::lock_guard<std::mutex> lk(m_timesyncMtx);
            const bool haveEstimate = m_timesyncSampleCount != 0;
            if (rttUs > kTimesyncMaxAcceptableRttUs) {
                rejectedByRtt = true;
                filteredOffsetNs = m_timesyncOffsetEstimateNs;
                sampleCount = m_timesyncSampleCount;
            } else {
                if (haveEstimate) {
                    jumpNs = observedOffsetNs - m_timesyncOffsetEstimateNs;
                    const int64_t absJumpNs = jumpNs < 0 ? -jumpNs : jumpNs;
                    if (absJumpNs > kTimesyncJumpResetThresholdNs) {
                        ResetTimesyncEstimateLocked();
                        resetByJump = true;
                    } else if (absJumpNs > kTimesyncJumpWarnThresholdNs) {
                        jumpWarn = true;
                    }
                }

                if (m_timesyncSampleCount == 0) {
                    m_timesyncOffsetEstimateNs = observedOffsetNs;
                } else {
                    m_timesyncOffsetEstimateNs = (m_timesyncOffsetEstimateNs * 7 + observedOffsetNs) / 8;
                }
                ++m_timesyncSampleCount;
                m_timesyncLastRttUs = rttUs;
                filteredOffsetNs = m_timesyncOffsetEstimateNs;
                sampleCount = m_timesyncSampleCount;
            }
            ++m_timesyncInboundResponseCount;
            const uint64_t nowUs = nowNs / 1000ULL;
            m_lastTimesyncActivityUs = nowUs;
            if (rejectedByRtt || resetByJump || jumpWarn || sampleCount <= 5 ||
                (sampleCount != 0 && (sampleCount % 20) == 0) || (nowUs > (m_lastTimesyncLogUs + 5000000ULL))) {
                m_lastTimesyncLogUs = nowUs;
                shouldLog = true;
            }
        }

        if (shouldLog) {
            printf("[timesync] samples=%u offset=%.3fms rtt=%.3fms accepted=%d reset=%d jump_ms=%.3f px4=%u/%u\n",
                   sampleCount, static_cast<double>(filteredOffsetNs) * 1e-6, static_cast<double>(rttUs) * 1e-3,
                   rejectedByRtt ? 0 : 1, resetByJump ? 1 : 0, static_cast<double>(jumpNs) * 1e-6,
                   static_cast<unsigned>(msg.sysid), static_cast<unsigned>(msg.compid));
        }

        SendTimesyncFollowUp(tsync.tc1, msg.sysid, msg.compid);
        return;
    }

    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t hb{};
        mavlink_msg_heartbeat_decode(&msg, &hb);
        if (hb.autopilot != MAV_AUTOPILOT_INVALID) {
            FlightModeInfo modeInfo{};
            modeInfo.baseMode = hb.base_mode;
            modeInfo.customMode = hb.custom_mode;
            modeInfo.mainMode = static_cast<uint8_t>((hb.custom_mode >> 16) & 0xFFu);
            modeInfo.subMode = static_cast<uint8_t>((hb.custom_mode >> 24) & 0xFFu);
            modeInfo.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            modeInfo.receivedUs = MonoTimeUs();
            {
                std::lock_guard<std::mutex> lk(m_flightModeMtx);
                m_flightModeInfo = modeInfo;
                m_haveFlightModeInfo = true;
            }
            m_havePx4Heartbeat.store(true);
            m_px4Sysid.store(msg.sysid);
            m_px4Compid.store(msg.compid);
            MaybeRequestLocalPositionNedStream(msg.sysid, msg.compid);
            MaybeRequestDistanceSensorStream(msg.sysid, msg.compid);
        }
        return;
    }

    if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
        mavlink_local_position_ned_t lpos{};
        mavlink_msg_local_position_ned_decode(&msg, &lpos);
        LocalPositionNed sample{};
        sample.x = lpos.x;
        sample.y = lpos.y;
        sample.z = lpos.z;
        sample.vx = lpos.vx;
        sample.vy = lpos.vy;
        sample.vz = lpos.vz;
        sample.timeBootMs = lpos.time_boot_ms;
        sample.receivedUs = MonoTimeUs();
        {
            std::lock_guard<std::mutex> lk(m_localPosMtx);
            m_localPosNed = sample;
            m_haveLocalPosNed = true;
        }
        return;
    }

    if (msg.msgid == MAVLINK_MSG_ID_DISTANCE_SENSOR) {
        mavlink_distance_sensor_t dist{};
        mavlink_msg_distance_sensor_decode(&msg, &dist);
        if (dist.orientation == MAV_SENSOR_ROTATION_PITCH_270) {
            DownwardDistanceSensor sample{};
            sample.currentDistance = 0.01f * static_cast<float>(dist.current_distance);
            sample.minDistance = 0.01f * static_cast<float>(dist.min_distance);
            sample.maxDistance = 0.01f * static_cast<float>(dist.max_distance);
            sample.signalQuality = dist.signal_quality;
            sample.receivedUs = MonoTimeUs();
            {
                std::lock_guard<std::mutex> lk(m_distanceSensorMtx);
                m_downwardDistanceSensor = sample;
                m_haveDownwardDistanceSensor = true;
            }
        }
        return;
    }

    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        mavlink_command_ack_t ack{};
        mavlink_msg_command_ack_decode(&msg, &ack);
        AckInfo info;
        info.result = ack.result;
        info.progress = ack.progress;
        info.resultParam2 = ack.result_param2;
        info.t = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lk(m_ackMtx);
            m_ackMap[ack.command] = info;
        }
        m_ackCv.notify_all();
        printf("[ACK] sys=%d, comp=%d cmd=%u result= %d(%s) progress=%d param2=%d\n", int(msg.sysid), int(msg.compid),
               ack.command, int(ack.result), MavResultToStr(ack.result), int(ack.progress), ack.result_param2);
        return;
    }

    if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        mavlink_statustext_t st{};
        mavlink_msg_statustext_decode(&msg, &st);
        char text[sizeof(st.text) + 1];
        std::memcpy(text, st.text, sizeof(st.text));
        text[sizeof(st.text)] = '\0';
        printf("[STATUSTEXT] sev=%d %s\n", int(st.severity), text);
        return;
    }
}

void Px4MavlinkGateway::MaybeRequestLocalPositionNedStream(uint8_t targetSystem, uint8_t targetComponent)
{
    bool expected = false;
    if (!m_localPosStreamRequested.compare_exchange_strong(expected, true)) {
        return;
    }
    constexpr float kIntervalUs = 50000.0f;
    SendCommandLong(MAV_CMD_SET_MESSAGE_INTERVAL, static_cast<float>(MAVLINK_MSG_ID_LOCAL_POSITION_NED), kIntervalUs, 0,
                    0, 0, 0, 0, targetSystem, targetComponent);
    printf("[mav] requested LOCAL_POSITION_NED @20Hz from sys=%d comp=%d\n", int(targetSystem), int(targetComponent));
}

void Px4MavlinkGateway::MaybeRequestDistanceSensorStream(uint8_t targetSystem, uint8_t targetComponent)
{
    bool expected = false;
    if (!m_distanceSensorStreamRequested.compare_exchange_strong(expected, true)) {
        return;
    }
    constexpr float kIntervalUs = 50000.0f;
    SendCommandLong(MAV_CMD_SET_MESSAGE_INTERVAL, static_cast<float>(MAVLINK_MSG_ID_DISTANCE_SENSOR), kIntervalUs, 0, 0,
                    0, 0, 0, targetSystem, targetComponent);
    printf("[mav] requested DISTANCE_SENSOR @20Hz from sys=%d comp=%d\n", int(targetSystem), int(targetComponent));
}
