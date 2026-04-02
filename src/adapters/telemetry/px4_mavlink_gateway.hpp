#pragma once

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <string>
#include <thread>
#include <time.h>
#include <unordered_map>

#include "common/mavlink.h"
#include "common/time_utils.hpp"
#include "adapters/telemetry/mavlink_serial_transport.hpp"

enum class OdomQualityMode
{
    GOOD,
    WEAK,
    LOST
};

static inline void FillCovDiag21(float cov[21],
                                 float varX, float varY, float varZ,
                                 float varRoll, float varPitch, float varYaw,
                                 bool fillOffdiagZero = true)
{
    if (fillOffdiagZero) {
        for (int i = 0; i < 21; i++) cov[i] = 0.0f;
    }
    cov[0] = varX;
    cov[6] = varY;
    cov[11] = varZ;
    cov[15] = varRoll;
    cov[18] = varPitch;
    cov[20] = varYaw;
}

static inline uint64_t ClockMonotonicNs()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

class Px4MavlinkGateway {
public:
    struct Pose {
        float x, y, z;
        float qw, qx, qy, qz;
    };

    struct LinearVelocityNed {
        LinearVelocityNed(float xIn = NAN, float yIn = NAN, float zIn = NAN)
            : x(xIn), y(yIn), z(zIn)
        {
        }

        float x;
        float y;
        float z;
    };

    struct LocalPositionNed {
        float x{0.0f}, y{0.0f}, z{0.0f};
        float vx{0.0f}, vy{0.0f}, vz{0.0f};
        uint32_t timeBootMs{0};
        uint64_t receivedUs{0};
    };

    struct DownwardDistanceSensor {
        float currentDistance{NAN};
        float minDistance{NAN};
        float maxDistance{NAN};
        uint8_t signalQuality{0};
        uint64_t receivedUs{0};
    };

    struct ManualControlInput {
        float throttleNorm{0.0f};
        float yawNorm{0.0f};
        float pitchNorm{0.0f};
        float rollNorm{0.0f};
    };

    struct FlightModeInfo {
        uint8_t baseMode{0};
        uint32_t customMode{0};
        uint8_t mainMode{0};
        uint8_t subMode{0};
        bool armed{false};
        uint64_t receivedUs{0};
    };

    struct SetpointLocalNED {
        float x = NAN, y = NAN, z = NAN;
        float vx = NAN, vy = NAN, vz = NAN;
        float ax = NAN, ay = NAN, az = NAN;
        float yaw = NAN;
        float yawspeed = NAN;
    };

    static constexpr uint8_t PX4_CUSTOM_MAIN_MODE_MANUAL = 1;
    static constexpr uint8_t PX4_CUSTOM_MAIN_MODE_ALTCTL = 2;
    static constexpr uint8_t PX4_CUSTOM_MAIN_MODE_POSCTL = 3;
    static constexpr uint8_t PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;

    explicit Px4MavlinkGateway(const std::string& dev, int baud,
                               uint8_t sysid = 42,
                               uint8_t compid = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY)
        : m_transport(dev, baud), m_sysid(sysid), m_compid(compid)
    {
    }

    ~Px4MavlinkGateway()
    {
        StopRx();
        StopSetpointStream();
    }

    void StartRx()
    {
        StopRx();
        {
            std::lock_guard<std::mutex> lk(m_timesyncMtx);
            m_havePendingTimesync = false;
            m_pendingTimesyncTs1Ns = 0;
            m_pendingTimesyncSentNs = 0;
            m_pendingTimesyncTargetSystem = 0;
            m_pendingTimesyncTargetComponent = 0;
            m_timesyncEstimatedOffsetNs = 0;
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
        m_rxThread = std::thread([this]() { this->RxLoop(); });
        m_timesyncThread = std::thread([this]() { this->TimesyncLoop(); });
    }

    void StopRx()
    {
        m_rxRunning.store(false);
        m_timesyncRunning.store(false);
        if (m_rxThread.joinable()) m_rxThread.join();
        if (m_timesyncThread.joinable()) m_timesyncThread.join();
    }

    uint8_t GetTargetSystem() const { return m_px4Sysid.load(); }
    uint8_t GetTargetComponent() const { return m_px4Compid.load(); }

    bool GetLocalPositionNed(LocalPositionNed& out, uint64_t maxAgeUs = 500000) const
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

    bool GetDownwardDistanceSensor(DownwardDistanceSensor& out, uint64_t maxAgeUs = 200000) const
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

    bool WaitCommandAck(uint16_t command, int timeoutMs, uint8_t& outResult)
    {
        std::unique_lock<std::mutex> lk(m_ackMtx);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
        auto pred = [&]() {
            auto it = m_ackMap.find(command);
            return it != m_ackMap.end();
        };
        if (!m_ackCv.wait_until(lk, deadline, pred)) return false;
        outResult = m_ackMap[command].result;
        return true;
    }

    bool GetFlightModeInfo(FlightModeInfo& out, uint64_t maxAgeUs = 1500000) const
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

    void SendCommandLong(uint16_t command,
                         float p1 = 0, float p2 = 0, float p3 = 0, float p4 = 0,
                         float p5 = 0, float p6 = 0, float p7 = 0,
                         uint8_t targetSystem = 1, uint8_t targetComponent = 1,
                         uint8_t confirmation = 0)
    {
        mavlink_message_t msg{};
        mavlink_msg_command_long_pack(
            m_sysid, m_compid, &msg,
            targetSystem, targetComponent,
            command, confirmation,
            p1, p2, p3, p4, p5, p6, p7);
        WriteMessage(msg);
    }

    bool SendCommandLongAndWaitAck(uint16_t command,
                                   float p1 = 0, float p2 = 0, float p3 = 0, float p4 = 0,
                                   float p5 = 0, float p6 = 0, float p7 = 0,
                                   int timeoutMs = 800,
                                   uint8_t targetSystem = 0, uint8_t targetComponent = 0,
                                   uint8_t* outResult = nullptr)
    {
        if (targetSystem == 0) targetSystem = GetTargetSystem();
        if (targetComponent == 0) targetComponent = GetTargetComponent();
        {
            std::lock_guard<std::mutex> lk(m_ackMtx);
            m_ackMap.erase(command);
        }
        SendCommandLong(command, p1, p2, p3, p4, p5, p6, p7, targetSystem, targetComponent);
        uint8_t res = 255;
        bool ok = WaitCommandAck(command, timeoutMs, res);
        if (outResult) *outResult = res;
        if (!ok) {
            printf("[ACK] TIMEOUT cmd=%u (target sys=%d comp=%d)\n", command, int(targetSystem), int(targetComponent));
        } else {
            printf("[ACK] cmd=%u result=%s\n", command, MavResultToStr(res));
        }
        return ok;
    }

    bool SetModePx4Main(uint8_t mainMode, int ackTimeoutMs = 800, uint8_t targetSystem = 0, uint8_t targetComponent = 0)
    {
        const float baseMode = (float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        const uint32_t customMode = (uint32_t)mainMode << 16;
        uint8_t res = 255;
        bool got = SendCommandLongAndWaitAck(MAV_CMD_DO_SET_MODE, baseMode, (float)customMode, 0, 0, 0, 0, 0,
                                             ackTimeoutMs, targetSystem, targetComponent, &res);
        return got && (res == MAV_RESULT_ACCEPTED);
    }

    bool SetModeOffboard(int ackTimeoutMs = 800, uint8_t targetSystem = 0, uint8_t targetComponent = 0)
    {
        return SetModePx4Main(PX4_CUSTOM_MAIN_MODE_OFFBOARD, ackTimeoutMs, targetSystem, targetComponent);
    }

    bool SetModePosition(int ackTimeoutMs = 800, uint8_t targetSystem = 0, uint8_t targetComponent = 0)
    {
        return SetModePx4Main(PX4_CUSTOM_MAIN_MODE_POSCTL, ackTimeoutMs, targetSystem, targetComponent);
    }

    bool SetModeAltitude(int ackTimeoutMs = 800, uint8_t targetSystem = 0, uint8_t targetComponent = 0)
    {
        return SetModePx4Main(PX4_CUSTOM_MAIN_MODE_ALTCTL, ackTimeoutMs, targetSystem, targetComponent);
    }

    bool Arm(bool doArm, int ackTimeoutMs = 800, uint8_t targetSystem = 0, uint8_t targetComponent = 0)
    {
        uint8_t res = 255;
        bool got = SendCommandLongAndWaitAck(MAV_CMD_COMPONENT_ARM_DISARM, doArm ? 1.0f : 0.0f,
                                             0, 0, 0, 0, 0, 0,
                                             ackTimeoutMs, targetSystem, targetComponent, &res);
        return got && (res == MAV_RESULT_ACCEPTED);
    }

    bool EmergencyStop(int ackTimeoutMs = 800, uint8_t targetSystem = 0, uint8_t targetComponent = 0)
    {
        uint8_t res = 255;
        bool got = SendCommandLongAndWaitAck(MAV_CMD_COMPONENT_ARM_DISARM, 0.0f, 21196.0f,
                                             0, 0, 0, 0, 0,
                                             ackTimeoutMs, targetSystem, targetComponent, &res);
        return got && (res == MAV_RESULT_ACCEPTED);
    }

    bool StartOffboardAndArm(double, double, int warmupMs = 800, int ackTimeoutMs = 1000,
                             uint8_t targetSystem = 0, uint8_t targetComponent = 0)
    {
        usleep((useconds_t)warmupMs * 1000);
        if (!SetModeOffboard(ackTimeoutMs, targetSystem, targetComponent)) {
            printf("[px4] OFFBOARD failed. Common causes: setpoints not streaming, estimator not ready, safety/RC checks.\n");
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

    void SendSetPositionTargetLocalNed(uint32_t timeBootMs,
                                       const SetpointLocalNED& sp,
                                       uint8_t coordinateFrame = MAV_FRAME_LOCAL_NED)
    {
        uint16_t typeMask = 0;
        auto ign = [&](int bit) { typeMask |= (1u << bit); };
        if (!std::isfinite(sp.x)) ign(0);
        if (!std::isfinite(sp.y)) ign(1);
        if (!std::isfinite(sp.z)) ign(2);
        if (!std::isfinite(sp.vx)) ign(3);
        if (!std::isfinite(sp.vy)) ign(4);
        if (!std::isfinite(sp.vz)) ign(5);
        if (!std::isfinite(sp.ax)) ign(6);
        if (!std::isfinite(sp.ay)) ign(7);
        if (!std::isfinite(sp.az)) ign(8);
        ign(9);
        if (!std::isfinite(sp.yaw)) ign(10);
        if (!std::isfinite(sp.yawspeed)) ign(11);

        mavlink_message_t msg{};
        const uint8_t targetSystem = 1;
        const uint8_t targetComponent = 1;
        mavlink_msg_set_position_target_local_ned_pack(
            m_sysid, m_compid, &msg,
            timeBootMs,
            targetSystem,
            targetComponent,
            coordinateFrame,
            typeMask,
            std::isfinite(sp.x) ? sp.x : 0.0f,
            std::isfinite(sp.y) ? sp.y : 0.0f,
            std::isfinite(sp.z) ? sp.z : 0.0f,
            std::isfinite(sp.vx) ? sp.vx : 0.0f,
            std::isfinite(sp.vy) ? sp.vy : 0.0f,
            std::isfinite(sp.vz) ? sp.vz : 0.0f,
            std::isfinite(sp.ax) ? sp.ax : 0.0f,
            std::isfinite(sp.ay) ? sp.ay : 0.0f,
            std::isfinite(sp.az) ? sp.az : 0.0f,
            std::isfinite(sp.yaw) ? sp.yaw : 0.0f,
            std::isfinite(sp.yawspeed) ? sp.yawspeed : 0.0f);
        WriteMessage(msg);
    }

    void StartSetpointStreamHz(double hz = 20.0)
    {
        if (hz <= 0.0) hz = 20.0;
        m_streamPeriodUs = static_cast<uint64_t>(1e6 / hz);
        StopSetpointStream();
        m_streaming.store(true);
        m_streamThread = std::thread([this]() {
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

    void StopSetpointStream()
    {
        m_streaming.store(false);
        if (m_streamThread.joinable()) m_streamThread.join();
    }

    void UpdateStreamSetpoint(const SetpointLocalNED& spNed)
    {
        std::lock_guard<std::mutex> lk(m_spMtx);
        m_spCurrent = spNed;
    }

    void UpdateStreamPosition(float xN, float yE, float zD, float yawRad = NAN)
    {
        SetpointLocalNED sp{};
        sp.x = xN;
        sp.y = yE;
        sp.z = zD;
        sp.yaw = yawRad;
        UpdateStreamSetpoint(sp);
    }

    void SendManualControl(const ManualControlInput& input,
                           uint16_t buttons = 0,
                           uint16_t buttons2 = 0,
                           uint8_t targetSystem = 0)
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
        mavlink_msg_manual_control_pack(
            m_sysid, m_compid, &msg,
            targetSystem,
            toAxis(input.pitchNorm),
            toAxis(input.rollNorm),
            toThrottleAxis(input.throttleNorm),
            toAxis(input.yawNorm),
            buttons,
            buttons2,
            0,
            0, 0, 0, 0, 0, 0, 0, 0);
        WriteMessage(msg);
    }

    bool SendLand(int ackTimeoutMs = 800, uint8_t targetSystem = 1, uint8_t targetComponent = 1)
    {
        uint8_t res = 255;
        bool got = SendCommandLongAndWaitAck(MAV_CMD_NAV_LAND,
                                             0, 0, 0, 0, 0, 0, 0,
                                             ackTimeoutMs,
                                             targetSystem, targetComponent,
                                             &res);
        return got && (res == MAV_RESULT_ACCEPTED);
    }

    void SendOdometry(uint64_t timestampUs,
                      const Pose& poseNed,
                      const LinearVelocityNed& velNed = LinearVelocityNed{},
                      uint8_t frameId = MAV_FRAME_LOCAL_NED,
                      uint8_t childFrameId = MAV_FRAME_BODY_FRD,
                      uint8_t resetCounter = 0,
                      OdomQualityMode mode = OdomQualityMode::GOOD)
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

        mavlink_msg_odometry_pack(
            m_sysid, m_compid, &msg,
            timestampUs,
            frameId,
            childFrameId,
            poseNed.x, poseNed.y, poseNed.z,
            q,
            vx, vy, vz,
            rollspeed, pitchspeed, yawspeed,
            poseCov,
            velCov,
            resetCounter,
            estimatorType,
            quality);

        WriteMessage(msg);
    }

    static Pose EnuToNed(const Pose& pEnu)
    {
        Pose out{};
        out.x = pEnu.y;
        out.y = pEnu.x;
        out.z = -pEnu.z;

        const float s = 0.7071067811865476f;
        const float qrW = 0.0f, qrX = s, qrY = s, qrZ = 0.0f;

        auto qmul = [](float aw, float ax, float ay, float az,
                       float bw, float bx, float by, float bz,
                       float& ow, float& ox, float& oy, float& oz) {
            ow = aw * bw - ax * bx - ay * by - az * bz;
            ox = aw * bx + ax * bw + ay * bz - az * by;
            oy = aw * by - ax * bz + ay * bw + az * bx;
            oz = aw * bz + ax * by - ay * bx + az * bw;
        };

        auto qconj = [](float w, float x, float y, float z, float& ow, float& ox, float& oy, float& oz) {
            ow = w; ox = -x; oy = -y; oz = -z;
        };

        float tW, tX, tY, tZ;
        qmul(qrW, qrX, qrY, qrZ, pEnu.qw, pEnu.qx, pEnu.qy, pEnu.qz, tW, tX, tY, tZ);
        float qrcW, qrcX, qrcY, qrcZ;
        qconj(qrW, qrX, qrY, qrZ, qrcW, qrcX, qrcY, qrcZ);
        qmul(tW, tX, tY, tZ, qrcW, qrcX, qrcY, qrcZ, out.qw, out.qx, out.qy, out.qz);
        NormalizeQuat(out.qw, out.qx, out.qy, out.qz);
        return out;
    }

    static void NormalizeQuat(float& w, float& x, float& y, float& z)
    {
        const float n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n > 1e-9f) { w /= n; x /= n; y /= n; z /= n; }
        else { w = 1; x = y = z = 0; }
    }

private:
    struct AckInfo {
        uint8_t result = 255;
        uint8_t progress = 0;
        int32_t resultParam2 = 0;
        std::chrono::steady_clock::time_point t;
    };

    static const char* MavResultToStr(uint8_t r)
    {
        switch (r) {
            case MAV_RESULT_ACCEPTED: return "ACCEPTED";
            case MAV_RESULT_TEMPORARILY_REJECTED: return "TEMP_REJECTED";
            case MAV_RESULT_DENIED: return "DENIED";
            case MAV_RESULT_UNSUPPORTED: return "UNSUPPORTED";
            case MAV_RESULT_FAILED: return "FAILED";
            case MAV_RESULT_IN_PROGRESS: return "IN_PROGRESS";
            default: return "UNKNOWN";
        }
    }

    void WriteMessage(const mavlink_message_t& msg)
    {
        std::lock_guard<std::mutex> txLock(m_txMtx);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        if (!m_transport.WriteAll(buf, len, 200)) {
            printf("[mav] write failed or timed out len=%u\n", unsigned(len));
        }
    }

    void SendTimesyncMessage(int64_t tc1Ns, int64_t ts1Ns, uint8_t targetSystem, uint8_t targetComponent)
    {
        mavlink_message_t msg{};
        mavlink_msg_timesync_pack(
            m_sysid, m_compid, &msg,
            tc1Ns, ts1Ns,
            targetSystem, targetComponent);
        WriteMessage(msg);
    }

    void SendTimesyncRequest(uint8_t targetSystem, uint8_t targetComponent)
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

    void SendTimesyncResponse(int64_t requestTs1Ns, uint8_t targetSystem, uint8_t targetComponent)
    {
        const int64_t nowNs = static_cast<int64_t>(ClockMonotonicNs());
        SendTimesyncMessage(nowNs, requestTs1Ns, targetSystem, targetComponent);
    }

    void SendTimesyncFollowUp(int64_t remoteStampNs, uint8_t targetSystem, uint8_t targetComponent)
    {
        const int64_t nowNs = static_cast<int64_t>(ClockMonotonicNs());
        SendTimesyncMessage(nowNs, remoteStampNs, targetSystem, targetComponent);
    }

    void TimesyncLoop()
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
                    recentlyActive =
                        (m_lastTimesyncActivityUs != 0) &&
                        (nowUs <= (m_lastTimesyncActivityUs + 2000000ULL));
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

    void RxLoop()
    {
        mavlink_message_t msg{};
        mavlink_status_t status{};
        while (m_rxRunning.load()) {
            int pr = m_transport.PollReadable(200);
            if (pr <= 0) continue;

            uint8_t buf[512];
            ssize_t n = m_transport.Read(buf, sizeof(buf));
            if (n <= 0) continue;
            for (ssize_t i = 0; i < n; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    HandleMavlinkMessage(msg);
                }
            }
        }
    }

    void HandleMavlinkMessage(const mavlink_message_t& msg)
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
                    if (requestCount <= 3 || (requestCount % 50) == 0 ||
                        (nowUs > (m_lastTimesyncLogUs + 5000000ULL))) {
                        m_lastTimesyncLogUs = nowUs;
                        shouldLog = true;
                    }
                }
                if (shouldLog) {
                    printf("[timesync] rx PX4 request count=%u px4=%u/%u\n",
                           requestCount,
                           static_cast<unsigned>(msg.sysid),
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
                if (m_havePendingTimesync &&
                    tsync.ts1 == m_pendingTimesyncTs1Ns &&
                    msg.sysid == m_pendingTimesyncTargetSystem &&
                    msg.compid == m_pendingTimesyncTargetComponent) {
                    matchedPending = true;
                    pendingSentNs = m_pendingTimesyncSentNs;
                    m_havePendingTimesync = false;
                }
            }

            if (!matchedPending) {
                return;
            }

            const int64_t observedOffsetNs =
                static_cast<int64_t>((pendingSentNs + nowNs) / 2ULL) - tsync.tc1;
            const uint64_t rttUs64 = (nowNs > pendingSentNs) ? ((nowNs - pendingSentNs) / 1000ULL) : 0ULL;
            const uint32_t rttUs = (rttUs64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFu : static_cast<uint32_t>(rttUs64);

            int64_t filteredOffsetNs = observedOffsetNs;
            uint32_t sampleCount = 0;
            bool shouldLog = false;
            {
                std::lock_guard<std::mutex> lk(m_timesyncMtx);
                if (m_timesyncSampleCount == 0) {
                    m_timesyncEstimatedOffsetNs = observedOffsetNs;
                } else {
                    m_timesyncEstimatedOffsetNs =
                        (m_timesyncEstimatedOffsetNs * 7 + observedOffsetNs) / 8;
                }
                ++m_timesyncInboundResponseCount;
                m_timesyncLastRttUs = rttUs;
                filteredOffsetNs = m_timesyncEstimatedOffsetNs;
                sampleCount = ++m_timesyncSampleCount;
                const uint64_t nowUs = nowNs / 1000ULL;
                m_lastTimesyncActivityUs = nowUs;
                if (sampleCount <= 5 || (sampleCount % 20) == 0 ||
                    (nowUs > (m_lastTimesyncLogUs + 5000000ULL))) {
                    m_lastTimesyncLogUs = nowUs;
                    shouldLog = true;
                }
            }

            if (shouldLog) {
                printf("[timesync] samples=%u offset=%.3fms rtt=%.3fms px4=%u/%u\n",
                       sampleCount,
                       static_cast<double>(filteredOffsetNs) * 1e-6,
                       static_cast<double>(rttUs) * 1e-3,
                       static_cast<unsigned>(msg.sysid),
                       static_cast<unsigned>(msg.compid));
            }

            // Feed PX4 with a response-shaped sample even if it is not actively initiating
            // TIMESYNC requests on this link, so sync_stamp() can converge for EV timestamps.
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
            printf("[ACK] sys=%d, comp=%d cmd=%u result= %d(%s) progress=%d param2=%d\n",
                   int(msg.sysid), int(msg.compid), ack.command, int(ack.result), MavResultToStr(ack.result),
                   int(ack.progress), ack.result_param2);
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

    void MaybeRequestLocalPositionNedStream(uint8_t targetSystem, uint8_t targetComponent)
    {
        bool expected = false;
        if (!m_localPosStreamRequested.compare_exchange_strong(expected, true)) {
            return;
        }
        constexpr float kIntervalUs = 50000.0f;
        SendCommandLong(MAV_CMD_SET_MESSAGE_INTERVAL,
                        static_cast<float>(MAVLINK_MSG_ID_LOCAL_POSITION_NED),
                        kIntervalUs, 0, 0, 0, 0, 0,
                        targetSystem, targetComponent);
        printf("[mav] requested LOCAL_POSITION_NED @20Hz from sys=%d comp=%d\n",
               int(targetSystem), int(targetComponent));
    }

    void MaybeRequestDistanceSensorStream(uint8_t targetSystem, uint8_t targetComponent)
    {
        bool expected = false;
        if (!m_distanceSensorStreamRequested.compare_exchange_strong(expected, true)) {
            return;
        }
        constexpr float kIntervalUs = 50000.0f;
        SendCommandLong(MAV_CMD_SET_MESSAGE_INTERVAL,
                        static_cast<float>(MAVLINK_MSG_ID_DISTANCE_SENSOR),
                        kIntervalUs, 0, 0, 0, 0, 0,
                        targetSystem, targetComponent);
        printf("[mav] requested DISTANCE_SENSOR @20Hz from sys=%d comp=%d\n",
               int(targetSystem), int(targetComponent));
    }

    smartdrone::adapters::telemetry::MavlinkSerialTransport m_transport;
    uint8_t m_sysid;
    uint8_t m_compid;
    std::atomic<bool> m_streaming{false};
    std::thread m_streamThread;
    std::mutex m_spMtx;
    SetpointLocalNED m_spCurrent{};
    uint64_t m_streamPeriodUs{50000};
    std::atomic<bool> m_rxRunning{false};
    std::thread m_rxThread;
    std::atomic<bool> m_timesyncRunning{false};
    std::thread m_timesyncThread;
    std::mutex m_ackMtx;
    std::condition_variable m_ackCv;
    std::unordered_map<uint16_t, AckInfo> m_ackMap;
    std::mutex m_txMtx;
    mutable std::mutex m_timesyncMtx;
    mutable std::mutex m_flightModeMtx;
    mutable std::mutex m_localPosMtx;
    mutable std::mutex m_distanceSensorMtx;
    std::atomic<uint8_t> m_px4Sysid{1};
    std::atomic<uint8_t> m_px4Compid{1};
    std::atomic<bool> m_havePx4Heartbeat{false};
    std::atomic<bool> m_localPosStreamRequested{false};
    std::atomic<bool> m_distanceSensorStreamRequested{false};
    FlightModeInfo m_flightModeInfo{};
    bool m_haveFlightModeInfo{false};
    LocalPositionNed m_localPosNed{};
    bool m_haveLocalPosNed{false};
    DownwardDistanceSensor m_downwardDistanceSensor{};
    bool m_haveDownwardDistanceSensor{false};
    bool m_havePendingTimesync{false};
    int64_t m_pendingTimesyncTs1Ns{0};
    uint64_t m_pendingTimesyncSentNs{0};
    uint8_t m_pendingTimesyncTargetSystem{0};
    uint8_t m_pendingTimesyncTargetComponent{0};
    int64_t m_timesyncEstimatedOffsetNs{0};
    uint32_t m_timesyncLastRttUs{0};
    uint32_t m_timesyncSampleCount{0};
    uint32_t m_timesyncInboundRequestCount{0};
    uint32_t m_timesyncInboundResponseCount{0};
    uint64_t m_lastTimesyncActivityUs{0};
    uint64_t m_lastTimesyncLogUs{0};
};
