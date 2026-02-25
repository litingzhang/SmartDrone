#pragma once

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <chrono>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <unordered_map>
#include <sstream>
#include <poll.h>
#include "common/mavlink.h"

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

    // diag indices in MAVLink 21-array (upper-triangular of 6x6):
    cov[0]  = varX;     // xx
    cov[6]  = varY;     // yy
    cov[11] = varZ;     // zz
    cov[15] = varRoll;  // rollroll
    cov[18] = varPitch; // pitchpitch
    cov[20] = varYaw;   // yawyaw
}

class MavlinkSerial {
public:
    struct Pose {
        // position in meters
        float x, y, z;
        // quaternion (w, x, y, z)
        float qw, qx, qy, qz;
    };

    explicit MavlinkSerial(const std::string& dev, int baud,
                           uint8_t sysid = 42, uint8_t compid = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY)
        : m_fd(-1), m_sysid(sysid), m_compid(compid), m_seq(0)
    {
        OpenSerial(dev, baud);
    }

    // ====== RX: start/stop ======
    void StartRx()
    {
        m_rxRunning.store(true);
        if (m_rxThread.joinable()) m_rxThread.join();
        m_rxThread = std::thread([this]() { this->RxLoop(); });
    }

    void StopRx()
    {
        m_rxRunning.store(false);
        if (m_rxThread.joinable()) m_rxThread.join();
    }

    // 自动学习到的 PX4 target（从 HEARTBEAT 得到）
    uint8_t GetTargetSystem() const { return m_px4Sysid.load(); }
    uint8_t GetTargetComponent() const { return m_px4Compid.load(); }

    // 等待某个 COMMAND 的 ACK（返回 true 表示等到了）
    // outResult: MAV_RESULT_*
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

    // customMode = (mainMode << 16) | (subMode << 24)
    static constexpr uint8_t PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;

    void SendCommandLong(uint16_t command,
                         float p1=0,float p2=0,float p3=0,float p4=0,float p5=0,float p6=0,float p7=0,
                         uint8_t targetSystem = 1, uint8_t targetComponent = 1,
                         uint8_t confirmation=0)
    {
        mavlink_message_t msg{};
        mavlink_msg_command_long_pack(
            m_sysid, m_compid, &msg,
            targetSystem, targetComponent,
            command, confirmation,
            p1,p2,p3,p4,p5,p6,p7
        );
        WriteMessage(msg);
    }

    bool SendCommandLongAndWaitAck(uint16_t command,
                                float p1=0,float p2=0,float p3=0,float p4=0,float p5=0,float p6=0,float p7=0,
                                int timeoutMs=800,
                                uint8_t targetSystem = 0, uint8_t targetComponent = 0,
                                uint8_t* outResult=nullptr)
    {
        if (targetSystem == 0) targetSystem = GetTargetSystem();
        if (targetComponent == 0) targetComponent = GetTargetComponent();

        {
            std::lock_guard<std::mutex> lk(m_ackMtx);
            m_ackMap.erase(command);
        }

        SendCommandLong(command, p1,p2,p3,p4,p5,p6,p7, targetSystem, targetComponent);

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

    // baseMode: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    // customMode: (OFFBOARD << 16)
    bool SetModeOffboard(int ackTimeoutMs = 800,
                        uint8_t targetSystem = 0, uint8_t targetComponent = 0)
    {
        const float baseMode = (float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        const uint32_t customMode = (uint32_t)PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16;

        uint8_t res = 255;
        bool got = SendCommandLongAndWaitAck(MAV_CMD_DO_SET_MODE,
                                            baseMode,
                                            (float)customMode,
                                            0,0,0,0,0,
                                            ackTimeoutMs,
                                            targetSystem, targetComponent,
                                            &res);

        return got && (res == MAV_RESULT_ACCEPTED);
    }

    // ARM / DISARM
    bool Arm(bool doArm, int ackTimeoutMs = 800, uint8_t targetSystem = 0, uint8_t targetComponent = 0)
    {
        uint8_t res = 255;
        bool got = SendCommandLongAndWaitAck(MAV_CMD_COMPONENT_ARM_DISARM, doArm ? 1.0f : 0.0f,
            0,0,0,0,0,0, ackTimeoutMs, targetSystem, targetComponent, &res);
        return got && (res == MAV_RESULT_ACCEPTED);
    }

    bool StartOffboardAndArm(double setpointHz = 20.0,
                            double heartbeatHz = 1.0,
                            int warmupMs = 800,
                            int ackTimeoutMs = 1000,
                            uint8_t targetSystem = 0, uint8_t targetComponent = 0)
    {
        // StartSetpointStreamHz(setpointHz);
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

    ~MavlinkSerial() {
        StopRx();
        StopSetpointStream();
        if (m_fd >= 0) close(m_fd);
    }
    struct SetpointLocalNED {
        float x = NAN, y = NAN, z = NAN;      // position (m) NED
        float vx = NAN, vy = NAN, vz = NAN;   // velocity (m/s) NED
        float ax = NAN, ay = NAN, az = NAN;   // accel (m/s^2) NED  (通常可以不用)
        float yaw = NAN;                      // rad
        float yawspeed = NAN;                 // rad/s
    };

    void SendSetPositionTargetLocalNed(uint32_t timeBootMs,
                                       const SetpointLocalNED& sp,
                                       uint8_t coordinateFrame = MAV_FRAME_LOCAL_NED)
    {
        // typeMask: 1 表示“忽略该字段”
        // bits:
        // 0..2: x,y,z
        // 3..5: vx,vy,vz
        // 6..8: ax,ay,az
        // 9:  force_set (不用)
        // 10: yaw
        // 11: yaw_rate
        uint16_t typeMask = 0;

        auto ign = [&](int bit){ typeMask |= (1u << bit); };

        if (!std::isfinite(sp.x)) ign(0);
        if (!std::isfinite(sp.y)) ign(1);
        if (!std::isfinite(sp.z)) ign(2);

        if (!std::isfinite(sp.vx)) ign(3);
        if (!std::isfinite(sp.vy)) ign(4);
        if (!std::isfinite(sp.vz)) ign(5);

        if (!std::isfinite(sp.ax)) ign(6);
        if (!std::isfinite(sp.ay)) ign(7);
        if (!std::isfinite(sp.az)) ign(8);

        // force_set 不用就忽略（bit9 = 1）
        ign(9);

        if (!std::isfinite(sp.yaw)) ign(10);
        if (!std::isfinite(sp.yawspeed)) ign(11);

        mavlink_message_t msg{};
        // GetTargetSystem / GetTargetComponent: 指向飞控(通常 1 / 1)
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
            std::isfinite(sp.yawspeed) ? sp.yawspeed : 0.0f
        );

        WriteMessage(msg);
    }
    void StartSetpointStreamHz(double hz = 20.0)
    {
        if (hz <= 0.0) hz = 20.0;
        m_streamPeriodUs = static_cast<uint64_t>(1e6 / hz);

        m_streaming.store(true);
        if (m_streamThread.joinable()) m_streamThread.join();

        m_streamThread = std::thread([this]() {
            while (this->m_streaming.load()) {
                const uint32_t tMs = TimeBootMs();

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
        sp.x = xN; sp.y = yE; sp.z = zD;
        sp.yaw = yawRad;
        UpdateStreamSetpoint(sp);
    }

    bool SendLand(int ackTimeoutMs = 800, uint8_t targetSystem = 1, uint8_t targetComponent = 1)
    {
        uint8_t res = 255;
        bool got = SendCommandLongAndWaitAck(MAV_CMD_NAV_LAND,
                                        0,0,0,0,0,0,0,
                                        ackTimeoutMs,
                                        targetSystem, targetComponent,
                                        &res);

        return got && (res == MAV_RESULT_ACCEPTED);
    }

    void SendOdometry(uint64_t timestampUs,
                    const Pose& poseNed,
                    uint8_t frameId = MAV_FRAME_LOCAL_NED,
                    uint8_t childFrameId = MAV_FRAME_BODY_FRD,
                    OdomQualityMode mode = OdomQualityMode::GOOD)
    {
        mavlink_message_t msg;

        const float vx = NAN, vy = NAN, vz = NAN;
        const float rollspeed = NAN, pitchspeed = NAN, yawspeed = NAN;

        float poseCov[21];
        float velCov[21];

        int8_t quality = 100;
        uint8_t estimatorType = 0;

        float q[4] = {poseNed.qw, poseNed.qx, poseNed.qy, poseNed.qz};

        if (mode == OdomQualityMode::GOOD) {
            for (int i = 0; i < 21; i++) { poseCov[i] = NAN; velCov[i] = NAN; }
            quality = 100;
        } else if (mode == OdomQualityMode::WEAK) {
            FillCovDiag21(poseCov,
                            2.25f, 2.25f, 2.25f,
                            0.12f, 0.12f, 0.12f);
            FillCovDiag21(velCov,
                            4.0f, 4.0f, 4.0f,
                            1.0f, 1.0f, 1.0f);
            quality = 20;
        } else {
            FillCovDiag21(poseCov,
                            1e4f, 1e4f, 1e4f,
                            10.0f, 10.0f, 10.0f);
            FillCovDiag21(velCov,
                            1e2f, 1e2f, 1e2f,
                            1e2f, 1e2f, 1e2f);
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
            0,
            estimatorType,
            quality
        );

        WriteMessage(msg);
    }

    static Pose EnuToNed(const Pose& pEnu) {
        Pose out{};
        out.x = pEnu.y;
        out.y = pEnu.x;
        out.z = -pEnu.z;

        const float s = 0.7071067811865476f;
        const float qrW = 0.0f, qrX = s, qrY = s, qrZ = 0.0f;

        auto qmul = [](float aw, float ax, float ay, float az,
                       float bw, float bx, float by, float bz,
                       float& ow, float& ox, float& oy, float& oz) {
            ow = aw*bw - ax*bx - ay*by - az*bz;
            ox = aw*bx + ax*bw + ay*bz - az*by;
            oy = aw*by - ax*bz + ay*bw + az*bx;
            oz = aw*bz + ax*by - ay*bx + az*bw;
        };

        auto qconj = [](float w,float x,float y,float z, float& ow,float& ox,float& oy,float& oz){
            ow=w; ox=-x; oy=-y; oz=-z;
        };

        // out_q = q_r * q_enu * q_r_conj
        float tW,tX,tY,tZ;
        qmul(qrW,qrX,qrY,qrZ, pEnu.qw,pEnu.qx,pEnu.qy,pEnu.qz, tW,tX,tY,tZ);

        float qrcW,qrcX,qrcY,qrcZ;
        qconj(qrW,qrX,qrY,qrZ, qrcW,qrcX,qrcY,qrcZ);

        qmul(tW,tX,tY,tZ, qrcW,qrcX,qrcY,qrcZ, out.qw,out.qx,out.qy,out.qz);

        NormalizeQuat(out.qw,out.qx,out.qy,out.qz);
        return out;
    }

    static void NormalizeQuat(float& w,float& x,float& y,float& z) {
        const float n = std::sqrt(w*w+x*x+y*y+z*z);
        if (n > 1e-9f) { w/=n; x/=n; y/=n; z/=n; }
        else { w=1; x=y=z=0; }
    }

private:
    std::atomic<bool> m_streaming{false};
    std::thread m_streamThread;
    std::mutex m_spMtx;
    SetpointLocalNED m_spCurrent{};
    uint64_t m_streamPeriodUs{50000}; // 20Hz

    static inline uint32_t TimeBootMs()
    {
        using namespace std::chrono;
        return (uint32_t)duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    }

    int m_fd;
    uint8_t m_sysid;
    uint8_t m_compid;
    uint8_t m_seq;

    static speed_t BaudToTermios(int baud) {
        switch (baud) {
            case 57600: return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
            case 921600: return B921600;
            default:
                throw std::runtime_error("Unsupported baud for termios: " + std::to_string(baud));
        }
    }

    void OpenSerial(const std::string& dev, int baud) {
        m_fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (m_fd < 0) {
            throw std::runtime_error("open(" + dev + ") failed: " + std::string(std::strerror(errno)));
        }

        termios tio{};
        if (tcgetattr(m_fd, &tio) != 0) {
            throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
        }

        // raw mode
        cfmakeraw(&tio);

        // 8N1
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;

        // enable receiver, ignore modem ctrl lines
        tio.c_cflag |= (CLOCAL | CREAD);

        // no flow control
        tio.c_cflag &= ~CRTSCTS;
        tio.c_iflag &= ~(IXON | IXOFF | IXANY);

        // set baud
        speed_t spd = BaudToTermios(baud);
        cfsetispeed(&tio, spd);
        cfsetospeed(&tio, spd);

        // non-blocking reads, but we mainly write
        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 0;

        if (tcsetattr(m_fd, TCSANOW, &tio) != 0) {
            throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
        }

        // set blocking for writes (optional)
        int flags = fcntl(m_fd, F_GETFL, 0);
        if (flags >= 0) fcntl(m_fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    void WriteMessage(const mavlink_message_t& msg) {
        std::lock_guard<std::mutex> txLock(m_txMtx);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        ssize_t n = ::write(m_fd, buf, len);
        if (n < 0) {
            printf("[mav] write failed: %d\n", std::strerror(errno));
        }
    }

    struct AckInfo {
        uint8_t result = 255; // MAV_RESULT (0..), 255=unknown
        uint8_t progress = 0; // MAVLink2 才有，common v2 支持
        int32_t resultParam2 = 0;
        std::chrono::steady_clock::time_point t;
    };

    std::atomic<bool> m_rxRunning{false};
    std::thread m_rxThread;

    std::mutex m_ackMtx;
    std::condition_variable m_ackCv;
    std::unordered_map<uint16_t, AckInfo> m_ackMap;
    std::mutex m_txMtx;

    std::atomic<uint8_t> m_px4Sysid{1};
    std::atomic<uint8_t> m_px4Compid{1};

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

    void RxLoop()
    {
        mavlink_message_t msg{};
        mavlink_status_t status{};

        // 确保 m_fd 可读：用 poll，避免 busy loop
        pollfd pfd{};
        pfd.fd = m_fd;
        pfd.events = POLLIN;

        while (m_rxRunning.load()) {
            int pr = ::poll(&pfd, 1, 200); // 200ms
            if (pr <= 0) continue;
            if (!(pfd.revents & POLLIN)) continue;

            uint8_t buf[512];
            ssize_t n = ::read(m_fd, buf, sizeof(buf));
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
        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            mavlink_heartbeat_t hb{};
            mavlink_msg_heartbeat_decode(&msg, &hb);

            if (hb.autopilot != MAV_AUTOPILOT_INVALID) {
                m_px4Sysid.store(msg.sysid);
                m_px4Compid.store(msg.compid);
            }
            return;
        }

        if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
            mavlink_command_ack_t ack{};
            mavlink_msg_command_ack_decode(&msg, &ack);

            AckInfo info;
            info.result = ack.result;
            info.progress = ack.progress;          // mavlink2 才有效
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

            // st.text 不是必然以 '\0' 结尾，安全处理一下
            char text[sizeof(st.text) + 1];
            std::memcpy(text, st.text, sizeof(st.text));
            text[sizeof(st.text)] = '\0';

            printf("[STATUSTEXT] sev=%d %s\n", int(st.severity), text);
            return;
        }
    }
};

// Helper: monotonic microseconds
static inline uint64_t MonoTimeUs() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}
