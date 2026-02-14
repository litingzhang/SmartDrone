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

#include "common/mavlink.h"

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
        : fd_(-1), sysid_(sysid), compid_(compid), seq_(0)
    {
        openSerial(dev, baud);
    }

    ~MavlinkSerial() {
        if (fd_ >= 0) close(fd_);
    }

    // Optional: send heartbeat periodically (1Hz is fine)
    void sendHeartbeat() {
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(
            sysid_, compid_, &msg,
            MAV_TYPE_ONBOARD_CONTROLLER,          // or MAV_TYPE_VISION_SYSTEM
            MAV_AUTOPILOT_INVALID,                // not an autopilot
            0,                                    // base_mode
            0,                                    // custom_mode
            MAV_STATE_ACTIVE
        );
        writeMessage(msg);
    }

    // Send ODOMETRY to PX4
    // timestamp_us: monotonic or sensor time in microseconds
    // pose_ned: pose expressed in NED (meters) and quaternion in NED frame
    // child_frame_id: MAV_FRAME_BODY_FRD if you are sending body frame; for world pose use MAV_FRAME_LOCAL_NED
    void sendOdometry(uint64_t timestamp_us, const Pose& pose_ned,
                      uint8_t frame_id = MAV_FRAME_LOCAL_NED,
                      uint8_t child_frame_id = MAV_FRAME_BODY_FRD)
    {
        mavlink_message_t msg;

        // Minimal: we send pose; leave velocities as NaN (PX4 will accept pose-only visual odom).
        float vx = NAN, vy = NAN, vz = NAN;
        float rollspeed = NAN, pitchspeed = NAN, yawspeed = NAN;

        // Covariances: set -1 for "unknown" in MAVLink ODOMETRY? Actually ODOMETRY uses float[21].
        // We'll set to NAN to indicate unknown (common practice). PX4 handles it.
        float pose_cov[21];
        float vel_cov[21];
        for (int i = 0; i < 21; i++) { pose_cov[i] = NAN; vel_cov[i] = NAN; }

        // q order in MAVLink: [w, x, y, z]
        float q[4] = {pose_ned.qw, pose_ned.qx, pose_ned.qy, pose_ned.qz};

        mavlink_msg_odometry_pack(
            sysid_, compid_, &msg,
            timestamp_us,
            frame_id,
            child_frame_id,
            pose_ned.x, pose_ned.y, pose_ned.z,
            q,
            vx, vy, vz,
            rollspeed, pitchspeed, yawspeed,
            pose_cov,
            vel_cov,
            0,      // reset_counter
            0,      // estimator_type (0=unknown)
            (int8_t)100 // quality: 0..100, 或 -1 表示未知（看你版本定义）
        );

        writeMessage(msg);
    }

    // Helpers: if your SLAM outputs ENU (x=E,y=N,z=Up), convert to NED (x=N,y=E,z=Down)
    // Also quaternion transform for ENU->NED.
    // IMPORTANT: quaternion frame transform is subtle; this is a standard ENU->NED rotation:
    // R_ned = R_enu2ned * R_enu * R_enu2ned^T
    // We implement via quaternion multiplication.
    static Pose enuToNed(const Pose& p_enu) {
        Pose out{};
        // position: ENU -> NED
        // ENU: (E, N, U)
        // NED: (N, E, D)
        out.x = p_enu.y;
        out.y = p_enu.x;
        out.z = -p_enu.z;

        // quaternion transform
        // Rotation from ENU to NED can be represented by quaternion q_r.
        // One common mapping is a +90deg about Z then 180deg about X (depends on conventions).
        // We use a standard fixed transform:
        // ENU basis to NED basis: [x_n y_n z_n] = [ y_e, x_e, -z_e ]
        // Equivalent rotation: q_r = (0, 1/sqrt(2), 1/sqrt(2), 0) ??? (varies)
        //
        // To avoid “looks right but wrong”, we provide a conservative option:
        // If you are unsure, initially send position-only and keep attitude as identity.
        //
        // Here is a widely used ENU->NED quaternion:
        // q_r = [0, 1/sqrt(2), 1/sqrt(2), 0]  (w,x,y,z)
        const float s = 0.7071067811865476f;
        const float qr_w = 0.0f, qr_x = s, qr_y = s, qr_z = 0.0f;

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
        float t_w,t_x,t_y,t_z;
        qmul(qr_w,qr_x,qr_y,qr_z, p_enu.qw,p_enu.qx,p_enu.qy,p_enu.qz, t_w,t_x,t_y,t_z);

        float qrc_w,qrc_x,qrc_y,qrc_z;
        qconj(qr_w,qr_x,qr_y,qr_z, qrc_w,qrc_x,qrc_y,qrc_z);

        qmul(t_w,t_x,t_y,t_z, qrc_w,qrc_x,qrc_y,qrc_z, out.qw,out.qx,out.qy,out.qz);

        normalizeQuat(out.qw,out.qx,out.qy,out.qz);
        return out;
    }

    static void normalizeQuat(float& w,float& x,float& y,float& z) {
        const float n = std::sqrt(w*w+x*x+y*y+z*z);
        if (n > 1e-9f) { w/=n; x/=n; y/=n; z/=n; }
        else { w=1; x=y=z=0; }
    }

private:
    int fd_;
    uint8_t sysid_;
    uint8_t compid_;
    uint8_t seq_;

    static speed_t baudToTermios(int baud) {
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

    void openSerial(const std::string& dev, int baud) {
        fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            throw std::runtime_error("open(" + dev + ") failed: " + std::string(std::strerror(errno)));
        }

        termios tio{};
        if (tcgetattr(fd_, &tio) != 0) {
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
        speed_t spd = baudToTermios(baud);
        cfsetispeed(&tio, spd);
        cfsetospeed(&tio, spd);

        // non-blocking reads, but we mainly write
        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
            throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
        }

        // set blocking for writes (optional)
        int flags = fcntl(fd_, F_GETFL, 0);
        if (flags >= 0) fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
    }

    void writeMessage(const mavlink_message_t& msg) {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        ssize_t n = ::write(fd_, buf, len);
        if (n < 0) {
            std::cerr << "[mav] write failed: " << std::strerror(errno) << "\n";
        }
    }
};

// Helper: monotonic microseconds
static inline uint64_t mono_time_us() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}
