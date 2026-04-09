#pragma once

#include <cstdint>

constexpr uint8_t TLV_SYNC0 = 0xAA;
constexpr uint8_t TLV_SYNC1 = 0x55;
constexpr uint8_t TLV_VER = 1;

enum TlvCmd : uint8_t {
    CMD_PING = 0x01,

    CMD_ARM = 0x10,
    CMD_DISARM = 0x11,
    CMD_OFFBOARD = 0x12,
    CMD_HOLD = 0x13,
    CMD_LAND = 0x14,
    CMD_EMERGENCY_STOP = 0x15,
    CMD_POSITION = 0x16,

    CMD_MOVE = 0x20,
    CMD_RUNTIME_MODE = 0x30,
    CMD_RUNTIME_CONFIG = 0x31,
    CMD_CALIB_CLEAN = 0x32,
    CMD_GET_CAPABILITIES = 0x33,
    CMD_GET_CONFIG = 0x34,
    CMD_FORCE_RESTART = 0x35,

    CMD_ACK = 0xF0,
    CMD_STATE = 0xF1,
    CMD_CAPABILITIES = 0xF3,
    CMD_CONFIG = 0xF4,
    CMD_HEARTBEAT = 0xF5,
};

// MOVE payload layout legacy (len=21):
// frame(u8) a(f32le) b(f32le) c(f32le) d(f32le) maxV(f32le)
// flags bit0 == 0: a/b/c/d => x/y/z/yaw (position setpoint)
// flags bit0 == 1: a/b/c/d => vx/vy/vz/yawRate (velocity setpoint)
constexpr uint16_t MOVE_PAYLOAD_LEN = 21;
constexpr uint8_t MOVE_FLAG_VELOCITY = 0x01;
// MOVE payload layout rc joystick (len=21):
// frame(u8) throttle(f32le) yaw(f32le) pitch(f32le) roll(f32le) maxV(f32le)
constexpr uint16_t MOVE_RC_PAYLOAD_LEN = 21;
constexpr uint8_t MOVE_FLAG_RC_JOYSTICK = 0x02;
constexpr uint16_t RUNTIME_MODE_PAYLOAD_LEN = 1;
// RUNTIME_CONFIG payload v4 (len=44):
// exposureUs(u32le) gain(f32le) sensorMode(u8) streamFlags(u8) pairMs(u16le) reservedOrIp[30]
// reservedOrIp keeps backward compatibility with the legacy sender IP field.
// v2 additionally stores slamInputFps(u16le) at the tail bytes [40,41].
// v3 additionally stores slamOperationMode(u8) at byte [42].
// v4 additionally stores autoExposureEnabled(u8) at byte [43] (1=AE on, 0=manual exposure/gain).
// legacy v1 (len=40) omitted pairMs and started reservedOrIp at byte 10.
constexpr uint16_t RUNTIME_CONFIG_PAYLOAD_LEN_LEGACY = 40;
constexpr uint16_t RUNTIME_CONFIG_PAYLOAD_LEN_V2 = 42;
constexpr uint16_t RUNTIME_CONFIG_PAYLOAD_LEN_V3 = 43;
constexpr uint16_t RUNTIME_CONFIG_PAYLOAD_LEN = 44;
constexpr uint16_t RUNTIME_CONFIG_PAIR_MS_OFFSET = 10;
constexpr uint16_t RUNTIME_CONFIG_IP_OFFSET = 12;
constexpr uint16_t RUNTIME_CONFIG_IP_LEN = 30;
constexpr uint16_t RUNTIME_CONFIG_SLAM_FPS_OFFSET = 40;
constexpr uint16_t RUNTIME_CONFIG_SLAM_MODE_OFFSET = 42;
constexpr uint16_t RUNTIME_CONFIG_AE_OFFSET = 43;
constexpr uint8_t RUNTIME_CFG_FLAG_SEND_IMAGE = 0x01;
constexpr uint8_t RUNTIME_CFG_FLAG_SEND_FEATURE = 0x02;
constexpr uint8_t RUNTIME_CFG_FLAG_SEND_MAP = 0x04;
// STATE payload v4:
// runtimeMode(u8) slamMode(u8) trackingState(u8) armed(u8) resetCounter(u16le) resetMapCount(u16le)
// x/y/z/qw/qx/qy/qz (7 * f32le) px4MainMode(u8) px4SubMode(u8)
// legacy v3 omitted px4 mode fields and was 36 bytes; legacy v2 omitted armed and was 35 bytes;
// legacy v1 omitted slamMode and was 34 bytes.
constexpr uint16_t STATE_POSE_PAYLOAD_LEN_LEGACY = 34;
constexpr uint16_t STATE_POSE_PAYLOAD_LEN_V2 = 35;
constexpr uint16_t STATE_POSE_PAYLOAD_LEN_V3 = 36;
constexpr uint16_t STATE_POSE_PAYLOAD_LEN = 38;

enum RuntimeMode : uint8_t {
    RUNTIME_MODE_IDLE = 0,
    RUNTIME_MODE_SLAM = 1,
    RUNTIME_MODE_CALIB = 2,
};

enum RuntimeSensorMode : uint8_t {
    RUNTIME_SENSOR_STEREO = 0,
    RUNTIME_SENSOR_STEREO_IMU = 1,
    RUNTIME_SENSOR_MONO = 2,
    RUNTIME_SENSOR_MONO_IMU = 3,
};

enum RuntimeSlamMode : uint8_t {
    RUNTIME_SLAM_MODE_MAPPING = 0,
    RUNTIME_SLAM_MODE_LOCALIZATION = 1,
    RUNTIME_SLAM_MODE_RELOCALIZATION = 2,
    RUNTIME_SLAM_MODE_TRACKING_ONLY = 3,
    RUNTIME_SLAM_MODE_AUTO = 4,
};

enum FrameType : uint8_t {
    FRAME_MAP = 0,
    FRAME_ENU = 1,
    FRAME_NED = 2,
};

// ACK payload (len=9):
// ackCmd(u8) ackSeq(u32) status(i16) reserved(u16)
constexpr uint16_t ACK_PAYLOAD_LEN = 9;

enum AckStatus : int16_t {
    ACK_OK = 0,
    ACK_E_BAD_CRC = -1,
    ACK_E_BAD_LEN = -2,
    ACK_E_BAD_ARGS = -3,
    ACK_E_BAD_STATE = -4,
    ACK_E_UNKNOWN = -5,
    ACK_E_INTERNAL = -6,
};
