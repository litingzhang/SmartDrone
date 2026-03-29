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

    CMD_MOVE = 0x20,
    CMD_RUNTIME_MODE = 0x30,
    CMD_RUNTIME_CONFIG = 0x31,
    CMD_CALIB_CLEAN = 0x32,

    CMD_ACK = 0xF0,
    CMD_STATE = 0xF1,
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
// RUNTIME_CONFIG payload v2 (len=42):
// exposureUs(u32le) gain(f32le) sensorMode(u8) streamFlags(u8) pairMs(u16le) reservedOrIp[30]
// reservedOrIp keeps backward compatibility with the legacy sender IP field.
// v2 additionally stores slamInputFps(u16le) at the tail bytes [40,41].
// legacy v1 (len=40) omitted pairMs and started reservedOrIp at byte 10.
constexpr uint16_t RUNTIME_CONFIG_PAYLOAD_LEN_LEGACY = 40;
constexpr uint16_t RUNTIME_CONFIG_PAYLOAD_LEN = 42;
constexpr uint16_t RUNTIME_CONFIG_PAIR_MS_OFFSET = 10;
constexpr uint16_t RUNTIME_CONFIG_IP_OFFSET = 12;
constexpr uint16_t RUNTIME_CONFIG_IP_LEN = 30;
constexpr uint16_t RUNTIME_CONFIG_SLAM_FPS_OFFSET = 40;
constexpr uint8_t RUNTIME_CFG_FLAG_SEND_IMAGE = 0x01;
constexpr uint8_t RUNTIME_CFG_FLAG_SEND_FEATURE = 0x02;
constexpr uint8_t RUNTIME_CFG_FLAG_SEND_MAP = 0x04;
// STATE payload:
// runtimeMode(u8) trackingState(u8) resetCounter(u16le) resetMapCount(u16le)
// x/y/z/qw/qx/qy/qz (7 * f32le)
constexpr uint16_t STATE_POSE_PAYLOAD_LEN = 34;

enum RuntimeMode : uint8_t {
    RUNTIME_MODE_IDLE = 0,
    RUNTIME_MODE_SLAM = 1,
    RUNTIME_MODE_CALIB = 2,
};

enum RuntimeSensorMode : uint8_t {
    RUNTIME_SENSOR_STEREO = 0,
    RUNTIME_SENSOR_STEREO_IMU = 1,
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
