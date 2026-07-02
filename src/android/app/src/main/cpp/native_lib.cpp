#include <jni.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include "tlv_pack.h"
#include "udp_client.h"

static UdpClient g_udpClient;
static std::mutex g_mutex;
static std::atomic<uint32_t> g_seqCounter{1};
static constexpr uint8_t CMD_MOVE = 0x20;
static constexpr uint8_t CMD_RUNTIME_MODE = 0x30;
static constexpr uint8_t CMD_RUNTIME_CONFIG = 0x31;
static constexpr uint8_t CMD_GET_CAPABILITIES = 0x33;
static constexpr uint8_t CMD_GET_CONFIG = 0x34;
static constexpr uint8_t CMD_HEARTBEAT = 0xF5;
static constexpr uint8_t MOVE_FLAG_VELOCITY = 0x01;
static constexpr uint8_t MOVE_FLAG_RC_JOYSTICK = 0x02;
static constexpr uint8_t RUNTIME_CFG_FLAG_SEND_IMAGE = 0x01;
static constexpr uint8_t RUNTIME_CFG_FLAG_SEND_FEATURE = 0x02;
static constexpr uint8_t RUNTIME_CFG_FLAG_SEND_MAP = 0x04;
static constexpr uint16_t RUNTIME_MODE_PAYLOAD_LEN = 1;
static constexpr uint16_t RUNTIME_CONFIG_PAYLOAD_LEN = 141;

struct RuntimeConfigJniArgs {
    jint exposureUs{};
    jfloat gain{};
    jint pairMs{};
    jint slamFps{};
    jint slamMode{};
    jint sensorMode{};
    jboolean sendImage{};
    jboolean sendFeature{};
    jboolean sendMap{};
    jboolean autoExposure{};
    jboolean useCustomTbc{};
    jfloat tbcTx{};
    jfloat tbcTy{};
    jfloat tbcTz{};
    jfloat tbcRollDeg{};
    jfloat tbcPitchDeg{};
    jfloat tbcYawDeg{};
    jint orbNFeatures{};
    jfloat orbScaleFactor{};
    jint orbNLevels{};
    jint orbIniThFAST{};
    jint orbMinThFAST{};
    jint featureFrontend{};
    jint superpointTopK{};
    jint superpointMaxPoints{};
    jint superpointInputMaxWidth{};
    jint superpointInputMaxHeight{};
    jint lkPerFrameAcceleration{};
    jint orbAcceleration{};
    jint slamBackend{};
    jboolean avoidanceEnabled{};
    jboolean avoidanceHoldOnStaleCloud{};
    jfloat avoidanceRadiusM{};
    jfloat avoidanceLookaheadM{};
    jfloat avoidanceSpeedLookaheadS{};
    jfloat avoidanceNearFieldRadiusM{};
    jint avoidanceMaxPointAgeMs{};
    jint avoidanceMinCloudPoints{};
    jint avoidanceMinBlockingPoints{};
    jint px4PoseOutputMode{};
};

static uint32_t NowMs32()
{
    using namespace std::chrono;
    const auto ms = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    return static_cast<uint32_t>(ms & 0xFFFFFFFFu);
}

static void WriteF32LeAt(std::vector<uint8_t> &payload, size_t offset, float value)
{
    if (offset + 4 > payload.size()) {
        return;
    }
    uint32_t raw = 0;
    std::memcpy(&raw, &value, sizeof(float));
    payload[offset + 0] = static_cast<uint8_t>(raw & 0xFFu);
    payload[offset + 1] = static_cast<uint8_t>((raw >> 8) & 0xFFu);
    payload[offset + 2] = static_cast<uint8_t>((raw >> 16) & 0xFFu);
    payload[offset + 3] = static_cast<uint8_t>((raw >> 24) & 0xFFu);
}

static std::vector<uint8_t> BuildFrame(uint8_t cmd, uint8_t flags, uint32_t seq,
                                       const uint8_t *payload, uint16_t len)
{
    return MakeFrame({.ver = 1,
                      .cmd = cmd,
                      .flags = flags,
                      .seq = seq,
                      .tMs = NowMs32(),
                      .payload = payload,
                      .len = len});
}

static jint SendFrame(uint8_t cmd, uint8_t flags, const uint8_t *payload,
                      uint16_t len)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const uint32_t seq = g_seqCounter.fetch_add(1);
    const std::vector<uint8_t> frame = BuildFrame(cmd, flags, seq, payload, len);
    const bool ok = g_udpClient.Send(frame.data(), frame.size());
    return ok ? static_cast<jint>(seq) : static_cast<jint>(-1);
}

static jint SendMoveFrame(uint8_t flags, const MovePayloadValues &values)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const uint32_t seq = g_seqCounter.fetch_add(1);
    const std::vector<uint8_t> payload = MakeMovePayload(values);
    const std::vector<uint8_t> frame =
        BuildFrame(CMD_MOVE, flags, seq, payload.data(),
                   static_cast<uint16_t>(payload.size()));
    const bool ok = g_udpClient.Send(frame.data(), frame.size());
    return ok ? static_cast<jint>(seq) : static_cast<jint>(-1);
}

static uint8_t RuntimeStreamFlags(const RuntimeConfigJniArgs &args)
{
    uint8_t streamFlags = 0;
    if (args.sendImage == JNI_TRUE) {
        streamFlags |= RUNTIME_CFG_FLAG_SEND_IMAGE;
    }
    if (args.sendFeature == JNI_TRUE) {
        streamFlags |= RUNTIME_CFG_FLAG_SEND_FEATURE;
    }
    if (args.sendMap == JNI_TRUE) {
        streamFlags |= RUNTIME_CFG_FLAG_SEND_MAP;
    }
    return streamFlags;
}

static void WriteRuntimeConfigPrefix(std::vector<uint8_t> &payload,
                                     const RuntimeConfigJniArgs &args)
{
    WriteU32Le(payload, static_cast<uint32_t>(args.exposureUs));
    WriteF32Le(payload, static_cast<float>(args.gain));
    payload.push_back(static_cast<uint8_t>(args.sensorMode));
    payload.push_back(RuntimeStreamFlags(args));
    WriteU16Le(payload, static_cast<uint16_t>(args.pairMs > 0 ? args.pairMs : 0));
    payload.resize(RUNTIME_CONFIG_PAYLOAD_LEN, 0);
}

static void WriteRuntimeConfigCore(std::vector<uint8_t> &payload,
                                   const RuntimeConfigJniArgs &args)
{
    const uint16_t slamFpsValue =
        static_cast<uint16_t>(args.slamFps > 0 ? args.slamFps : 0);
    payload[40] = static_cast<uint8_t>(slamFpsValue & 0xFF);
    payload[41] = static_cast<uint8_t>((slamFpsValue >> 8) & 0xFF);
    payload[42] = static_cast<uint8_t>(args.slamMode);
    payload[43] = static_cast<uint8_t>(args.autoExposure == JNI_TRUE ? 1 : 0);
    payload[44] = static_cast<uint8_t>(args.useCustomTbc == JNI_TRUE ? 1 : 0);
}

static void WriteRuntimeConfigTbc(std::vector<uint8_t> &payload,
                                  const RuntimeConfigJniArgs &args)
{
    WriteF32LeAt(payload, 45, args.tbcTx);
    WriteF32LeAt(payload, 49, args.tbcTy);
    WriteF32LeAt(payload, 53, args.tbcTz);
    WriteF32LeAt(payload, 57, args.tbcPitchDeg);
    WriteF32LeAt(payload, 61, args.tbcRollDeg);
    WriteF32LeAt(payload, 65, args.tbcYawDeg);
}

static void WriteRuntimeConfigOrb(std::vector<uint8_t> &payload,
                                  const RuntimeConfigJniArgs &args)
{
    WriteF32LeAt(payload, 69, static_cast<float>(args.orbNFeatures));
    WriteF32LeAt(payload, 73, args.orbScaleFactor);
    WriteF32LeAt(payload, 77, static_cast<float>(args.orbNLevels));
    WriteF32LeAt(payload, 81, static_cast<float>(args.orbIniThFAST));
    WriteF32LeAt(payload, 85, static_cast<float>(args.orbMinThFAST));
}

static void WriteRuntimeConfigFeature(std::vector<uint8_t> &payload,
                                      const RuntimeConfigJniArgs &args)
{
    payload[89] = static_cast<uint8_t>(args.featureFrontend);
    WriteF32LeAt(payload, 90, static_cast<float>(args.superpointTopK));
    WriteF32LeAt(payload, 94, static_cast<float>(args.superpointMaxPoints));
    WriteF32LeAt(payload, 98,
                 static_cast<float>(args.superpointInputMaxWidth));
    WriteF32LeAt(payload, 102,
                 static_cast<float>(args.superpointInputMaxHeight));
    payload[106] = 0;
    payload[107] = static_cast<uint8_t>(args.lkPerFrameAcceleration);
    payload[108] = static_cast<uint8_t>(args.orbAcceleration);
    payload[109] = static_cast<uint8_t>(args.slamBackend);
}

static void WriteRuntimeConfigAvoidance(std::vector<uint8_t> &payload,
                                        const RuntimeConfigJniArgs &args)
{
    payload[110] = static_cast<uint8_t>(args.avoidanceEnabled == JNI_TRUE ? 1 : 0);
    payload[111] =
        static_cast<uint8_t>(args.avoidanceHoldOnStaleCloud == JNI_TRUE ? 1 : 0);
    WriteF32LeAt(payload, 112, args.avoidanceRadiusM);
    WriteF32LeAt(payload, 116, args.avoidanceLookaheadM);
    WriteF32LeAt(payload, 120, args.avoidanceSpeedLookaheadS);
    WriteF32LeAt(payload, 124, args.avoidanceNearFieldRadiusM);
    WriteF32LeAt(payload, 128, static_cast<float>(args.avoidanceMaxPointAgeMs));
    WriteF32LeAt(payload, 132, static_cast<float>(args.avoidanceMinCloudPoints));
    WriteF32LeAt(payload, 136,
                 static_cast<float>(args.avoidanceMinBlockingPoints));
}

static void WriteRuntimeConfigPx4(std::vector<uint8_t> &payload,
                                  const RuntimeConfigJniArgs &args)
{
    payload[140] = static_cast<uint8_t>(args.px4PoseOutputMode);
}

static std::vector<uint8_t> MakeRuntimeConfigPayload(
    const RuntimeConfigJniArgs &args)
{
    std::vector<uint8_t> payload;
    payload.reserve(RUNTIME_CONFIG_PAYLOAD_LEN);
    WriteRuntimeConfigPrefix(payload, args);
    WriteRuntimeConfigCore(payload, args);
    WriteRuntimeConfigTbc(payload, args);
    WriteRuntimeConfigOrb(payload, args);
    WriteRuntimeConfigFeature(payload, args);
    WriteRuntimeConfigAvoidance(payload, args);
    WriteRuntimeConfigPx4(payload, args);
    return payload;
}

extern "C" JNIEXPORT jboolean JNICALL Java_com_example_smartdrone_NativeUdp_init(JNIEnv *env, jclass, jstring ip,
                                                                                 jint cmdPort, jint videoPort)
{
    const char *cStr = env->GetStringUTFChars(ip, nullptr);
    const std::string ipString = (cStr != nullptr) ? cStr : "";
    env->ReleaseStringUTFChars(ip, cStr);

    std::lock_guard<std::mutex> lock(g_mutex);
    return g_udpClient.Open(ipString, static_cast<uint16_t>(cmdPort), static_cast<uint16_t>(videoPort)) ? JNI_TRUE
                                                                                                        : JNI_FALSE;
}

extern "C" JNIEXPORT void JNICALL Java_com_example_smartdrone_NativeUdp_close(JNIEnv *, jclass)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_udpClient.Close();
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendCmd(JNIEnv *, jclass, jint cmd)
{
    return SendFrame(static_cast<uint8_t>(cmd), 0, nullptr, 0);
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendMove(JNIEnv *, jclass, jint frameType,
                                                                                 jfloat x, jfloat y, jfloat z,
                                                                                 jfloat yaw, jfloat maxV)
{
    return SendMoveFrame(0, {static_cast<uint8_t>(frameType), x, y, z, yaw, maxV});
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendMoveVelocity(JNIEnv *, jclass,
                                                                                         jint frameType, jfloat vx,
                                                                                         jfloat vy, jfloat vz,
                                                                                         jfloat yawRate, jfloat maxV)
{
    return SendMoveFrame(MOVE_FLAG_VELOCITY,
                         {static_cast<uint8_t>(frameType), vx, vy, vz,
                          yawRate, maxV});
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendMoveRcJoystick(
    JNIEnv *, jclass, jint frameType, jfloat throttle, jfloat yaw, jfloat pitch, jfloat roll, jfloat maxV)
{
    return SendMoveFrame(MOVE_FLAG_RC_JOYSTICK,
                         {static_cast<uint8_t>(frameType), throttle, yaw, pitch,
                          roll, maxV});
}

extern "C" JNIEXPORT jbyteArray JNICALL Java_com_example_smartdrone_NativeUdp_pollRecv(JNIEnv *env, jclass)
{
    uint8_t buffer[2048]{};
    int recvLen = 0;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        recvLen = g_udpClient.Recv(buffer, sizeof(buffer));
    }
    if (recvLen <= 0) {
        return nullptr;
    }

    jbyteArray outArray = env->NewByteArray(recvLen);
    env->SetByteArrayRegion(outArray, 0, recvLen, reinterpret_cast<jbyte *>(buffer));
    return outArray;
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendRuntimeMode(JNIEnv *, jclass, jint mode)
{
    const uint8_t payload[RUNTIME_MODE_PAYLOAD_LEN] = {static_cast<uint8_t>(mode)};
    return SendFrame(CMD_RUNTIME_MODE, 0, payload, RUNTIME_MODE_PAYLOAD_LEN);
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendGetCapabilities(JNIEnv *, jclass)
{
    return SendFrame(CMD_GET_CAPABILITIES, 0, nullptr, 0);
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendGetConfig(JNIEnv *, jclass)
{
    return SendFrame(CMD_GET_CONFIG, 0, nullptr, 0);
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendHeartbeat(JNIEnv *, jclass)
{
    return SendFrame(CMD_HEARTBEAT, 0, nullptr, 0);
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendRuntimeConfig(
    JNIEnv *, jclass, jint exposureUs, jfloat gain, jint pairMs, jint slamFps, jint slamMode, jint sensorMode,
    jboolean sendImage, jboolean sendFeature, jboolean sendMap, jboolean autoExposure, jboolean useCustomTbc,
    jfloat tbcTx, jfloat tbcTy, jfloat tbcTz, jfloat tbcRollDeg, jfloat tbcPitchDeg, jfloat tbcYawDeg,
    jint orbNFeatures, jfloat orbScaleFactor, jint orbNLevels, jint orbIniThFAST, jint orbMinThFAST,
    jint featureFrontend, jint superpointTopK, jint superpointMaxPoints, jint superpointInputMaxWidth, jint superpointInputMaxHeight,
    jint lkPerFrameAcceleration, jint orbAcceleration, jint slamBackend,
    jboolean avoidanceEnabled, jboolean avoidanceHoldOnStaleCloud,
    jfloat avoidanceRadiusM, jfloat avoidanceLookaheadM,
    jfloat avoidanceSpeedLookaheadS, jfloat avoidanceNearFieldRadiusM,
    jint avoidanceMaxPointAgeMs, jint avoidanceMinCloudPoints,
    jint avoidanceMinBlockingPoints, jint px4PoseOutputMode)
{
    const RuntimeConfigJniArgs args{
        exposureUs, gain, pairMs, slamFps, slamMode, sensorMode, sendImage,
        sendFeature, sendMap, autoExposure, useCustomTbc, tbcTx, tbcTy, tbcTz,
        tbcRollDeg, tbcPitchDeg, tbcYawDeg, orbNFeatures, orbScaleFactor,
        orbNLevels, orbIniThFAST, orbMinThFAST, featureFrontend, superpointTopK,
        superpointMaxPoints, superpointInputMaxWidth, superpointInputMaxHeight,
        lkPerFrameAcceleration, orbAcceleration, slamBackend, avoidanceEnabled,
        avoidanceHoldOnStaleCloud, avoidanceRadiusM, avoidanceLookaheadM,
        avoidanceSpeedLookaheadS, avoidanceNearFieldRadiusM,
        avoidanceMaxPointAgeMs, avoidanceMinCloudPoints,
        avoidanceMinBlockingPoints, px4PoseOutputMode};
    const std::vector<uint8_t> payload = MakeRuntimeConfigPayload(args);
    return SendFrame(CMD_RUNTIME_CONFIG, 0, payload.data(),
                     static_cast<uint16_t>(payload.size()));
}
