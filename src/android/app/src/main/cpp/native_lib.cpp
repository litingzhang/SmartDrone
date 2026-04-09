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
static constexpr uint16_t RUNTIME_CONFIG_PAYLOAD_LEN = 44;

static uint32_t NowMs32()
{
    using namespace std::chrono;
    const auto ms = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    return static_cast<uint32_t>(ms & 0xFFFFFFFFu);
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
    std::lock_guard<std::mutex> lock(g_mutex);
    const uint32_t seq = g_seqCounter.fetch_add(1);
    const std::vector<uint8_t> frame = MakeFrame(1, static_cast<uint8_t>(cmd), 0, seq, NowMs32(), nullptr, 0);
    const bool ok = g_udpClient.Send(frame.data(), frame.size());
    return ok ? static_cast<jint>(seq) : static_cast<jint>(-1);
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendMove(JNIEnv *, jclass, jint frameType,
                                                                                 jfloat x, jfloat y, jfloat z,
                                                                                 jfloat yaw, jfloat maxV)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const uint32_t seq = g_seqCounter.fetch_add(1);

    const std::vector<uint8_t> payload = MakeMovePayload(static_cast<uint8_t>(frameType), x, y, z, yaw, maxV);
    const std::vector<uint8_t> frame =
        MakeFrame(1, CMD_MOVE, 0, seq, NowMs32(), payload.data(), static_cast<uint16_t>(payload.size()));

    const bool ok = g_udpClient.Send(frame.data(), frame.size());
    return ok ? static_cast<jint>(seq) : static_cast<jint>(-1);
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendMoveVelocity(JNIEnv *, jclass,
                                                                                         jint frameType, jfloat vx,
                                                                                         jfloat vy, jfloat vz,
                                                                                         jfloat yawRate, jfloat maxV)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const uint32_t seq = g_seqCounter.fetch_add(1);

    const std::vector<uint8_t> payload = MakeMovePayload(static_cast<uint8_t>(frameType), vx, vy, vz, yawRate, maxV);
    const std::vector<uint8_t> frame = MakeFrame(1, CMD_MOVE, MOVE_FLAG_VELOCITY, seq, NowMs32(), payload.data(),
                                                 static_cast<uint16_t>(payload.size()));

    const bool ok = g_udpClient.Send(frame.data(), frame.size());
    return ok ? static_cast<jint>(seq) : static_cast<jint>(-1);
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendMoveRcJoystick(
    JNIEnv *, jclass, jint frameType, jfloat throttle, jfloat yaw, jfloat pitch, jfloat roll, jfloat maxV)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const uint32_t seq = g_seqCounter.fetch_add(1);

    const std::vector<uint8_t> payload =
        MakeMoveRcPayload(static_cast<uint8_t>(frameType), throttle, yaw, pitch, roll, maxV);
    const std::vector<uint8_t> frame = MakeFrame(1, CMD_MOVE, MOVE_FLAG_RC_JOYSTICK, seq, NowMs32(), payload.data(),
                                                 static_cast<uint16_t>(payload.size()));

    const bool ok = g_udpClient.Send(frame.data(), frame.size());
    return ok ? static_cast<jint>(seq) : static_cast<jint>(-1);
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
    std::lock_guard<std::mutex> lock(g_mutex);
    const uint32_t seq = g_seqCounter.fetch_add(1);
    const uint8_t payload[RUNTIME_MODE_PAYLOAD_LEN] = {static_cast<uint8_t>(mode)};
    const std::vector<uint8_t> frame =
        MakeFrame(1, CMD_RUNTIME_MODE, 0, seq, NowMs32(), payload, RUNTIME_MODE_PAYLOAD_LEN);
    const bool ok = g_udpClient.Send(frame.data(), frame.size());
    return ok ? static_cast<jint>(seq) : static_cast<jint>(-1);
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendGetCapabilities(JNIEnv *, jclass)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const uint32_t seq = g_seqCounter.fetch_add(1);
    const std::vector<uint8_t> frame = MakeFrame(1, CMD_GET_CAPABILITIES, 0, seq, NowMs32(), nullptr, 0);
    const bool ok = g_udpClient.Send(frame.data(), frame.size());
    return ok ? static_cast<jint>(seq) : static_cast<jint>(-1);
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendGetConfig(JNIEnv *, jclass)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const uint32_t seq = g_seqCounter.fetch_add(1);
    const std::vector<uint8_t> frame = MakeFrame(1, CMD_GET_CONFIG, 0, seq, NowMs32(), nullptr, 0);
    const bool ok = g_udpClient.Send(frame.data(), frame.size());
    return ok ? static_cast<jint>(seq) : static_cast<jint>(-1);
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendHeartbeat(JNIEnv *, jclass)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const uint32_t seq = g_seqCounter.fetch_add(1);
    const std::vector<uint8_t> frame = MakeFrame(1, CMD_HEARTBEAT, 0, seq, NowMs32(), nullptr, 0);
    const bool ok = g_udpClient.Send(frame.data(), frame.size());
    return ok ? static_cast<jint>(seq) : static_cast<jint>(-1);
}

extern "C" JNIEXPORT jint JNICALL Java_com_example_smartdrone_NativeUdp_sendRuntimeConfig(
    JNIEnv *, jclass, jint exposureUs, jfloat gain, jint pairMs, jint slamFps, jint slamMode, jint sensorMode,
    jboolean sendImage, jboolean sendFeature, jboolean sendMap, jboolean autoExposure)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const uint32_t seq = g_seqCounter.fetch_add(1);
    std::vector<uint8_t> payload;
    payload.reserve(RUNTIME_CONFIG_PAYLOAD_LEN);
    WriteU32Le(payload, static_cast<uint32_t>(exposureUs));
    WriteF32Le(payload, static_cast<float>(gain));
    payload.push_back(static_cast<uint8_t>(sensorMode));
    uint8_t streamFlags = 0;
    if (sendImage == JNI_TRUE)
        streamFlags |= RUNTIME_CFG_FLAG_SEND_IMAGE;
    if (sendFeature == JNI_TRUE)
        streamFlags |= RUNTIME_CFG_FLAG_SEND_FEATURE;
    if (sendMap == JNI_TRUE)
        streamFlags |= RUNTIME_CFG_FLAG_SEND_MAP;
    payload.push_back(streamFlags);
    WriteU16Le(payload, static_cast<uint16_t>(pairMs > 0 ? pairMs : 0));
    payload.resize(RUNTIME_CONFIG_PAYLOAD_LEN, 0);
    const uint16_t slamFpsValue = static_cast<uint16_t>(slamFps > 0 ? slamFps : 0);
    payload[40] = static_cast<uint8_t>(slamFpsValue & 0xFF);
    payload[41] = static_cast<uint8_t>((slamFpsValue >> 8) & 0xFF);
    payload[42] = static_cast<uint8_t>(slamMode);
    payload[43] = static_cast<uint8_t>(autoExposure == JNI_TRUE ? 1 : 0);

    const std::vector<uint8_t> frame =
        MakeFrame(1, CMD_RUNTIME_CONFIG, 0, seq, NowMs32(), payload.data(), static_cast<uint16_t>(payload.size()));
    const bool ok = g_udpClient.Send(frame.data(), frame.size());
    return ok ? static_cast<jint>(seq) : static_cast<jint>(-1);
}
