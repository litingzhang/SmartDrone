#include <jni.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "tlv_pack.hpp"
#include "udp_client.hpp"

static UdpClient g_udpClient;
static std::mutex g_mutex;
static std::atomic<uint32_t> g_seqCounter{1};

static uint32_t NowMs32()
{
    using namespace std::chrono;
    const auto ms = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    return static_cast<uint32_t>(ms & 0xFFFFFFFFu);
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_example_ZControl_NativeUdp_init(JNIEnv* env, jclass, jstring ip, jint port)
{
    const char* cStr = env->GetStringUTFChars(ip, nullptr);
    const std::string ipString = (cStr != nullptr) ? cStr : "";
    env->ReleaseStringUTFChars(ip, cStr);

    std::lock_guard<std::mutex> lock(g_mutex);
    return g_udpClient.Open(ipString, static_cast<uint16_t>(port)) ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_ZControl_NativeUdp_close(JNIEnv*, jclass)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_udpClient.Close();
}

extern "C" JNIEXPORT jint JNICALL
Java_com_example_ZControl_NativeUdp_sendCmd(JNIEnv*, jclass, jint cmd)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const uint32_t seq = g_seqCounter.fetch_add(1);
    const std::vector<uint8_t> frame = MakeFrame(
        1, static_cast<uint8_t>(cmd), 0, seq, NowMs32(), nullptr, 0);
    const bool ok = g_udpClient.Send(frame.data(), frame.size());
    return ok ? static_cast<jint>(seq) : static_cast<jint>(-1);
}

extern "C" JNIEXPORT jint JNICALL
Java_com_example_ZControl_NativeUdp_sendMove(
    JNIEnv*,
    jclass,
    jint frameType,
    jfloat x,
    jfloat y,
    jfloat z,
    jfloat yaw,
    jfloat maxV)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    const uint32_t seq = g_seqCounter.fetch_add(1);

    const std::vector<uint8_t> payload =
        MakeMovePayload(static_cast<uint8_t>(frameType), x, y, z, yaw, maxV);
    const std::vector<uint8_t> frame = MakeFrame(
        1,
        0x20,
        0,
        seq,
        NowMs32(),
        payload.data(),
        static_cast<uint16_t>(payload.size()));

    const bool ok = g_udpClient.Send(frame.data(), frame.size());
    return ok ? static_cast<jint>(seq) : static_cast<jint>(-1);
}

extern "C" JNIEXPORT jbyteArray JNICALL
Java_com_example_ZControl_NativeUdp_pollRecv(JNIEnv* env, jclass)
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
    env->SetByteArrayRegion(outArray, 0, recvLen, reinterpret_cast<jbyte*>(buffer));
    return outArray;
}
