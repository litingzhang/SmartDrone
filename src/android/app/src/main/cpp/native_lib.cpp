#include <jni.h>
#include <cstdint>
#include <string>
#include <vector>
#include <chrono>
#include <mutex>
#include <atomic>
#include <fcntl.h>

#include "udp_client.hpp"
#include "tlv_pack.hpp"

static UdpClient g_udp;
static std::mutex g_mtx;
static std::atomic<uint32_t> g_seq{1};

static uint32_t nowMs32()
{
    using namespace std::chrono;
    auto ms = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    return (uint32_t)(ms & 0xFFFFFFFFu);
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_example_ZControl_NativeUdp_init(JNIEnv* env, jclass, jstring ip, jint port)
{
    const char* cstr = env->GetStringUTFChars(ip, nullptr);
    std::string sip = cstr ? cstr : "";
    env->ReleaseStringUTFChars(ip, cstr);

    std::lock_guard<std::mutex> lk(g_mtx);
    return g_udp.open(sip, (uint16_t)port) ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT void JNICALL
Java_com_example_ZControl_NativeUdp_close(JNIEnv*, jclass)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    g_udp.close();
}

extern "C" JNIEXPORT jint JNICALL
Java_com_example_ZControl_NativeUdp_sendCmd(JNIEnv*, jclass, jint cmd)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    const uint32_t seq = g_seq.fetch_add(1);
    auto frame = makeFrame(1, (uint8_t)cmd, 0, seq, nowMs32(), nullptr, 0);
    bool ok = g_udp.send(frame.data(), frame.size());
    return ok ? (jint)seq : (jint)-1;
}

extern "C" JNIEXPORT jint JNICALL
Java_com_example_ZControl_NativeUdp_sendMove(JNIEnv*, jclass,
                                          jint frameType,
                                          jfloat x, jfloat y, jfloat z,
                                          jfloat yaw, jfloat maxV)
{
    std::lock_guard<std::mutex> lk(g_mtx);
    const uint32_t seq = g_seq.fetch_add(1);

    auto payload = makeMovePayload((uint8_t)frameType, x, y, z, yaw, maxV);
    auto frame = makeFrame(1, 0x20 /*CMD_MOVE*/, 0, seq, nowMs32(),
                           payload.data(), (uint16_t)payload.size());

    bool ok = g_udp.send(frame.data(), frame.size());
    return ok ? (jint)seq : (jint)-1;
}

// 非阻塞接收：返回 byte[]（可能为 null）
extern "C" JNIEXPORT jbyteArray JNICALL
Java_com_example_ZControl_NativeUdp_pollRecv(JNIEnv* env, jclass)
{
    uint8_t buf[2048];
    int n = 0;
    {
        std::lock_guard<std::mutex> lk(g_mtx);
        n = g_udp.recv(buf, sizeof(buf));
    }
    if (n <= 0) return nullptr;

    jbyteArray arr = env->NewByteArray(n);
    env->SetByteArrayRegion(arr, 0, n, (jbyte*)buf);
    return arr;
}