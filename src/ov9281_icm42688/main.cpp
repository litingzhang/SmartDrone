#include <opencv2/opencv.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <pthread.h>
#include <sched.h>

#include "ImuTypes.h"
#include "System.h"
#include "mavlink_pose_sender.hpp"
#include "px4_console.hpp"
#include <sophus/se3.hpp>
#include "app_args.hpp"
#include "icm42688_imu.hpp"
#include "drdy_gpio.hpp"
#include "imu_buffer.hpp"
#include "stereo_ov9281.hpp"
#include "udp_image_sender.hpp"
#include "logger.hpp"
#include "../udp_server/mavlink_hooks.hpp"
#include "../udp_server/tlv_cmd_router.hpp"
#include "../udp_server/tlv_pack.hpp"
#include "../udp_server/tlv_parser.hpp"
#include "../udp_server/udp_server.hpp"

// Build shortcut: embed the router implementation so this entry target can use udp_server
// without requiring a separate link step update.
#include "../udp_server/tlv_cmd_router.cpp"

namespace {

class Px4UdpHooks final : public MavlinkHooks {
public:
    explicit Px4UdpHooks(MavlinkSerial& mavlink) : m_mavlink(mavlink) {}

    VehicleGate GetGate() const override
    {
        VehicleGate gate{};
        gate.vioOk = true;
        gate.offboardReady = true;
        return gate;
    }

    bool Arm(std::string* err) override
    {
        if (!EnsureSetpointStream()) {
            if (err != nullptr) {
                *err = "setpoint stream start failed";
            }
            return false;
        }
        if (!m_mavlink.Arm(true)) {
            if (err != nullptr) {
                *err = "px4 arm rejected";
            }
            return false;
        }
        return true;
    }

    bool Disarm(std::string* err) override
    {
        if (!m_mavlink.Arm(false)) {
            if (err != nullptr) {
                *err = "px4 disarm rejected";
            }
            return false;
        }
        return true;
    }

    bool SetOffboard(std::string* err) override
    {
        if (!EnsureSetpointStream()) {
            if (err != nullptr) {
                *err = "setpoint stream start failed";
            }
            return false;
        }
        if (!m_mavlink.SetModeOffboard()) {
            if (err != nullptr) {
                *err = "px4 offboard rejected";
            }
            return false;
        }
        return true;
    }

    bool Hold(std::string* err) override
    {
        (void)err;
        if (!EnsureSetpointStream()) {
            return false;
        }
        MavlinkSerial::SetpointLocalNED setpoint{};
        setpoint.x = NAN;
        setpoint.y = NAN;
        setpoint.z = NAN;
        setpoint.vx = 0.0f;
        setpoint.vy = 0.0f;
        setpoint.vz = 0.0f;
        setpoint.yaw = NAN;
        setpoint.yawspeed = 0.0f;
        m_mavlink.UpdateStreamSetpoint(setpoint);
        return true;
    }

    bool Land(std::string* err) override
    {
        m_mavlink.StopSetpointStream();
        m_streamStarted.store(false, std::memory_order_relaxed);
        if (!m_mavlink.SendLand()) {
            if (err != nullptr) {
                *err = "px4 land rejected";
            }
            return false;
        }
        return true;
    }

    bool SetMoveGoal(const MoveGoal& goal, std::string* err) override
    {
        (void)err;
        if (!EnsureSetpointStream()) {
            return false;
        }

        MavlinkSerial::SetpointLocalNED setpoint{};
        if (goal.isVelocity) {
            setpoint.x = NAN;
            setpoint.y = NAN;
            setpoint.z = NAN;
            setpoint.vx = goal.vx;
            setpoint.vy = goal.vy;
            setpoint.vz = goal.vz;
            setpoint.yaw = NAN;
            setpoint.yawspeed = goal.yawRate;
        } else {
            setpoint.x = goal.x;
            setpoint.y = goal.y;
            setpoint.z = goal.z;
            setpoint.vx = NAN;
            setpoint.vy = NAN;
            setpoint.vz = NAN;
            setpoint.yaw = goal.yaw;
            setpoint.yawspeed = NAN;
        }
        m_mavlink.UpdateStreamSetpoint(setpoint);
        return true;
    }

private:
    bool EnsureSetpointStream()
    {
        bool expected = false;
        if (m_streamStarted.compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
            m_mavlink.StartSetpointStreamHz(20.0);
        }
        return true;
    }

    MavlinkSerial& m_mavlink;
    std::atomic<bool> m_streamStarted{false};
};

struct MainRuntimeAliases {
    int width{};
    int height{};
    int fps{};
    bool aeDisable{};
    int exposureUs{};
    float gain{};
    bool requestY8{};
    bool r16Norm{};
    int pairMs{};
    int keepMs{};
    int cam1TsOffsetMs{};
    bool autoCamOffset{};
    int autoOffsetSamples{};
    int autoOffsetTimeoutMs{};

    bool udpEnable{};
    std::string udpIp;
    int udpPort{};
    int udpJpegQ{};
    int udpPayload{};
    int udpQueue{};

    std::string spiDev;
    uint32_t spiSpeed{};
    uint8_t spiMode{};
    uint8_t spiBits{};
    std::string gpiochip;
    unsigned drdyLine{};
    int imuHz{};
    int accelFsG{};
    int gyroFsDps{};
    uint8_t imuStartReg{};
    int64_t offRejectNs{};
    bool allowEmptyImu{};
    bool rtImu{};
    int rtPrio{};
};

struct ImuThreadState {
    ImuBuffer imuBuffer;
    std::atomic<bool> imuOk{false};
    std::atomic<uint64_t> imuCnt{0};
    std::atomic<uint64_t> imuDrop{0};
    std::atomic<float> accelLsbPerG{0.0f};
    std::atomic<float> gyroLsbPerDps{0.0f};
};

MainRuntimeAliases BuildRuntimeAliases(const AppConfig& appConfig)
{
    MainRuntimeAliases aliases{};
    const CameraConfig& cameraConfig = appConfig.camera;
    const UdpConfig& udpConfig = appConfig.udp;
    const ImuRuntimeConfig& imuConfig = appConfig.imu;
    const RuntimeConfig& runtimeConfig = appConfig.runtime;

    aliases.width = cameraConfig.width;
    aliases.height = cameraConfig.height;
    aliases.fps = cameraConfig.fps;
    aliases.aeDisable = cameraConfig.aeDisable;
    aliases.exposureUs = cameraConfig.exposureUs;
    aliases.gain = cameraConfig.gain;
    aliases.requestY8 = cameraConfig.requestY8;
    aliases.r16Norm = cameraConfig.r16Norm;
    aliases.pairMs = cameraConfig.pairMs;
    aliases.keepMs = cameraConfig.keepMs;
    aliases.cam1TsOffsetMs = cameraConfig.cam1TsOffsetMs;
    aliases.autoCamOffset = cameraConfig.autoCamOffset;
    aliases.autoOffsetSamples = cameraConfig.autoOffsetSamples;
    aliases.autoOffsetTimeoutMs = cameraConfig.autoOffsetTimeoutMs;

    aliases.udpEnable = udpConfig.enable;
    aliases.udpIp = udpConfig.ip;
    aliases.udpPort = udpConfig.port;
    aliases.udpJpegQ = udpConfig.jpegQ;
    aliases.udpPayload = udpConfig.payload;
    aliases.udpQueue = udpConfig.queue;

    aliases.spiDev = imuConfig.spiDev;
    aliases.spiSpeed = imuConfig.spiSpeed;
    aliases.spiMode = imuConfig.spiMode;
    aliases.spiBits = imuConfig.spiBits;
    aliases.gpiochip = imuConfig.gpiochip;
    aliases.drdyLine = imuConfig.drdyLine;
    aliases.imuHz = imuConfig.imuHz;
    aliases.accelFsG = imuConfig.accelFsG;
    aliases.gyroFsDps = imuConfig.gyroFsDps;
    aliases.imuStartReg = imuConfig.imuStartReg;
    aliases.offRejectNs = runtimeConfig.offRejectNs;
    aliases.allowEmptyImu = runtimeConfig.allowEmptyImu;
    aliases.rtImu = imuConfig.rtImu;
    aliases.rtPrio = imuConfig.rtPrio;
    return aliases;
}

void InstallSignalHandlers()
{
    signal(SIGINT, SigIntHandler);
    signal(SIGTERM, SigIntHandler);
    signal(SIGKILL, SigIntHandler);
}

void PrintStartupConfig(
    const AppConfig& appConfig,
    const MainRuntimeAliases& aliases)
{
    std::cerr << "ORB vocab=" << appConfig.vocab << "\nsettings=" << appConfig.settings << "\n";
    std::cerr << "cam " << aliases.width << "x" << aliases.height << " @" << aliases.fps
              << " aeDisable=" << (aliases.aeDisable ? "true" : "false")
              << " exp_us=" << aliases.exposureUs << " gain=" << aliases.gain
              << " requestY8=" << (aliases.requestY8 ? "Y" : "N")
              << " r16Norm=" << (aliases.r16Norm ? "Y" : "N") << "\n";
    std::cerr << "pair_thresh=" << aliases.pairMs << "ms keep_window=" << aliases.keepMs
              << "ms\n";
    std::cerr << "cam1TsOffsetMs=" << aliases.cam1TsOffsetMs
              << " autoCamOffset=" << (aliases.autoCamOffset ? "Y" : "N")
              << " auto_samples=" << aliases.autoOffsetSamples
              << " auto_timeout_ms=" << aliases.autoOffsetTimeoutMs << "\n";

    std::cerr << "imu spi=" << aliases.spiDev << " speed=" << aliases.spiSpeed
              << " mode=" << int(aliases.spiMode) << " bits=" << int(aliases.spiBits)
              << " drdy=" << aliases.gpiochip << ":" << aliases.drdyLine
              << " imuHz=" << aliases.imuHz << " imuStartReg=0x" << std::hex
              << int(aliases.imuStartReg) << std::dec << " accelFs=" << aliases.accelFsG
              << "g"
              << " gyroFs=" << aliases.gyroFsDps << "dps" << "\n";
    std::cerr << "udpEnable=" << (aliases.udpEnable ? "Y" : "N")
              << " udpIp=" << aliases.udpIp << " udpPort=" << aliases.udpPort
              << " jpeg_q=" << aliases.udpJpegQ << " payload=" << aliases.udpPayload
              << " queue=" << aliases.udpQueue << "\n";
}

std::thread StartUdpCommandThread(int udpPort, Px4UdpHooks& udpHooks)
{
    return std::thread([udpPort, &udpHooks]() {
        UdpServer tlvServer;
        if (!tlvServer.Open(static_cast<uint16_t>(udpPort))) {
            std::cerr << "[udp_cmd] open failed on 0.0.0.0:" << udpPort << "\n";
            return;
        }
        std::cerr << "[udp_cmd] listening on 0.0.0.0:" << udpPort << "\n";

        TlvCmdRouter router(udpHooks);
        router.RegisterDefaults();
        TlvParser parser;
        UdpPeer lastPeer{};
        uint8_t rxBuffer[2048]{};

        while (g_runningFlag.load()) {
            UdpPeer peer{};
            const int recvLen = tlvServer.Recv(rxBuffer, sizeof(rxBuffer), &peer);
            if (recvLen <= 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            lastPeer = peer;
            parser.Push(rxBuffer, static_cast<size_t>(recvLen));
            while (auto frame = parser.TryPop()) {
                const RouteResult routeResult = router.Handle(*frame);
                std::vector<uint8_t> ackFrame = MakeAckFrame(
                    frame->seq, frame->tMs, frame->cmd, frame->seq, routeResult.status);
                tlvServer.SendTo(lastPeer, ackFrame.data(), ackFrame.size());
                if (!routeResult.msg.empty()) {
                    std::cerr << "[udp_cmd] " << routeResult.msg << "\n";
                }
            }
        }
    });
}

std::thread StartImuThread(const MainRuntimeAliases& aliases, ImuThreadState& imuState)
{
    ImuScale imuScale{};
    imuState.accelLsbPerG.store(imuScale.accelLsbPerG);
    imuState.gyroLsbPerDps.store(imuScale.gyroLsbPerDps);

    return std::thread([&aliases, &imuState]() {
        if (aliases.rtImu) {
            if (!SetThreadRealtime(aliases.rtPrio)) {
                std::cerr << "[imu] SetThreadRealtime failed (need CAP_SYS_NICE/root). continue.\n";
            } else {
                std::cerr << "[imu] realtime SCHED_FIFO prio=" << aliases.rtPrio << "\n";
            }
        }

        SpiDev spi(aliases.spiDev);
        if (!spi.Open(aliases.spiSpeed, aliases.spiMode, aliases.spiBits)) {
            return;
        }

        ImuScale scale{};
        if (!IcmResetAndConfig(
                spi, aliases.imuHz, aliases.accelFsG, aliases.gyroFsDps, scale)) {
            return;
        }

        imuState.accelLsbPerG.store(scale.accelLsbPerG);
        imuState.gyroLsbPerDps.store(scale.gyroLsbPerDps);
        std::cerr << "[imu] scale accel_lsb/g=" << scale.accelLsbPerG
                  << " gyro_lsb/dps=" << scale.gyroLsbPerDps << "\n";

        DrdyGpio drdy;
        if (!drdy.Open(aliases.gpiochip, aliases.drdyLine)) {
            return;
        }

        imuState.imuOk.store(true);

        uint8_t raw12[12]{};
        uint8_t intStatus = 0;
        spi.ReadReg(REG_INT_STATUS, intStatus);

        while (g_runningFlag.load()) {
            int64_t tIrqNs = 0;
            if (!drdy.WaitTs(1000, tIrqNs)) {
                continue;
            }

            ImuSample sample{};
            sample.tNs = tIrqNs;

            spi.ReadReg(REG_INT_STATUS, intStatus);
            if (!spi.ReadRegs(aliases.imuStartReg, raw12, sizeof(raw12))) {
                imuState.imuDrop.fetch_add(1, std::memory_order_relaxed);
                continue;
            }

            ImuScale currentScale{};
            currentScale.accelLsbPerG = imuState.accelLsbPerG.load();
            currentScale.gyroLsbPerDps = imuState.gyroLsbPerDps.load();
            ConvertRaw12AccelGyroToSi(raw12, currentScale, sample);

            imuState.imuBuffer.Push(sample);
            imuState.imuCnt.fetch_add(1, std::memory_order_relaxed);
        }
    });
}

bool OpenAndConfigureCamera(
    LibcameraStereoOV9281_TsPair& cam,
    const MainRuntimeAliases& aliases,
    std::thread& imuThread,
    std::thread& udpCmdThread)
{
    if (!cam.Open(
            aliases.width,
            aliases.height,
            aliases.fps,
            aliases.aeDisable,
            aliases.exposureUs,
            aliases.gain,
            aliases.requestY8,
            static_cast<int64_t>(aliases.pairMs) * 1'000'000LL,
            static_cast<int64_t>(aliases.keepMs) * 1'000'000LL,
            8,
            aliases.r16Norm)) {
        g_runningFlag.store(false);
        if (udpCmdThread.joinable()) {
            udpCmdThread.join();
        }
        if (imuThread.joinable()) {
            imuThread.join();
        }
        return false;
    }

    if (aliases.cam1TsOffsetMs != 0) {
        cam.SetCam1OffsetNs(static_cast<int64_t>(aliases.cam1TsOffsetMs) * 1'000'000LL);
        std::cerr << "[camoff] manual cam1TsOffsetMs=" << aliases.cam1TsOffsetMs << "\n";
    } else if (aliases.autoCamOffset) {
        std::cerr << "[camoff] auto estimating cam1 offset... samples=" << aliases.autoOffsetSamples
                  << " timeoutMs=" << aliases.autoOffsetTimeoutMs << "\n";
        cam.AutoEstimateAndSetOffset(aliases.autoOffsetSamples, aliases.autoOffsetTimeoutMs);
    }
    return true;
}

int64_t EstimateTimestampOffsetNs(
    LibcameraStereoOV9281_TsPair& cam,
    const ImuThreadState& imuState)
{
    std::vector<int64_t> offsets;
    offsets.reserve(120);

    int warmPairs = 0;
    int64_t lastPrintNs = NowNs();
    while (g_runningFlag.load() && warmPairs < 60) {
        FrameItem leftFrame, rightFrame;
        if (!cam.GrabPair(leftFrame, rightFrame, 1000)) {
            const int64_t nowNs = NowNs();
            if (nowNs - lastPrintNs > 1'000'000'000LL) {
                lastPrintNs = nowNs;
                std::cerr << "[warmup] waiting pairs..."
                          << " last_seq=" << cam.LastSeq()
                          << " dtMs=" << cam.LastDtMs()
                          << " pendL=" << cam.PendL()
                          << " pendR=" << cam.PendR()
                          << " imuOk=" << (imuState.imuOk.load() ? "Y" : "N")
                          << " imuBuffer=" << imuState.imuBuffer.Size() << "\n";
            }
            continue;
        }

        const int64_t camTsNs = static_cast<int64_t>((leftFrame.tsNs + rightFrame.tsNs) / 2);
        const int64_t arriveNs =
            (leftFrame.arriveNs && rightFrame.arriveNs)
                ? (leftFrame.arriveNs + rightFrame.arriveNs) / 2
                : NowNs();
        offsets.push_back(arriveNs - camTsNs);
        warmPairs++;
    }

    int64_t tsOffsetNs = Median(offsets);
    if (!offsets.empty()) {
        const int64_t minOffset = *std::min_element(offsets.begin(), offsets.end());
        const int64_t maxOffset = *std::max_element(offsets.begin(), offsets.end());
        std::cerr << "Init tsOffsetNs(median)=" << tsOffsetNs << " range=[" << minOffset << ","
                  << maxOffset << "] ns\n";
    } else {
        std::cerr << "Init tsOffsetNs failed (no warmup pairs). Using 0.\n";
        tsOffsetNs = 0;
    }
    return tsOffsetNs;
}

void ShutdownRuntime(
    LibcameraStereoOV9281_TsPair& cam,
    Px4Console& console,
    MavlinkSerial& mav,
    std::thread& udpCmdThread,
    std::thread& imuThread)
{
    cam.Close();
    g_runningFlag.store(false);
    console.Stop();
    mav.StopSetpointStream();
    mav.StopRx();
    if (udpCmdThread.joinable()) {
        udpCmdThread.join();
    }
    if (imuThread.joinable()) {
        imuThread.join();
    }
}

}  // namespace

int main(int argc, char **argv)
{
    InstallSignalHandlers();

    const AppConfig appConfig = ParseAppConfig(argc, argv);
    const MainRuntimeAliases runtimeAliases = BuildRuntimeAliases(appConfig);

    const std::string& vocab = appConfig.vocab;
    const std::string& settings = appConfig.settings;

    // Temporary aliases kept to minimize changes in the main processing loop.
    const int w = runtimeAliases.width;
    const int h = runtimeAliases.height;
    const int fps = runtimeAliases.fps;
    const bool aeDisable = runtimeAliases.aeDisable;
    const int exposureUs = runtimeAliases.exposureUs;
    const float gain = runtimeAliases.gain;
    const bool requestY8 = runtimeAliases.requestY8;
    const bool r16Norm = runtimeAliases.r16Norm;
    const int pairMs = runtimeAliases.pairMs;
    const int keepMs = runtimeAliases.keepMs;
    const bool udpEnable = runtimeAliases.udpEnable;
    const std::string& udpIp = runtimeAliases.udpIp;
    const int udpPort = runtimeAliases.udpPort;
    const int udpJpegQ = runtimeAliases.udpJpegQ;
    const int udpPayload = runtimeAliases.udpPayload;
    const int udpQueue = runtimeAliases.udpQueue;
    const int imuHz = runtimeAliases.imuHz;
    const int64_t offRejectNs = runtimeAliases.offRejectNs;
    const bool allowEmptyImu = runtimeAliases.allowEmptyImu;

    PrintStartupConfig(appConfig, runtimeAliases);
    Logger::Init("./stereo_vslam.log", 32 * 1024 * 1024, Logger::INFO, true);
    // ORB-SLAM3 init
    ORB_SLAM3::System SLAM(vocab, settings, ORB_SLAM3::System::STEREO, false);
    MavlinkSerial mav("/dev/ttyAMA0", 921600);
    mav.StartRx();
    Px4Console console(mav);
    console.Start();
    Px4UdpHooks udpHooks(mav);
    std::thread udpCmdThread = StartUdpCommandThread(udpPort, udpHooks);
    // UDP sender (optional)
    UdpImageSender udp;
    if (udpEnable) {
        if (!udp.Open(udpIp, udpPort, udpJpegQ, udpPayload, udpQueue)) {
            std::cerr << "[udp] open failed, continue without udp.\n";
        } else {
            std::cerr << "[udp] sending to " << udpIp << ":" << udpPort << "\n";
        }
    }

    // IMU thread
    ImuThreadState imuState;
    std::thread imuThread = StartImuThread(runtimeAliases, imuState);

    // Stereo camera
    LibcameraStereoOV9281_TsPair cam;
    if (!OpenAndConfigureCamera(cam, runtimeAliases, imuThread, udpCmdThread)) {
        return 1;
    }
    int64_t tsOffsetNs = EstimateTimestampOffsetNs(cam, imuState);

    int64_t lastFrameNs = 0;
    uint64_t frameCnt1s = 0;
    uint64_t lastImuCnt = imuState.imuCnt.load();
    uint64_t lastImuDrop = imuState.imuDrop.load();
    int64_t lastStatNs = NowNs();

    const int64_t frame_step_ns = (int64_t)(1000000000LL / std::max(1, fps));
    const int64_t imuDtNs = (int64_t)(1000000000LL / std::max(1, imuHz));
    const int64_t slackBeforeNs = std::max<int64_t>(2 * imuDtNs, 5'000'000);
    const int64_t slackAfterNs = std::max<int64_t>(2 * imuDtNs, 5'000'000);

    mav.UpdateStreamPosition(0.0f, 0.0f, -0.3f, NAN); // NED: z=-0.3 表示向上 0.3m

    while (g_runningFlag.load()) {
        FrameItem L, R;
        if (!cam.GrabPair(L, R, 1000)) {
            const int64_t now = NowNs();
            if (now - lastStatNs > 1'000'000'000LL) {
                lastStatNs = now;
                const uint64_t ic = imuState.imuCnt.load();
                const uint64_t id = imuState.imuDrop.load();
                std::cerr << "[wd] no pair 1s" << " last_seq=" << cam.LastSeq()
                          << " dtMs=" << cam.LastDtMs() << " pendL=" << cam.PendL()
                          << " pendR=" << cam.PendR() << " imuHz~=" << (ic - lastImuCnt)
                          << " imuDrop~=" << (id - lastImuDrop)
                          << " imuBuffer=" << imuState.imuBuffer.Size()
                          << "\n";
                lastImuCnt = ic;
                lastImuDrop = id;
            }
            continue;
        }

        frameCnt1s++;

        const int64_t camTs = (int64_t)((L.tsNs + R.tsNs) / 2);
        const int64_t arrive =
            (L.arriveNs && R.arriveNs) ? (L.arriveNs + R.arriveNs) / 2 : NowNs();
        const int64_t off_meas = arrive - camTs;

        const int64_t err = off_meas - tsOffsetNs;
        if (Abs64(err) < offRejectNs) {
            tsOffsetNs = tsOffsetNs + (err >> 6); // alpha=1/64
        }

        int64_t frameNs = camTs + tsOffsetNs;

        if (lastFrameNs != 0 && frameNs <= lastFrameNs) {
            frameNs = lastFrameNs + frame_step_ns;
        }

        std::vector<ORB_SLAM3::IMU::Point> vImu;
        if (lastFrameNs != 0) {
            vImu = imuState.imuBuffer.PopBetweenNs(
                lastFrameNs, frameNs, slackBeforeNs, slackAfterNs);
        }
        const int64_t prevFrameNs = lastFrameNs;
        lastFrameNs = frameNs;

        const double frameTime = (double)frameNs * 1e-9;

        const int64_t now = NowNs();
        if (now - lastStatNs > 1'000'000'000LL) {
            lastStatNs = now;
            const uint64_t ic = imuState.imuCnt.load();
            const uint64_t id = imuState.imuDrop.load();
            // std::cerr << "[STAT] fps~=" << frameCnt1s
            //           << " dtMs=" << cam.LastDtMs()
            //           << " imuHz~=" << (ic - lastImuCnt)
            //           << " imuDrop~=" << (id - lastImuDrop)
            //           << " imuBuffer=" << imuBuffer.Size()
            //           << " last_vImu=" << vImu.size()
            //           << " off_ns=" << tsOffsetNs
            //           << "\n";
            frameCnt1s = 0;
            lastImuCnt = ic;
            lastImuDrop = id;
        }

        if (prevFrameNs != 0 && vImu.empty()) {
            int64_t tFirst = 0, tLast = 0;
            bool ok = imuState.imuBuffer.PeekFirstLast(tFirst, tLast);
            std::cerr << "[imu] EMPTY vImu" << " t0=" << (double)prevFrameNs * 1e-9
                      << " t1=" << (double)frameNs * 1e-9
                      << " dtMs=" << (frameNs - prevFrameNs) / 1e6
                      << " buf=" << imuState.imuBuffer.Size();
            if (ok) {
                std::cerr << " imu_first=" << (double)tFirst * 1e-9
                          << " imu_last=" << (double)tLast * 1e-9;
            }
            std::cerr << "\n";

            if (!allowEmptyImu)
                continue;
        }

        if (udpEnable) {
            udp.Enqueue(0, L.seq, frameTime, L.gray);
            udp.Enqueue(1, R.seq, frameTime, R.gray);
        }

        Sophus::SE3f Tcw = SLAM.TrackStereo(L.gray, R.gray, frameTime, vImu);
        int state = SLAM.GetTrackingState();
        Sophus::SE3f Twc = Tcw.inverse();

        const Eigen::Vector3f t = Twc.translation();
        const Eigen::Quaternionf q(Twc.so3().unit_quaternion());
        MavlinkSerial::Pose pNed;
        pNed.x = t.x();
        pNed.y = t.y();
        pNed.z = t.z();
        pNed.qw = q.w();
        pNed.qx = q.x();
        pNed.qy = q.y();
        pNed.qz = q.z();
        MavlinkSerial::NormalizeQuat(pNed.qw, pNed.qx, pNed.qy, pNed.qz);
        uint64_t tUs = MonoTimeUs();
        if (state == ORB_SLAM3::Tracking::OK) {
            mav.SendOdometry(tUs, pNed, MAV_FRAME_LOCAL_NED, MAV_FRAME_BODY_FRD, OdomQualityMode::GOOD);
            LOGI("[POSE]%f,(x:%f,y:%f,z:%f),(qw:%f,qx:%f,qy:%f,qz:%f)",
                frameTime, pNed.x, pNed.y, pNed.z, pNed.qw, pNed.qx, pNed.qy, pNed.qz);
        } else {
            mav.SendOdometry(tUs, pNed, MAV_FRAME_LOCAL_NED, MAV_FRAME_BODY_FRD, OdomQualityMode::LOST);
        }
    }

    ShutdownRuntime(cam, console, mav, udpCmdThread, imuThread);
    SLAM.Shutdown();
    return 0;
}
