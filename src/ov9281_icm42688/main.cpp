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

int main(int argc, char **argv)
{
    signal(SIGINT, SigIntHandler);
    signal(SIGTERM, SigIntHandler);
    signal(SIGKILL, SigIntHandler);

    const AppConfig appConfig = ParseAppConfig(argc, argv);
    const CameraConfig& cameraConfig = appConfig.camera;
    const UdpConfig& udpConfig = appConfig.udp;
    const ImuRuntimeConfig& imuConfig = appConfig.imu;
    const RuntimeConfig& runtimeConfig = appConfig.runtime;

    const std::string& vocab = appConfig.vocab;
    const std::string& settings = appConfig.settings;

    // Temporary aliases kept to minimize changes in the main processing loop.
    const int w = cameraConfig.width;
    const int h = cameraConfig.height;
    const int fps = cameraConfig.fps;
    const bool aeDisable = cameraConfig.aeDisable;
    const int exposureUs = cameraConfig.exposureUs;
    const float gain = cameraConfig.gain;
    const bool requestY8 = cameraConfig.requestY8;
    const bool r16Norm = cameraConfig.r16Norm;
    const int pairMs = cameraConfig.pairMs;
    const int keepMs = cameraConfig.keepMs;
    const int cam1TsOffsetMs = cameraConfig.cam1TsOffsetMs;
    const bool autoCamOffset = cameraConfig.autoCamOffset;
    const int autoOffsetSamples = cameraConfig.autoOffsetSamples;
    const int autoOffsetTimeoutMs = cameraConfig.autoOffsetTimeoutMs;

    const bool udpEnable = udpConfig.enable;
    const std::string& udpIp = udpConfig.ip;
    const int udpPort = udpConfig.port;
    const int udpJpegQ = udpConfig.jpegQ;
    const int udpPayload = udpConfig.payload;
    const int udpQueue = udpConfig.queue;

    const std::string& spiDev = imuConfig.spiDev;
    const uint32_t spiSpeed = imuConfig.spiSpeed;
    const uint8_t spiMode = imuConfig.spiMode;
    const uint8_t spiBits = imuConfig.spiBits;
    const std::string& gpiochip = imuConfig.gpiochip;
    const unsigned drdyLine = imuConfig.drdyLine;
    const int imuHz = imuConfig.imuHz;
    const int accelFsG = imuConfig.accelFsG;
    const int gyroFsDps = imuConfig.gyroFsDps;
    const uint8_t imuStartReg = imuConfig.imuStartReg;
    const int64_t offRejectNs = runtimeConfig.offRejectNs;
    const bool allowEmptyImu = runtimeConfig.allowEmptyImu;
    const bool rtImu = imuConfig.rtImu;
    const int rtPrio = imuConfig.rtPrio;

    std::cerr << "ORB vocab=" << vocab << "\nsettings=" << settings << "\n";
    std::cerr << "cam " << w << "x" << h << " @" << fps
              << " aeDisable=" << (aeDisable ? "true" : "false") << " exp_us=" << exposureUs
              << " gain=" << gain << " requestY8=" << (requestY8 ? "Y" : "N")
              << " r16Norm=" << (r16Norm ? "Y" : "N") << "\n";
    std::cerr << "pair_thresh=" << pairMs << "ms keep_window=" << keepMs << "ms\n";
    std::cerr << "cam1TsOffsetMs=" << cam1TsOffsetMs
              << " autoCamOffset=" << (autoCamOffset ? "Y" : "N")
              << " auto_samples=" << autoOffsetSamples
              << " auto_timeout_ms=" << autoOffsetTimeoutMs << "\n";

    std::cerr << "imu spi=" << spiDev << " speed=" << spiSpeed << " mode=" << int(spiMode)
              << " bits=" << int(spiBits) << " drdy=" << gpiochip << ":" << drdyLine
              << " imuHz=" << imuHz << " imuStartReg=0x" << std::hex << int(imuStartReg)
              << std::dec << " accelFs=" << accelFsG << "g" << " gyroFs=" << gyroFsDps
              << "dps" << "\n";
    std::cerr << "udpEnable=" << (udpEnable ? "Y" : "N") << " udpIp=" << udpIp
              << " udpPort=" << udpPort << " jpeg_q=" << udpJpegQ << " payload=" << udpPayload
              << " queue=" << udpQueue << "\n";
    Logger::Init("./stereo_vslam.log", 32 * 1024 * 1024, Logger::INFO, true);
    // ORB-SLAM3 init
    ORB_SLAM3::System SLAM(vocab, settings, ORB_SLAM3::System::STEREO, false);
    MavlinkSerial mav("/dev/ttyAMA0", 921600);
    mav.StartRx();
    Px4Console console(mav);
    console.Start();
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
    ImuBuffer imuBuffer;
    std::atomic<bool> imuOk{false};
    std::atomic<uint64_t> imuCnt{0};
    std::atomic<uint64_t> imuDrop{0};

    ImuScale imu_scale{};
    std::atomic<float> accelLsbPerG{imu_scale.accelLsbPerG};
    std::atomic<float> gyroLsbPerDps{imu_scale.gyroLsbPerDps};

    std::thread imuThread([&]() {
        if (rtImu) {
            if (!SetThreadRealtime(rtPrio)) {
                std::cerr << "[imu] SetThreadRealtime failed (need CAP_SYS_NICE/root). continue.\n";
            } else {
                std::cerr << "[imu] realtime SCHED_FIFO prio=" << rtPrio << "\n";
            }
        }

        SpiDev spi(spiDev);
        if (!spi.Open(spiSpeed, spiMode, spiBits))
            return;

        ImuScale sc{};
        if (!IcmResetAndConfig(spi, imuHz, accelFsG, gyroFsDps, sc))
            return;

        accelLsbPerG.store(sc.accelLsbPerG);
        gyroLsbPerDps.store(sc.gyroLsbPerDps);
        std::cerr << "[imu] scale accel_lsb/g=" << sc.accelLsbPerG
                  << " gyro_lsb/dps=" << sc.gyroLsbPerDps << "\n";

        DrdyGpio drdy;
        if (!drdy.Open(gpiochip, drdyLine))
            return;

        imuOk.store(true);

        uint8_t raw12[12]{};
        uint8_t st = 0;
        spi.ReadReg(REG_INT_STATUS, st);

        while (g_runningFlag.load()) {
            int64_t tIrqNs = 0;
            if (!drdy.WaitTs(1000, tIrqNs))
                continue;

            ImuSample s{};
            s.tNs = tIrqNs;

            spi.ReadReg(REG_INT_STATUS, st);

            if (!spi.ReadRegs(imuStartReg, raw12, sizeof(raw12))) {
                imuDrop.fetch_add(1, std::memory_order_relaxed);
                continue;
            }

            ImuScale sc2{};
            sc2.accelLsbPerG = accelLsbPerG.load();
            sc2.gyroLsbPerDps = gyroLsbPerDps.load();
            ConvertRaw12AccelGyroToSi(raw12, sc2, s);

            // static uint64_t imuCnt = 0;
            // if (imuCnt % 200 == 0) {

            //   std::cerr.setf(std::ios::fixed);
            //   std::cerr << std::setprecision(6);

            //   std::cerr << "[IMU] a(m/s^2)=[" << s.ax << "," << s.ay << "," << s.az << "]"
            //             << " g(rad/s)=[" << s.gx << "," << s.gy << "," << s.gz << "]"
            //             << "\n";
            // }
            // imuCnt++;

            imuBuffer.Push(s);
            imuCnt.fetch_add(1, std::memory_order_relaxed);
        }
    });

    // Stereo camera
    LibcameraStereoOV9281_TsPair cam;
    if (!cam.Open(w, h, fps, aeDisable, exposureUs, gain, requestY8,
                  (int64_t)pairMs * 1'000'000LL, (int64_t)keepMs * 1'000'000LL, 8, r16Norm)) {
        g_runningFlag.store(false);
        if (imuThread.joinable())
            imuThread.join();
        return 1;
    }

    // Apply manual or auto cam1 offset
    if (cam1TsOffsetMs != 0) {
        cam.SetCam1OffsetNs((int64_t)cam1TsOffsetMs * 1'000'000LL);
        std::cerr << "[camoff] manual cam1TsOffsetMs=" << cam1TsOffsetMs << "\n";
    } else if (autoCamOffset) {
        std::cerr << "[camoff] auto estimating cam1 offset... samples=" << autoOffsetSamples
                  << " timeoutMs=" << autoOffsetTimeoutMs << "\n";
        cam.AutoEstimateAndSetOffset(autoOffsetSamples, autoOffsetTimeoutMs);
    }

    // Warmup: estimate tsOffsetNs (libcamera timestamp domain -> NowNs domain)
    // Now that we can pair, this should work.
    std::vector<int64_t> offs;
    offs.reserve(120);

    int warmPairs = 0;
    int64_t lastPrint = NowNs();
    while (g_runningFlag.load() && warmPairs < 60) {
        FrameItem L, R;
        if (!cam.GrabPair(L, R, 1000)) {
            const int64_t now = NowNs();
            if (now - lastPrint > 1'000'000'000LL) {
                lastPrint = now;
                std::cerr << "[warmup] waiting pairs..." << " last_seq=" << cam.LastSeq()
                          << " dtMs=" << cam.LastDtMs() << " pendL=" << cam.PendL()
                          << " pendR=" << cam.PendR() << " imuOk=" << (imuOk.load() ? "Y" : "N")
                          << " imuBuffer=" << imuBuffer.Size() << "\n";
            }
            continue;
        }
        int64_t camTs = (int64_t)((L.tsNs + R.tsNs) / 2);
        int64_t arrive = (L.arriveNs && R.arriveNs) ? (L.arriveNs + R.arriveNs) / 2 : NowNs();
        offs.push_back(arrive - camTs);
        warmPairs++;
    }

    int64_t tsOffsetNs = Median(offs);
    if (!offs.empty()) {
        int64_t mino = *std::min_element(offs.begin(), offs.end());
        int64_t maxo = *std::max_element(offs.begin(), offs.end());
        std::cerr << "Init tsOffsetNs(median)=" << tsOffsetNs << " range=[" << mino << ","
                  << maxo << "] ns\n";
    } else {
        std::cerr << "Init tsOffsetNs failed (no warmup pairs). Using 0.\n";
        tsOffsetNs = 0;
    }

    int64_t lastFrameNs = 0;
    uint64_t frameCnt1s = 0;
    uint64_t lastImuCnt = imuCnt.load();
    uint64_t lastImuDrop = imuDrop.load();
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
                const uint64_t ic = imuCnt.load();
                const uint64_t id = imuDrop.load();
                std::cerr << "[wd] no pair 1s" << " last_seq=" << cam.LastSeq()
                          << " dtMs=" << cam.LastDtMs() << " pendL=" << cam.PendL()
                          << " pendR=" << cam.PendR() << " imuHz~=" << (ic - lastImuCnt)
                          << " imuDrop~=" << (id - lastImuDrop) << " imuBuffer=" << imuBuffer.Size()
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
            vImu = imuBuffer.PopBetweenNs(lastFrameNs, frameNs, slackBeforeNs, slackAfterNs);
        }
        const int64_t prevFrameNs = lastFrameNs;
        lastFrameNs = frameNs;

        const double frameTime = (double)frameNs * 1e-9;

        const int64_t now = NowNs();
        lastStatNs = 0;
        if (now - lastStatNs > 1'000'000'000LL) {
            lastStatNs = now;
            const uint64_t ic = imuCnt.load();
            const uint64_t id = imuDrop.load();
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
            bool ok = imuBuffer.PeekFirstLast(tFirst, tLast);
            std::cerr << "[imu] EMPTY vImu" << " t0=" << (double)prevFrameNs * 1e-9
                      << " t1=" << (double)frameNs * 1e-9
                      << " dtMs=" << (frameNs - prevFrameNs) / 1e6 << " buf=" << imuBuffer.Size();
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

    cam.Close();
    g_runningFlag.store(false);
    if (imuThread.joinable())
        imuThread.join();
    SLAM.Shutdown();
    return 0;
}
