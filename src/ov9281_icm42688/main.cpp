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
              << std::dec << " accel_fs=" << accelFsG << "g" << " gyro_fs=" << gyroFsDps
              << "dps" << "\n";
    std::cerr << "udpEnable=" << (udpEnable ? "Y" : "N") << " udpIp=" << udpIp
              << " udpPort=" << udpPort << " jpeg_q=" << udpJpegQ << " payload=" << udpPayload
              << " queue=" << udpQueue << "\n";
    Logger::Init("./stereo_vslam.log", 32 * 1024 * 1024, Logger::INFO, true);
    // ORB-SLAM3 init
    ORB_SLAM3::System SLAM(vocab, settings, ORB_SLAM3::System::STEREO, false);
    MavlinkSerial mav("/dev/ttyAMA0", 921600);
    mav.startRx();
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
    ImuBuffer imu_buf;
    std::atomic<bool> imu_ok{false};
    std::atomic<uint64_t> imu_cnt{0};
    std::atomic<uint64_t> imu_drop{0};

    ImuScale imu_scale{};
    std::atomic<float> accelLsbPerG{imu_scale.accelLsbPerG};
    std::atomic<float> gyroLsbPerDps{imu_scale.gyroLsbPerDps};

    std::thread imu_th([&]() {
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

        imu_ok.store(true);

        uint8_t raw12[12]{};
        uint8_t st = 0;
        spi.ReadReg(REG_INT_STATUS, st);

        while (g_runningFlag.load()) {
            int64_t t_irq_ns = 0;
            if (!drdy.WaitTs(1000, t_irq_ns))
                continue;

            ImuSample s{};
            s.tNs = t_irq_ns;

            spi.ReadReg(REG_INT_STATUS, st);

            if (!spi.ReadRegs(imuStartReg, raw12, sizeof(raw12))) {
                imu_drop.fetch_add(1, std::memory_order_relaxed);
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

            imu_buf.Push(s);
            imu_cnt.fetch_add(1, std::memory_order_relaxed);
        }
    });

    // Stereo camera
    LibcameraStereoOV9281_TsPair cam;
    if (!cam.Open(w, h, fps, aeDisable, exposureUs, gain, requestY8,
                  (int64_t)pairMs * 1'000'000LL, (int64_t)keepMs * 1'000'000LL, 8, r16Norm)) {
        g_runningFlag.store(false);
        if (imu_th.joinable())
            imu_th.join();
        return 1;
    }

    // Apply manual or auto cam1 offset
    if (cam1TsOffsetMs != 0) {
        cam.SetCam1OffsetNs((int64_t)cam1TsOffsetMs * 1'000'000LL);
        std::cerr << "[camoff] manual cam1TsOffsetMs=" << cam1TsOffsetMs << "\n";
    } else if (autoCamOffset) {
        std::cerr << "[camoff] auto estimating cam1 offset... samples=" << autoOffsetSamples
                  << " timeout_ms=" << autoOffsetTimeoutMs << "\n";
        cam.AutoEstimateAndSetOffset(autoOffsetSamples, autoOffsetTimeoutMs);
    }

    // Warmup: estimate ts_offset_ns (libcamera timestamp domain -> NowNs domain)
    // Now that we can pair, this should work.
    std::vector<int64_t> offs;
    offs.reserve(120);

    int warm_pairs = 0;
    int64_t last_print = NowNs();
    while (g_runningFlag.load() && warm_pairs < 60) {
        FrameItem L, R;
        if (!cam.GrabPair(L, R, 1000)) {
            const int64_t now = NowNs();
            if (now - last_print > 1'000'000'000LL) {
                last_print = now;
                std::cerr << "[warmup] waiting pairs..." << " last_seq=" << cam.LastSeq()
                          << " dt_ms=" << cam.LastDtMs() << " pendL=" << cam.PendL()
                          << " pendR=" << cam.PendR() << " imu_ok=" << (imu_ok.load() ? "Y" : "N")
                          << " imu_buf=" << imu_buf.Size() << "\n";
            }
            continue;
        }
        int64_t cam_ts = (int64_t)((L.tsNs + R.tsNs) / 2);
        int64_t arrive = (L.arriveNs && R.arriveNs) ? (L.arriveNs + R.arriveNs) / 2 : NowNs();
        offs.push_back(arrive - cam_ts);
        warm_pairs++;
    }

    int64_t ts_offset_ns = Median(offs);
    if (!offs.empty()) {
        int64_t mino = *std::min_element(offs.begin(), offs.end());
        int64_t maxo = *std::max_element(offs.begin(), offs.end());
        std::cerr << "Init ts_offset_ns(median)=" << ts_offset_ns << " range=[" << mino << ","
                  << maxo << "] ns\n";
    } else {
        std::cerr << "Init ts_offset_ns failed (no warmup pairs). Using 0.\n";
        ts_offset_ns = 0;
    }

    int64_t last_frame_ns = 0;
    uint64_t frame_cnt_1s = 0;
    uint64_t last_imu_cnt = imu_cnt.load();
    uint64_t last_imu_drop = imu_drop.load();
    int64_t last_stat_ns = NowNs();

    const int64_t frame_step_ns = (int64_t)(1000000000LL / std::max(1, fps));
    const int64_t imu_dt_ns = (int64_t)(1000000000LL / std::max(1, imuHz));
    const int64_t slack_before_ns = std::max<int64_t>(2 * imu_dt_ns, 5'000'000);
    const int64_t slack_after_ns = std::max<int64_t>(2 * imu_dt_ns, 5'000'000);

    mav.updateStreamPosition(0.0f, 0.0f, -0.3f, NAN); // NED: z=-0.3 表示向上 0.3m

    while (g_runningFlag.load()) {
        FrameItem L, R;
        if (!cam.GrabPair(L, R, 1000)) {
            const int64_t now = NowNs();
            if (now - last_stat_ns > 1'000'000'000LL) {
                last_stat_ns = now;
                const uint64_t ic = imu_cnt.load();
                const uint64_t id = imu_drop.load();
                std::cerr << "[wd] no pair 1s" << " last_seq=" << cam.LastSeq()
                          << " dt_ms=" << cam.LastDtMs() << " pendL=" << cam.PendL()
                          << " pendR=" << cam.PendR() << " imuHz~=" << (ic - last_imu_cnt)
                          << " imu_drop~=" << (id - last_imu_drop) << " imu_buf=" << imu_buf.Size()
                          << "\n";
                last_imu_cnt = ic;
                last_imu_drop = id;
            }
            continue;
        }

        frame_cnt_1s++;

        const int64_t cam_ts = (int64_t)((L.tsNs + R.tsNs) / 2);
        const int64_t arrive =
            (L.arriveNs && R.arriveNs) ? (L.arriveNs + R.arriveNs) / 2 : NowNs();
        const int64_t off_meas = arrive - cam_ts;

        const int64_t err = off_meas - ts_offset_ns;
        if (Abs64(err) < offRejectNs) {
            ts_offset_ns = ts_offset_ns + (err >> 6); // alpha=1/64
        }

        int64_t frame_ns = cam_ts + ts_offset_ns;

        if (last_frame_ns != 0 && frame_ns <= last_frame_ns) {
            frame_ns = last_frame_ns + frame_step_ns;
        }

        std::vector<ORB_SLAM3::IMU::Point> vImu;
        if (last_frame_ns != 0) {
            vImu = imu_buf.PopBetweenNs(last_frame_ns, frame_ns, slack_before_ns, slack_after_ns);
        }
        const int64_t prev_frame_ns = last_frame_ns;
        last_frame_ns = frame_ns;

        const double frameTime = (double)frame_ns * 1e-9;

        const int64_t now = NowNs();
        last_stat_ns = 0;
        if (now - last_stat_ns > 1'000'000'000LL) {
            last_stat_ns = now;
            const uint64_t ic = imu_cnt.load();
            const uint64_t id = imu_drop.load();
            // std::cerr << "[STAT] fps~=" << frame_cnt_1s
            //           << " dt_ms=" << cam.LastDtMs()
            //           << " imuHz~=" << (ic - last_imu_cnt)
            //           << " imu_drop~=" << (id - last_imu_drop)
            //           << " imu_buf=" << imu_buf.Size()
            //           << " last_vImu=" << vImu.size()
            //           << " off_ns=" << ts_offset_ns
            //           << "\n";
            frame_cnt_1s = 0;
            last_imu_cnt = ic;
            last_imu_drop = id;
        }

        if (prev_frame_ns != 0 && vImu.empty()) {
            int64_t t_first = 0, t_last = 0;
            bool ok = imu_buf.PeekFirstLast(t_first, t_last);
            std::cerr << "[imu] EMPTY vImu" << " t0=" << (double)prev_frame_ns * 1e-9
                      << " t1=" << (double)frame_ns * 1e-9
                      << " dt_ms=" << (frame_ns - prev_frame_ns) / 1e6 << " buf=" << imu_buf.Size();
            if (ok) {
                std::cerr << " imu_first=" << (double)t_first * 1e-9
                          << " imu_last=" << (double)t_last * 1e-9;
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
        MavlinkSerial::Pose p_ned;
        p_ned.x = t.x();
        p_ned.y = t.y();
        p_ned.z = t.z();
        p_ned.qw = q.w();
        p_ned.qx = q.x();
        p_ned.qy = q.y();
        p_ned.qz = q.z();
        MavlinkSerial::normalizeQuat(p_ned.qw, p_ned.qx, p_ned.qy, p_ned.qz);
        uint64_t t_us = mono_time_us();
        if (state == ORB_SLAM3::Tracking::OK) {
            mav.sendOdometry(t_us, p_ned, MAV_FRAME_LOCAL_NED, MAV_FRAME_BODY_FRD, OdomQualityMode::GOOD);
            LOGI("[POSE]%f,(x:%f,y:%f,z:%f),(qw:%f,qx:%f,qy:%f,qz:%f)",
                frameTime, p_ned.x, p_ned.y, p_ned.z, p_ned.qw, p_ned.qx, p_ned.qy, p_ned.qz);
        } else {
            mav.sendOdometry(t_us, p_ned, MAV_FRAME_LOCAL_NED, MAV_FRAME_BODY_FRD, OdomQualityMode::LOST);
        }
    }

    cam.Close();
    g_runningFlag.store(false);
    if (imu_th.joinable())
        imu_th.join();
    SLAM.Shutdown();
    return 0;
}
