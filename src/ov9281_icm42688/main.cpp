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
#include "spi_dev.hpp"
#include "drdy_gpio.hpp"
#include "imu_buffer.hpp"
#include "stereo_ov9281.hpp"
#include "udp_image_sender.hpp"

// ---------------- IMU (ICM42688) ----------------
static inline int16_t Be16ToI16(uint8_t hi, uint8_t lo)
{
    return (int16_t)((uint16_t(hi) << 8) | uint16_t(lo));
}

// Bank0 registers (subset)
static constexpr uint8_t REG_DEVICE_CONFIG = 0x11;
static constexpr uint8_t REG_INT_CONFIG = 0x14;
static constexpr uint8_t REG_INT_STATUS = 0x2D;
static constexpr uint8_t REG_PWR_MGMT0 = 0x4E;
static constexpr uint8_t REG_GYRO_CONFIG0 = 0x4F;
static constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50;
static constexpr uint8_t REG_INT_CONFIG1 = 0x64;
static constexpr uint8_t REG_INT_SOURCE0 = 0x65;

static bool SetThreadRealtime(int prio)
{
    sched_param sp{};
    sp.sched_priority = prio;
    return pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0;
}

static uint8_t OdrCodeFromHz(int hz)
{
    switch (hz) {
    case 8000:
        return 0x03;
    case 4000:
        return 0x04;
    case 2000:
        return 0x05;
    case 1000:
        return 0x06;
    case 200:
        return 0x07;
    case 100:
        return 0x08;
    case 50:
        return 0x09;
    case 25:
        return 0x0A;
    default:
        return 0x0F;
    }
}

static bool BuildFsBitsAndScale(int accel_fs_g, int gyro_fs_dps, uint8_t &accel_fs_bits,
                                uint8_t &gyro_fs_bits, ImuScale &scale)
{
    switch (accel_fs_g) {
    case 2:
        scale.accel_lsb_per_g = 16384.0f;
        accel_fs_bits = 0x03;
        break;
    case 4:
        scale.accel_lsb_per_g = 8192.0f;
        accel_fs_bits = 0x02;
        break;
    case 8:
        scale.accel_lsb_per_g = 4096.0f;
        accel_fs_bits = 0x01;
        break;
    case 16:
        scale.accel_lsb_per_g = 2048.0f;
        accel_fs_bits = 0x00;
        break;
    default:
        std::cerr << "Unsupported accel-fs " << accel_fs_g << " (use 2/4/8/16)\n";
        return false;
    }

    switch (gyro_fs_dps) {
    case 125:
        scale.gyro_lsb_per_dps = 262.4f;
        gyro_fs_bits = 0x04;
        break;
    case 250:
        scale.gyro_lsb_per_dps = 131.0f;
        gyro_fs_bits = 0x03;
        break;
    case 500:
        scale.gyro_lsb_per_dps = 65.5f;
        gyro_fs_bits = 0x02;
        break;
    case 1000:
        scale.gyro_lsb_per_dps = 32.8f;
        gyro_fs_bits = 0x01;
        break;
    case 2000:
        scale.gyro_lsb_per_dps = 16.4f;
        gyro_fs_bits = 0x00;
        break;
    default:
        std::cerr << "Unsupported gyro-fs " << gyro_fs_dps << " (use 125/250/500/1000/2000)\n";
        return false;
    }

    return true;
}

static bool IcmResetAndConfig(SpiDev &spi, int imu_hz, int accel_fs_g, int gyro_fs_dps,
                              ImuScale &scale_out)
{
    uint8_t accel_fs_bits = 0, gyro_fs_bits = 0;
    ImuScale sc{};
    if (!BuildFsBitsAndScale(accel_fs_g, gyro_fs_dps, accel_fs_bits, gyro_fs_bits, sc))
        return false;

    if (!spi.WriteReg(REG_DEVICE_CONFIG, 0x01))
        return false;
    usleep(100000);

    spi.WriteReg(REG_INT_CONFIG, 0x30);
    spi.WriteReg(REG_INT_SOURCE0, 0x08); // DRDY
    spi.WriteReg(REG_INT_CONFIG1, 0x00);

    spi.WriteReg(REG_PWR_MGMT0, 0x0F);
    usleep(20000);

    uint8_t odr = OdrCodeFromHz(imu_hz);

    uint8_t gyro_cfg0 = uint8_t((gyro_fs_bits << 5) | (odr & 0x0F));
    uint8_t accel_cfg0 = uint8_t((accel_fs_bits << 5) | (odr & 0x0F));

    if (!spi.WriteReg(REG_GYRO_CONFIG0, gyro_cfg0))
        return false;
    if (!spi.WriteReg(REG_ACCEL_CONFIG0, accel_cfg0))
        return false;
    usleep(20000);

    scale_out = sc;
    return true;
}

static void ConvertRaw12_AccelGyro_ToSI(const uint8_t raw12[12], const ImuScale &sc, ImuSample &s)
{
    int16_t ax = Be16ToI16(raw12[0], raw12[1]);
    int16_t ay = Be16ToI16(raw12[2], raw12[3]);
    int16_t az = Be16ToI16(raw12[4], raw12[5]);
    int16_t gx = Be16ToI16(raw12[6], raw12[7]);
    int16_t gy = Be16ToI16(raw12[8], raw12[9]);
    int16_t gz = Be16ToI16(raw12[10], raw12[11]);

    constexpr float kG = 9.80665f;
    s.ax = (float(ax) / sc.accel_lsb_per_g) * kG;
    s.ay = (float(ay) / sc.accel_lsb_per_g) * kG;
    s.az = (float(az) / sc.accel_lsb_per_g) * kG;

    constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;
    s.gx = (float(gx) / sc.gyro_lsb_per_dps) * kDeg2Rad;
    s.gy = (float(gy) / sc.gyro_lsb_per_dps) * kDeg2Rad;
    s.gz = (float(gz) / sc.gyro_lsb_per_dps) * kDeg2Rad;
}

// ---------------- args ----------------
static std::string GetArgS(int argc, char **argv, const char *name, const char *def)
{
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == name)
            return argv[i + 1];
    return def;
}
static int GetArgI(int argc, char **argv, const char *name, int def)
{
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == name)
            return std::stoi(argv[i + 1]);
    return def;
}
static int64_t GetArgI64(int argc, char **argv, const char *name, int64_t def)
{
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == name)
            return std::stoll(argv[i + 1]);
    return def;
}
static float GetArgF(int argc, char **argv, const char *name, float def)
{
    for (int i = 1; i + 1 < argc; i++)
        if (std::string(argv[i]) == name)
            return std::stof(argv[i + 1]);
    return def;
}
static bool HasArg(int argc, char **argv, const char *name)
{
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == name)
            return true;
    return false;
}
static uint8_t ParseU8HexOrDec(const std::string &s, uint8_t def)
{
    try {
        int base = 10;
        std::string t = s;
        if (t.size() > 2 && t[0] == '0' && (t[1] == 'x' || t[1] == 'X'))
            base = 16;
        int v = std::stoi(t, nullptr, base);
        if (v < 0 || v > 255)
            return def;
        return (uint8_t)v;
    } catch (...) {
        return def;
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, SigIntHandler);
    signal(SIGTERM, SigIntHandler);
    signal(SIGKILL, SigIntHandler);

    const std::string vocab = GetArgS(argc, argv, "--vocab", "ORBvoc.txt");
    const std::string settings = GetArgS(argc, argv, "--settings", "stereo_inertial.yaml");

    // camera
    const int w = GetArgI(argc, argv, "--w", 1280);
    const int h = GetArgI(argc, argv, "--h", 800);
    const int fps = GetArgI(argc, argv, "--fps", 30);
    const bool ae_disable = !HasArg(argc, argv, "--ae");
    const int exposure_us = GetArgI(argc, argv, "--exp-us", 5000);
    const float gain = GetArgF(argc, argv, "--gain", 8.0f);

    const bool request_y8 = !HasArg(argc, argv, "--no-y8");
    const bool r16_norm = HasArg(argc, argv, "--r16-norm");

    // stereo pairing
    const int pair_ms = GetArgI(argc, argv, "--pair-ms", 2);
    const int keep_ms = GetArgI(argc, argv, "--keep-ms", 120);
    // udp
    const bool udp_enable = HasArg(argc, argv, "--udp");
    const std::string udp_ip = GetArgS(argc, argv, "--udp-ip", "192.168.1.10");
    const int udp_port = GetArgI(argc, argv, "--udp-port", 5000);
    const int udp_jpeg_q = GetArgI(argc, argv, "--udp-jpeg-q", 80);
    const int udp_payload = GetArgI(argc, argv, "--udp-payload", 1200);
    const int udp_q = GetArgI(argc, argv, "--udp-queue", 4);

    // --- NEW: camera timestamp compensation ---
    const int cam1_ts_offset_ms = GetArgI(argc, argv, "--cam1-ts-offset-ms", 0);
    const bool auto_cam_offset =
        HasArg(argc, argv, "--auto-cam-offset") || (cam1_ts_offset_ms == 0);
    const int auto_offset_samples = GetArgI(argc, argv, "--auto-offset-samples", 120);
    const int auto_offset_timeout_ms = GetArgI(argc, argv, "--auto-offset-timeout-ms", 3000);

    // imu
    const std::string spi_dev = GetArgS(argc, argv, "--spi", "/dev/spidev0.0");
    const uint32_t spi_speed = (uint32_t)GetArgI(argc, argv, "--speed", 8000000);
    const uint8_t spi_mode = (uint8_t)GetArgI(argc, argv, "--mode", 0);
    const uint8_t spi_bits = (uint8_t)GetArgI(argc, argv, "--bits", 8);

    const std::string gpiochip = GetArgS(argc, argv, "--gpiochip", "/dev/gpiochip0");
    const unsigned drdy_line = (unsigned)GetArgI(argc, argv, "--drdy", 24);
    const int imu_hz = GetArgI(argc, argv, "--imu-hz", 200);

    const int accel_fs_g = GetArgI(argc, argv, "--accel-fs", 16);
    const int gyro_fs_dps = GetArgI(argc, argv, "--gyro-fs", 2000);

    uint8_t imu_start_reg = 0x1F;
    {
        std::string s = GetArgS(argc, argv, "--imu-start-reg", "0x1F");
        imu_start_reg = ParseU8HexOrDec(s, 0x1F);
    }

    // timing / robust
    const int64_t off_reject_ns = GetArgI64(argc, argv, "--off-reject-ns", 10'000'000); // 10ms
    const bool allow_empty_imu = HasArg(argc, argv, "--allow-empty-imu");

    // realtime (optional)
    const bool rt_imu = HasArg(argc, argv, "--rt-imu");
    const int rt_prio = GetArgI(argc, argv, "--rt-prio", 60);

    std::cerr << "ORB vocab=" << vocab << "\nsettings=" << settings << "\n";
    std::cerr << "cam " << w << "x" << h << " @" << fps
              << " ae_disable=" << (ae_disable ? "true" : "false") << " exp_us=" << exposure_us
              << " gain=" << gain << " request_y8=" << (request_y8 ? "Y" : "N")
              << " r16_norm=" << (r16_norm ? "Y" : "N") << "\n";
    std::cerr << "pair_thresh=" << pair_ms << "ms keep_window=" << keep_ms << "ms\n";
    std::cerr << "cam1_ts_offset_ms=" << cam1_ts_offset_ms
              << " auto_cam_offset=" << (auto_cam_offset ? "Y" : "N")
              << " auto_samples=" << auto_offset_samples
              << " auto_timeout_ms=" << auto_offset_timeout_ms << "\n";

    std::cerr << "imu spi=" << spi_dev << " speed=" << spi_speed << " mode=" << int(spi_mode)
              << " bits=" << int(spi_bits) << " drdy=" << gpiochip << ":" << drdy_line
              << " imu_hz=" << imu_hz << " imu_start_reg=0x" << std::hex << int(imu_start_reg)
              << std::dec << " accel_fs=" << accel_fs_g << "g" << " gyro_fs=" << gyro_fs_dps
              << "dps" << "\n";
    std::cerr << "udp_enable=" << (udp_enable ? "Y" : "N") << " udp_ip=" << udp_ip
              << " udp_port=" << udp_port << " jpeg_q=" << udp_jpeg_q << " payload=" << udp_payload
              << " queue=" << udp_q << "\n";

    // ORB-SLAM3 init
    ORB_SLAM3::System SLAM(vocab, settings, ORB_SLAM3::System::STEREO, false);
    MavlinkSerial mav("/dev/ttyAMA0", 921600);
    mav.startRx();
    Px4Console console(mav);
    console.start();
    // UDP sender (optional)
    UdpImageSender udp;
    if (udp_enable) {
        if (!udp.Open(udp_ip, udp_port, udp_jpeg_q, udp_payload, udp_q)) {
            std::cerr << "[udp] open failed, continue without udp.\n";
        } else {
            std::cerr << "[udp] sending to " << udp_ip << ":" << udp_port << "\n";
        }
    }

    // IMU thread
    ImuBuffer imu_buf;
    std::atomic<bool> imu_ok{false};
    std::atomic<uint64_t> imu_cnt{0};
    std::atomic<uint64_t> imu_drop{0};

    ImuScale imu_scale{};
    std::atomic<float> accel_lsb_per_g{imu_scale.accel_lsb_per_g};
    std::atomic<float> gyro_lsb_per_dps{imu_scale.gyro_lsb_per_dps};

    std::thread imu_th([&]() {
        if (rt_imu) {
            if (!SetThreadRealtime(rt_prio)) {
                std::cerr << "[imu] SetThreadRealtime failed (need CAP_SYS_NICE/root). continue.\n";
            } else {
                std::cerr << "[imu] realtime SCHED_FIFO prio=" << rt_prio << "\n";
            }
        }

        SpiDev spi(spi_dev);
        if (!spi.Open(spi_speed, spi_mode, spi_bits))
            return;

        ImuScale sc{};
        if (!IcmResetAndConfig(spi, imu_hz, accel_fs_g, gyro_fs_dps, sc))
            return;

        accel_lsb_per_g.store(sc.accel_lsb_per_g);
        gyro_lsb_per_dps.store(sc.gyro_lsb_per_dps);
        std::cerr << "[imu] scale accel_lsb/g=" << sc.accel_lsb_per_g
                  << " gyro_lsb/dps=" << sc.gyro_lsb_per_dps << "\n";

        DrdyGpio drdy;
        if (!drdy.Open(gpiochip, drdy_line))
            return;

        imu_ok.store(true);

        uint8_t raw12[12]{};
        uint8_t st = 0;
        spi.ReadReg(REG_INT_STATUS, st);

        while (g_running.load()) {
            int64_t t_irq_ns = 0;
            if (!drdy.WaitTs(1000, t_irq_ns))
                continue;

            ImuSample s{};
            s.t_ns = t_irq_ns;

            spi.ReadReg(REG_INT_STATUS, st);

            if (!spi.ReadRegs(imu_start_reg, raw12, sizeof(raw12))) {
                imu_drop.fetch_add(1, std::memory_order_relaxed);
                continue;
            }

            ImuScale sc2{};
            sc2.accel_lsb_per_g = accel_lsb_per_g.load();
            sc2.gyro_lsb_per_dps = gyro_lsb_per_dps.load();
            ConvertRaw12_AccelGyro_ToSI(raw12, sc2, s);

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
    if (!cam.Open(w, h, fps, ae_disable, exposure_us, gain, request_y8,
                  (int64_t)pair_ms * 1'000'000LL, (int64_t)keep_ms * 1'000'000LL, 8, r16_norm)) {
        g_running.store(false);
        if (imu_th.joinable())
            imu_th.join();
        return 1;
    }

    // Apply manual or auto cam1 offset
    if (cam1_ts_offset_ms != 0) {
        cam.SetCam1OffsetNs((int64_t)cam1_ts_offset_ms * 1'000'000LL);
        std::cerr << "[camoff] manual cam1_ts_offset_ms=" << cam1_ts_offset_ms << "\n";
    } else if (auto_cam_offset) {
        std::cerr << "[camoff] auto estimating cam1 offset... samples=" << auto_offset_samples
                  << " timeout_ms=" << auto_offset_timeout_ms << "\n";
        cam.AutoEstimateAndSetOffset(auto_offset_samples, auto_offset_timeout_ms);
    }

    // Warmup: estimate ts_offset_ns (libcamera timestamp domain -> NowNs domain)
    // Now that we can pair, this should work.
    std::vector<int64_t> offs;
    offs.reserve(120);

    int warm_pairs = 0;
    int64_t last_print = NowNs();
    while (g_running.load() && warm_pairs < 60) {
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
        int64_t cam_ts = (int64_t)((L.ts_ns + R.ts_ns) / 2);
        int64_t arrive = (L.arrive_ns && R.arrive_ns) ? (L.arrive_ns + R.arrive_ns) / 2 : NowNs();
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
    const int64_t imu_dt_ns = (int64_t)(1000000000LL / std::max(1, imu_hz));
    const int64_t slack_before_ns = std::max<int64_t>(2 * imu_dt_ns, 5'000'000);
    const int64_t slack_after_ns = std::max<int64_t>(2 * imu_dt_ns, 5'000'000);

    mav.updateStreamPosition(0.0f, 0.0f, -0.3f, NAN); // NED: z=-0.3 表示向上 0.3m
    mav.startSetpointStreamHz(30.0);

    while (g_running.load()) {
        FrameItem L, R;
        if (!cam.GrabPair(L, R, 1000)) {
            const int64_t now = NowNs();
            if (now - last_stat_ns > 1'000'000'000LL) {
                last_stat_ns = now;
                const uint64_t ic = imu_cnt.load();
                const uint64_t id = imu_drop.load();
                std::cerr << "[wd] no pair 1s" << " last_seq=" << cam.LastSeq()
                          << " dt_ms=" << cam.LastDtMs() << " pendL=" << cam.PendL()
                          << " pendR=" << cam.PendR() << " imu_hz~=" << (ic - last_imu_cnt)
                          << " imu_drop~=" << (id - last_imu_drop) << " imu_buf=" << imu_buf.Size()
                          << "\n";
                last_imu_cnt = ic;
                last_imu_drop = id;
            }
            continue;
        }

        frame_cnt_1s++;

        const int64_t cam_ts = (int64_t)((L.ts_ns + R.ts_ns) / 2);
        const int64_t arrive =
            (L.arrive_ns && R.arrive_ns) ? (L.arrive_ns + R.arrive_ns) / 2 : NowNs();
        const int64_t off_meas = arrive - cam_ts;

        const int64_t err = off_meas - ts_offset_ns;
        if (Abs64(err) < off_reject_ns) {
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

        const double frame_t = (double)frame_ns * 1e-9;

        const int64_t now = NowNs();
        last_stat_ns = 0;
        if (now - last_stat_ns > 1'000'000'000LL) {
            last_stat_ns = now;
            const uint64_t ic = imu_cnt.load();
            const uint64_t id = imu_drop.load();
            // std::cerr << "[STAT] fps~=" << frame_cnt_1s
            //           << " dt_ms=" << cam.LastDtMs()
            //           << " imu_hz~=" << (ic - last_imu_cnt)
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

            if (!allow_empty_imu)
                continue;
        }

        if (udp_enable) {
            udp.Enqueue(0, L.seq, frame_t, L.gray);
            udp.Enqueue(1, R.seq, frame_t, R.gray);
        }

        Sophus::SE3f Tcw = SLAM.TrackStereo(L.gray, R.gray, frame_t, vImu);
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
        mav.sendOdometry(t_us, p_ned, MAV_FRAME_LOCAL_NED, MAV_FRAME_BODY_FRD,
            state == ORB_SLAM3::Tracking::OK ? OdomQualityMode::GOOD : OdomQualityMode::LOST);

        static uint64_t posCnt = 0;
        if (posCnt % 100 == 0) {
            printf("[POSE]%f,(x:%f,y:%f,z:%f),(qw:%f,qx:%f,qy:%f,qz:%f)\n",
                frame_t, p_ned.x, p_ned.y, p_ned.z, p_ned.qw, p_ned.qx, p_ned.qy, p_ned.qz);
        }
        posCnt++;
    }

    cam.Close();
    g_running.store(false);
    if (imu_th.joinable())
        imu_th.join();
    SLAM.Shutdown();
    return 0;
}
