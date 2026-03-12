// ./calib_recorder --out ./calib_A --w 640 --h 400 --fps 30 --imu-hz 500 --ae-disable --exp-us 6000 --gain 4 --pair-tol-us 3000 --max-save-dt-us 2000 --udp-ip 192.168.0.101

#include <opencv2/opencv.hpp>

#include <gpiod.h>
#include <linux/spi/spidev.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "stereo_ov9281.hpp"
#include "udp_image_sender.hpp"

namespace fs = std::filesystem;

static uint32_t g_seq = 0;

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

static inline int16_t Be16ToI16(uint8_t hi, uint8_t lo)
{
    return (int16_t)((uint16_t(hi) << 8) | uint16_t(lo));
}

static constexpr uint8_t REG_DEVICE_CONFIG = 0x11;
static constexpr uint8_t REG_INT_CONFIG = 0x14;
static constexpr uint8_t REG_TEMP_DATA1 = 0x1D;
static constexpr uint8_t REG_INT_STATUS = 0x2D;
static constexpr uint8_t REG_PWR_MGMT0 = 0x4E;
static constexpr uint8_t REG_GYRO_CONFIG0 = 0x4F;
static constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50;
static constexpr uint8_t REG_INT_CONFIG1 = 0x64;
static constexpr uint8_t REG_INT_SOURCE0 = 0x65;
static constexpr uint8_t SPI_READ_MASK = 0x80;

struct ImuSample {
    int64_t tNs{};
    float ax{}, ay{}, az{};
    float gx{}, gy{}, gz{};
};

class SpiDev {
  public:
    explicit SpiDev(std::string dev) : m_dev(std::move(dev)) {}
    ~SpiDev()
    {
        if (m_fd >= 0)
            ::close(m_fd);
    }

    bool Open(uint32_t speed_hz, uint8_t mode, uint8_t bitsPerWord)
    {
        m_fd = ::open(m_dev.c_str(), O_RDWR);
        if (m_fd < 0) {
            std::cerr << "open " << m_dev << " failed: " << strerror(errno) << "\n";
            return false;
        }
        if (ioctl(m_fd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(m_fd, SPI_IOC_RD_MODE, &mode) < 0) {
            std::cerr << "SPI set mode failed: " << strerror(errno) << "\n";
            return false;
        }
        if (ioctl(m_fd, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) < 0 ||
            ioctl(m_fd, SPI_IOC_RD_BITS_PER_WORD, &bitsPerWord) < 0) {
            std::cerr << "SPI set bits failed: " << strerror(errno) << "\n";
            return false;
        }
        if (ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0 ||
            ioctl(m_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz) < 0) {
            std::cerr << "SPI set speed failed: " << strerror(errno) << "\n";
            return false;
        }
        m_speedHz = speed_hz;
        m_mode = mode;
        m_bits = bitsPerWord;
        return true;
    }

    bool WriteReg(uint8_t reg, uint8_t val)
    {
        uint8_t tx[2] = {uint8_t(reg & 0x7F), val};
        uint8_t rx[2] = {0, 0};
        return Transfer(tx, rx, sizeof(tx));
    }

    bool ReadReg(uint8_t reg, uint8_t &val)
    {
        uint8_t tx[2] = {uint8_t(SPI_READ_MASK | (reg & 0x7F)), 0x00};
        uint8_t rx[2] = {0, 0};
        if (!Transfer(tx, rx, sizeof(tx)))
            return false;
        val = rx[1];
        return true;
    }

    bool ReadRegs(uint8_t startReg, uint8_t *out, size_t len)
    {
        std::vector<uint8_t> tx(len + 1, 0x00);
        std::vector<uint8_t> rx(len + 1, 0x00);
        tx[0] = uint8_t(SPI_READ_MASK | (startReg & 0x7F));
        if (!Transfer(tx.data(), rx.data(), rx.size()))
            return false;
        memcpy(out, rx.data() + 1, len);
        return true;
    }

  private:
    bool Transfer(const uint8_t *tx, uint8_t *rx, size_t len)
    {
        spi_ioc_transfer tr{};
        tr.tx_buf = (unsigned long)tx;
        tr.rx_buf = (unsigned long)rx;
        tr.len = (uint32_t)len;
        tr.speed_hz = m_speedHz;
        tr.bits_per_word = m_bits;
        if (ioctl(m_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
            std::cerr << "SPI transfer failed: " << strerror(errno) << "\n";
            return false;
        }
        return true;
    }

    std::string m_dev;
    int m_fd{-1};
    uint32_t m_speedHz{8000000};
    uint8_t m_mode{SPI_MODE_0};
    uint8_t m_bits{8};
};

class DrdyGpio {
  public:
    bool Open(const std::string &chipPath, unsigned lineOffset, int maxBurst = 256)
    {
        m_chip = gpiod_chip_open(chipPath.c_str());
        if (!m_chip) {
            std::cerr << "gpiod_chip_open(" << chipPath << ") failed: " << strerror(errno) << "\n";
            return false;
        }

        gpiod_line_settings *settings = gpiod_line_settings_new();
        if (!settings)
            return false;
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
        gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);
        gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);

        gpiod_line_config *lineConfig = gpiod_line_config_new();
        if (!lineConfig) {
            gpiod_line_settings_free(settings);
            return false;
        }

        unsigned offsets[1] = {lineOffset};
        const int rc = gpiod_line_config_add_line_settings(lineConfig, offsets, 1, settings);
        gpiod_line_settings_free(settings);
        if (rc < 0) {
            gpiod_line_config_free(lineConfig);
            return false;
        }

        gpiod_request_config *requestConfig = gpiod_request_config_new();
        if (!requestConfig) {
            gpiod_line_config_free(lineConfig);
            return false;
        }
        gpiod_request_config_set_consumer(requestConfig, "icm42688_drdy");

        m_request = gpiod_chip_request_lines(m_chip, requestConfig, lineConfig);
        gpiod_request_config_free(requestConfig);
        gpiod_line_config_free(lineConfig);

        if (!m_request) {
            std::cerr << "gpiod_chip_request_lines failed: " << strerror(errno) << "\n";
            return false;
        }

        m_maxBurst = std::max(1, maxBurst);
        m_evbuf = gpiod_edge_event_buffer_new((size_t)m_maxBurst);
        return m_evbuf != nullptr;
    }

    ~DrdyGpio()
    {
        if (m_evbuf)
            gpiod_edge_event_buffer_free(m_evbuf);
        if (m_request)
            gpiod_line_request_release(m_request);
        if (m_chip)
            gpiod_chip_close(m_chip);
    }

    bool WaitLastTs(int timeoutMs, int64_t &tsNsOut)
    {
        const int64_t timeoutNs = (timeoutMs < 0) ? -1 : (int64_t)timeoutMs * 1000000LL;
        const int ret = gpiod_line_request_wait_edge_events(m_request, timeoutNs);
        if (ret <= 0)
            return false;

        const int n = gpiod_line_request_read_edge_events(m_request, m_evbuf, (size_t)m_maxBurst);
        if (n <= 0)
            return false;

        struct gpiod_edge_event *evLast =
            gpiod_edge_event_buffer_get_event(m_evbuf, (size_t)(n - 1));
        if (!evLast)
            return false;

        tsNsOut = (int64_t)gpiod_edge_event_get_timestamp_ns(evLast);
        return true;
    }

  private:
    gpiod_chip *m_chip{nullptr};
    gpiod_line_request *m_request{nullptr};
    gpiod_edge_event_buffer *m_evbuf{nullptr};
    int m_maxBurst{256};
};

static bool IcmResetAndConfig(SpiDev &spi, int imuHz)
{
    const auto odrCode = [&](int hz) -> uint8_t {
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
    };

    spi.WriteReg(REG_DEVICE_CONFIG, 0x01);
    usleep(100000);

    spi.WriteReg(REG_INT_CONFIG, 0x30);
    spi.WriteReg(REG_INT_SOURCE0, 0x08);
    spi.WriteReg(REG_INT_CONFIG1, 0x00);
    spi.WriteReg(REG_PWR_MGMT0, 0x0F);
    usleep(20000);

    const uint8_t odr = odrCode(imuHz);
    const uint8_t gyro_cfg0 = uint8_t((0x00 << 5) | (odr & 0x0F));
    const uint8_t accel_cfg0 = uint8_t((0x00 << 5) | (odr & 0x0F));
    if (!spi.WriteReg(REG_GYRO_CONFIG0, gyro_cfg0))
        return false;
    if (!spi.WriteReg(REG_ACCEL_CONFIG0, accel_cfg0))
        return false;
    usleep(20000);
    return true;
}

static void ConvertRawToSI(const uint8_t raw[14], ImuSample &s)
{
    const int16_t ax = Be16ToI16(raw[2], raw[3]);
    const int16_t ay = Be16ToI16(raw[4], raw[5]);
    const int16_t az = Be16ToI16(raw[6], raw[7]);
    const int16_t gx = Be16ToI16(raw[8], raw[9]);
    const int16_t gy = Be16ToI16(raw[10], raw[11]);
    const int16_t gz = Be16ToI16(raw[12], raw[13]);

    constexpr float kG = 9.80665f;
    constexpr float accelLsbPerG = 2048.0f;
    constexpr float gyroLsbPerDps = 16.4f;
    constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;

    s.ax = (float(ax) / accelLsbPerG) * kG;
    s.ay = (float(ay) / accelLsbPerG) * kG;
    s.az = (float(az) / accelLsbPerG) * kG;
    s.gx = (float(gx) / gyroLsbPerDps) * kDeg2Rad;
    s.gy = (float(gy) / gyroLsbPerDps) * kDeg2Rad;
    s.gz = (float(gz) / gyroLsbPerDps) * kDeg2Rad;
}

static void EnsureDir(const fs::path &p)
{
    std::error_code ec;
    fs::create_directories(p, ec);
    if (ec)
        std::cerr << "create_directories failed: " << p << " : " << ec.message() << "\n";
}

static std::string TsToName(int64_t tNs)
{
    return std::to_string(tNs) + ".png";
}

static void SetupFileBuffer(FILE *f, size_t bytes)
{
    if (f)
        setvbuf(f, nullptr, _IOFBF, bytes);
}

int main(int argc, char **argv)
{
    signal(SIGINT, SigIntHandler);
    signal(SIGTERM, SigIntHandler);

    const std::string outRoot = GetArgS(argc, argv, "--out", "./calib_out");
    const std::string udpIp = GetArgS(argc, argv, "--udp-ip", "10.42.0.109");
    const int udpPort = GetArgI(argc, argv, "--udp-port", 14550);

    const int w = GetArgI(argc, argv, "--w", 640);
    const int h = GetArgI(argc, argv, "--h", 400);
    const int fps = GetArgI(argc, argv, "--fps", 30);
    const bool aeDisable = HasArg(argc, argv, "--ae-disable");
    const int exposureUs = GetArgI(argc, argv, "--exp-us", 3000);
    const float gain = GetArgF(argc, argv, "--gain", 2.0f);

    const std::string spiDev = GetArgS(argc, argv, "--spi", "/dev/spidev0.0");
    const uint32_t spiSpeed = (uint32_t)GetArgI(argc, argv, "--speed", 8000000);
    const uint8_t spiMode = (uint8_t)GetArgI(argc, argv, "--mode", 0);
    const uint8_t spiBits = (uint8_t)GetArgI(argc, argv, "--bits", 8);

    const std::string gpiochip = GetArgS(argc, argv, "--gpiochip", "/dev/gpiochip0");
    const unsigned drdyLine = (unsigned)GetArgI(argc, argv, "--drdy", 24);
    const int imuHz = GetArgI(argc, argv, "--imu-hz", 500);
    const int pairTolUs = GetArgI(argc, argv, "--pair-tol-us", 15000);
    const int maxSaveDtUs = GetArgI(argc, argv, "--max-save-dt-us", pairTolUs);
    const int pairSearchWindow = GetArgI(argc, argv, "--pair-search-window", 6);

    const int imuFlushEvery = GetArgI(argc, argv, "--imu-flush-every", 800);
    const int drdyBurst = GetArgI(argc, argv, "--drdy-burst", 256);

    const int maxFrames = GetArgI(argc, argv, "--max-frames", -1);
    const int64_t pairTolNs = static_cast<int64_t>(std::max(1, pairTolUs)) * 1000LL;
    const int64_t framePeriodNs = 1000000000LL / std::max(1, fps);
    const int64_t keepWindowNs =
        std::max<int64_t>(pairTolNs * std::max(2, pairSearchWindow), framePeriodNs * pairSearchWindow);

    std::cerr << "out=" << outRoot << "\n";
    std::cerr << "cam " << w << "x" << h << " @" << fps
              << " aeDisable=" << (aeDisable ? "true" : "false")
              << " exp_us=" << exposureUs << " gain=" << gain
              << " pixelFormat=R16\n";
    std::cerr << "imu spi=" << spiDev << " speed=" << spiSpeed << " mode=" << int(spiMode)
              << " bits=" << int(spiBits) << " drdy=" << gpiochip << ":" << drdyLine
              << " imuHz=" << imuHz << " imuFlushEvery=" << imuFlushEvery
              << " drdyBurst=" << drdyBurst << "\n";
    std::cerr << "pairTolUs=" << pairTolUs << " maxSaveDtUs=" << maxSaveDtUs
              << " pairSearchWindow=" << pairSearchWindow
              << " keepWindowMs=" << (keepWindowNs / 1e6) << "\n";

    const fs::path root(outRoot);
    const fs::path cam0Data = root / "cam0";
    const fs::path cam1Data = root / "cam1";
    EnsureDir(cam0Data);
    EnsureDir(cam1Data);

    UdpImageSender udp;
    if (!udp.Open(udpIp, udpPort, 45)) {
        std::cerr << "[udp] open failed, continue without udp.\n";
    } else {
        std::cerr << "[udp] sending to " << udpIp << ":" << udpPort << "\n";
    }

    FILE *fCam0 = std::fopen((cam0Data / "data.csv").c_str(), "w");
    FILE *fCam1 = std::fopen((cam1Data / "data.csv").c_str(), "w");
    FILE *fImu = std::fopen((root / "imu.csv").c_str(), "w");
    if (!fCam0 || !fCam1 || !fImu) {
        std::cerr << "Failed to open output csv files.\n";
        return 1;
    }

    SetupFileBuffer(fCam0, 1 << 20);
    SetupFileBuffer(fCam1, 1 << 20);
    SetupFileBuffer(fImu, 4 << 20);

    std::fprintf(fCam0, "#timestamp [ns],filename\n");
    std::fprintf(fCam1, "#timestamp [ns],filename\n");
    std::fprintf(
        fImu,
        "#timestamp [ns],wX [rad/s],wY [rad/s],wZ [rad/s],aX [m/s^2],aY [m/s^2],aZ [m/s^2]\n");

    std::atomic<bool> imuOk{false};
    std::thread imuThread([&]() {
        SpiDev spi(spiDev);
        if (!spi.Open(spiSpeed, spiMode, spiBits))
            return;
        if (!IcmResetAndConfig(spi, imuHz))
            return;

        DrdyGpio drdy;
        if (!drdy.Open(gpiochip, drdyLine, drdyBurst))
            return;

        imuOk.store(true);

        uint8_t raw[14]{};
        uint8_t st = 0;
        spi.ReadReg(REG_INT_STATUS, st);

        int imuLines = 0;
        while (g_runningFlag.load()) {
            int64_t tIrqNs = 0;
            if (!drdy.WaitLastTs(1000, tIrqNs))
                continue;

            ImuSample sample{};
            sample.tNs = tIrqNs;

            spi.ReadReg(REG_INT_STATUS, st);
            if (!spi.ReadRegs(REG_TEMP_DATA1, raw, sizeof(raw)))
                continue;
            ConvertRawToSI(raw, sample);

            std::fprintf(
                fImu,
                "%lld,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                (long long)sample.tNs,
                (double)sample.gx,
                (double)sample.gy,
                (double)sample.gz,
                (double)sample.ax,
                (double)sample.ay,
                (double)sample.az);

            imuLines++;
            if (imuFlushEvery > 0 && (imuLines % imuFlushEvery) == 0)
                std::fflush(fImu);
        }

        std::fflush(fImu);
    });

    LibcameraStereoOV9281_TsPair cam;
    if (!cam.Open(w, h, fps, aeDisable, exposureUs, gain, true, pairTolNs, keepWindowNs, 8, false)) {
        g_runningFlag.store(false);
        if (imuThread.joinable())
            imuThread.join();
        return 1;
    }

    int saved = 0;
    int droppedWidePairs = 0;
    int64_t lastPairNs = 0;

    std::cerr << "Recording... press Ctrl+C to stop\n";
    while (g_runningFlag.load()) {
        if (maxFrames > 0 && saved >= maxFrames)
            break;

        FrameItem L, R;
        if (!cam.GrabPair(L, R, 1000))
            continue;

        const int64_t dtLr = (int64_t)L.tsNs - (int64_t)R.tsNs;
        const int64_t absDtLr = Abs64(dtLr);
        if ((saved % 30) == 0) {
            std::cerr << "[pair] dt_lr_us=" << (absDtLr / 1000.0)
                      << " tol_us=" << (cam.PairTolNs() / 1000.0) << "\n";
            std::cerr << "[imu] ok=" << (imuOk.load() ? "true" : "false") << "\n";
        }

        if (absDtLr > int64_t(std::max(0, maxSaveDtUs)) * 1000LL) {
            droppedWidePairs++;
            if ((droppedWidePairs % 10) == 1) {
                std::cerr << "[pair-drop] dt_lr_us=" << (absDtLr / 1000.0)
                          << " exceeds max_save_dt_us=" << maxSaveDtUs
                          << " dropped=" << droppedWidePairs << "\n";
            }
            continue;
        }

        int64_t pairNs = (int64_t)((L.tsNs + R.tsNs) / 2);
        if (lastPairNs != 0 && pairNs <= lastPairNs)
            pairNs = lastPairNs + 1;
        lastPairNs = pairNs;

        const std::string nameL = TsToName(pairNs);
        const std::string nameR = TsToName(pairNs);
        const fs::path fnL = cam0Data / nameL;
        const fs::path fnR = cam1Data / nameR;

        if (!cv::imwrite(fnL.string(), L.gray)) {
            std::cerr << "imwrite failed: " << fnL << "\n";
            continue;
        }
        if (!cv::imwrite(fnR.string(), R.gray)) {
            std::cerr << "imwrite failed: " << fnR << "\n";
            continue;
        }

        std::fprintf(fCam0, "%lld,%s\n", (long long)pairNs, nameL.c_str());
        std::fprintf(fCam1, "%lld,%s\n", (long long)pairNs, nameR.c_str());

        udp.Enqueue(0, g_seq, pairNs * 1e-9, L.gray);
        udp.Enqueue(1, g_seq++, pairNs * 1e-9, R.gray);
        if ((saved % 50) == 0) {
            std::fflush(fCam0);
            std::fflush(fCam1);
            std::cerr << "saved pairs=" << saved << "\n";
        }
        saved++;
    }

    std::cerr << "Stopping...\n";
    cam.Close();
    g_runningFlag.store(false);
    if (imuThread.joinable())
        imuThread.join();

    std::fflush(fCam0);
    std::fflush(fCam1);
    std::fflush(fImu);
    std::fclose(fCam0);
    std::fclose(fCam1);
    std::fclose(fImu);

    std::cerr << "Done. Saved pairs=" << saved << "\n";
    std::cerr << "Dropped wide pairs=" << droppedWidePairs << "\n";
    return 0;
}
