#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <libcamera/camera_manager.h>
#include <opencv2/core.hpp>
#include "FrameSyncQueue.h"
#include "FrameTypes.h"
#include "Icm20948Poller.h"
#include "LibcameraStereo.h"
#include "TimeUtils.h"
#include "CvUtils.h"

static std::atomic<bool> g_run(true);

static void OnSigint(int) { g_run.store(false); }

static std::string JoinPath(const std::string &a, const std::string &b)
{
    return (std::filesystem::path(a) / std::filesystem::path(b)).string();
}

static bool WritePgmFromYPlane(const FrameItem &item, const std::string &path)
{
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs.is_open())
        return false;

    // Binary PGM header
    ofs << "P5\n" << item.m_width << " " << item.m_height << "\n255\n";

    const uint8_t *y = item.m_data->data();
    for (unsigned int r = 0; r < item.m_height; r++) {
        ofs.write(reinterpret_cast<const char *>(y + r * item.m_stride),
                  static_cast<std::streamsize>(item.m_width));
    }
    return ofs.good();
}

class UdpFrameStreamer {
  public:
    UdpFrameStreamer() : m_fp(nullptr), m_enabled(false), m_w(0), m_h(0), m_lastSendNs(0) {}

    ~UdpFrameStreamer() { Stop(); }

    bool Start(const std::string &dstUrl, unsigned int w, unsigned int h)
    {
        Stop();
        if (dstUrl.empty())
            return false;

        m_w = w;
        m_h = h;

        std::string cmd =
            "ffmpeg -hide_banner -loglevel error "
            "-f rawvideo -pix_fmt yuv420p -s " + std::to_string(w) + "x" + std::to_string(h) +
            " -r 10 -i - "
            "-c:v libx264 -preset ultrafast -tune zerolatency -g 10 -bf 0 -b:v 300k "
            "-f mpegts " + dstUrl;

        // popen uses /bin/sh -c; redirect stderr already handled by ffmpeg flags
        m_fp = ::popen(cmd.c_str(), "w");
        if (!m_fp) {
            std::cerr << "[STREAM] popen failed\n";
            m_enabled = false;
            return false;
        }

        m_enabled = true;
        m_lastSendNs = 0;
        std::cerr << "[STREAM] started: " << dstUrl << " (1 fps)\n";
        return true;
    }

    void Stop()
    {
        m_enabled = false;
        if (m_fp) {
            ::pclose(m_fp);
            m_fp = nullptr;
        }
    }

    bool Enabled() const { return m_enabled; }

    // best-effort: send at most 1 frame per second
    void MaybeSendGray(const cv::Mat &gray, uint64_t tsNs)
    {
        if (!m_enabled || !m_fp)
            return;
        if (gray.empty())
            return;
        if (gray.cols != static_cast<int>(m_w) || gray.rows != static_cast<int>(m_h))
            return;

        if (m_lastSendNs != 0) {
            uint64_t dt = tsNs - m_lastSendNs;
            const uint64_t intervalNs = 100ULL * 1000ULL * 1000ULL; // 200ms = 10fps
            if (dt < intervalNs)
                return;
        }

        // gray (Y) -> yuv420p: Y=gray, U=128, V=128
        const size_t ySize = static_cast<size_t>(m_w) * static_cast<size_t>(m_h);
        const size_t uvSize = ySize / 4;
        const size_t total = ySize + uvSize + uvSize;

        m_buf.resize(total);
        std::memcpy(m_buf.data(), gray.data, ySize);
        std::memset(m_buf.data() + ySize, 128, uvSize + uvSize);

        size_t written = std::fwrite(m_buf.data(), 1, total, m_fp);
        if (written != total) {
            std::cerr << "[STREAM] write failed; disabling streaming\n";
            Stop();
            return;
        }
        std::fflush(m_fp);
        m_lastSendNs = tsNs;
    }

  private:
    FILE *m_fp;
    bool m_enabled;
    unsigned int m_w, m_h;
    uint64_t m_lastSendNs;
    std::vector<uint8_t> m_buf;
};

int main(int argc, char **argv)
{
    std::signal(SIGINT, OnSigint);
    std::signal(SIGPIPE, SIG_IGN); // avoid crash if ffmpeg/udp side breaks

    // argv[1] outDir (default ./record)
    // argv[2] width  (default 640)
    // argv[3] height (default 480)
    // argv[4] streamDstUrl (optional, e.g. udp://ip:port)
    std::string outDir = "./record";
    unsigned int width = 640;
    unsigned int height = 480;
    std::string streamUrl;

    try {
        if (argc >= 2) outDir = argv[1];
        if (argc >= 3) width = static_cast<unsigned int>(std::stoul(argv[2]));
        if (argc >= 4) height = static_cast<unsigned int>(std::stoul(argv[3]));
        if (argc >= 5) streamUrl = argv[4];
    } catch (...) {
        std::cerr << "Usage: " << argv[0] << " [outDir] [width] [height] [udp://ip:port]\n";
        return 1;
    }

    const std::string leftDir = JoinPath(outDir, "left");
    const std::string rightDir = JoinPath(outDir, "right");

    std::error_code ec;
    std::filesystem::create_directories(leftDir, ec);
    std::filesystem::create_directories(rightDir, ec);

    std::ofstream framesCsv(JoinPath(outDir, "frames.csv"));
    std::ofstream imuCsv(JoinPath(outDir, "imu.csv"));
    if (!framesCsv.is_open() || !imuCsv.is_open()) {
        std::cerr << "Failed to open output files in: " << outDir << "\n";
        return 1;
    }

    framesCsv << "ts_left_ns,ts_right_ns,left_file,right_file,cam_offset_right_minus_left_ns\n";
    imuCsv << "ts_ns,ax,ay,az,gx,gy,gz\n";

    // ---- IMU ----
    Icm20948Poller::Options imuOpt;
    imuOpt.m_i2cDev = "/dev/i2c-1";
    imuOpt.m_addr = 0x68;
    imuOpt.m_rateHz = 200.0;
    imuOpt.m_calibSeconds = 2.0;
    imuOpt.m_ringSeconds = 60;
    imuOpt.m_realtimeFifo = true;
    imuOpt.m_fifoPrio = 80;

    Icm20948Poller imu(imuOpt);
    imu.Start();
    std::cerr << "[IMU] calibrating... keep still for " << imuOpt.m_calibSeconds << "s\n";

    // ---- Cameras ----
    libcamera::CameraManager cameraManager;
    if (cameraManager.start()) {
        std::cerr << "CameraManager start failed\n";
        imu.Stop();
        return 1;
    }

    auto cameras = cameraManager.cameras();
    if (cameras.size() < 2) {
        std::cerr << "Need 2 cameras; found " << cameras.size() << "\n";
        cameraManager.stop();
        imu.Stop();
        return 1;
    }

    FrameSyncQueue frameQueue;
    LibcameraStereo stereo(frameQueue);

    if (!stereo.Start(cameras[0], cameras[1], width, height)) {
        std::cerr << "Stereo start failed\n";
        stereo.Stop();
        cameraManager.stop();
        imu.Stop();
        return 1;
    }

    UdpFrameStreamer streamer;
    if (!streamUrl.empty()) {
        streamer.Start(streamUrl, width, height);
    }

    const uint64_t maxDtNs = 30ULL * 1000ULL * 1000ULL; // 30ms pairing tolerance
    CamOffsetLearner offsetLearner;

    double lastFrameT = -1.0;
    uint64_t nFrames = 0;
    uint64_t nImu = 0;

    std::cerr << "[REC] writing to: " << outDir << "\n";
    if (streamer.Enabled())
        std::cerr << "[REC] streaming enabled: " << streamUrl << " (1 fps)\n";
    std::cerr << "[REC] press Ctrl+C to stop.\n";

    while (g_run.load()) {
        FrameItem left;
        FrameItem right;

        if (!frameQueue.PopSyncedPair(left, right, maxDtNs)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (!offsetLearner.HaveOffset())
            offsetLearner.UpdateAndCheckReady(left.m_tsNs, right.m_tsNs);

        const double t = TimeUtils::NsToSec(left.m_tsNs);

        // Dump IMU samples between last frame time and this frame time.
        if (lastFrameT > 0.0) {
            std::vector<ORB_SLAM3::IMU::Point> vImu = imu.GetBetween(lastFrameT, t);
            for (const auto &p : vImu) {
                uint64_t tsNs = static_cast<uint64_t>(p.t * 1e9);
                imuCsv << tsNs << ","
                       << p.a.x() << "," << p.a.y() << "," << p.a.z() << ","
                       << p.w.x() << "," << p.w.y() << "," << p.w.z() << "\n";
                nImu++;
            }
        }

        // Optional: stream left frame at 1 fps (using left timestamp)
        if (streamer.Enabled()) {
            cv::Mat grayL = CvUtils::YPlaneToGrayMat(left);
            streamer.MaybeSendGray(grayL, left.m_tsNs);
        }

        // Write PGM images.
        const std::string leftName = std::to_string(left.m_tsNs) + ".pgm";
        const std::string rightName = std::to_string(right.m_tsNs) + ".pgm";
        const std::string leftPath = JoinPath(leftDir, leftName);
        const std::string rightPath = JoinPath(rightDir, rightName);

        if (!WritePgmFromYPlane(left, leftPath)) {
            std::cerr << "[REC] failed to write " << leftPath << "\n";
            break;
        }
        if (!WritePgmFromYPlane(right, rightPath)) {
            std::cerr << "[REC] failed to write " << rightPath << "\n";
            break;
        }

        const double camOffsetNs = offsetLearner.HaveOffset() ? offsetLearner.GetOffsetNs() : 0.0;
        framesCsv << left.m_tsNs << "," << right.m_tsNs << ","
                  << "left/" << leftName << ","
                  << "right/" << rightName << ","
                  << camOffsetNs << "\n";

        nFrames++;
        lastFrameT = t;

        if ((nFrames % 30) == 0) {
            std::cerr << "[REC] frames=" << nFrames << " imu=" << nImu
                      << " cam_offset_ms=" << (camOffsetNs * 1e-6) << "\n";
            framesCsv.flush();
            imuCsv.flush();
        }
    }

    framesCsv.flush();
    imuCsv.flush();

    streamer.Stop();
    imu.Stop();
    stereo.Stop();
    cameraManager.stop();

    // Write info
    {
        std::ofstream info(JoinPath(outDir, "info.txt"));
        info << "width=" << width << "\n";
        info << "height=" << height << "\n";
        info << "imu_rate_hz=" << imuOpt.m_rateHz << "\n";
        info << "pair_max_dt_ns=" << maxDtNs << "\n";
        info << "frames=" << nFrames << "\n";
        info << "imu_samples=" << nImu << "\n";
        info << "cam_offset_right_minus_left_ns=" << (offsetLearner.HaveOffset() ? offsetLearner.GetOffsetNs() : 0.0) << "\n";
        info << "timebase=camera_ts_ns_and_imu_monotonic_sec\n";
        info << "note=imu.csv ts_ns derived from imu point.t (monotonic)\n";
        if (!streamUrl.empty())
            info << "stream_url=" << streamUrl << "\n";
    }

    std::cerr << "[REC] done. frames=" << nFrames << " imu=" << nImu << " output=" << outDir << "\n";
    return 0;
}