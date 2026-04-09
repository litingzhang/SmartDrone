#include "core/application/session/calib_session_service.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include <fcntl.h>
#include <unistd.h>

#include "adapters/stream/udp_image_sender.h"
#include "common/tlv/tlv_protocol.h"
#include "core/application/session/calib_storage_helpers.h"
#include "core/application/session/runtime_session_common.h"
#include "core/application/session/sensor_runtime_helpers.h"

namespace smartdrone::core::application {

cv::Mat EnsureGray8ForCalib(const cv::Mat &src, bool &convertedOut)
{
    convertedOut = false;
    if (src.empty()) {
        return src;
    }
    if (src.type() == CV_8UC1) {
        return src;
    }
    cv::Mat out;
    if (src.type() == CV_16UC1) {
        src.convertTo(out, CV_8U, 1.0 / 256.0);
    } else {
        src.convertTo(out, CV_8U);
    }
    convertedOut = true;
    return out;
}

void FlushAndSyncFile(FILE *file, const char *label)
{
    if (!file) {
        return;
    }
    std::fflush(file);
    const int fd = ::fileno(file);
    if (fd >= 0 && ::fsync(fd) != 0) {
        std::cerr << "[calib-sync] fsync failed label=" << label << " errno=" << errno << "\n";
    }
}

void SyncPathFile(const fs::path &path)
{
    const int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) {
        std::cerr << "[calib-sync] open failed path=" << path.string() << " errno=" << errno << "\n";
        return;
    }
    if (::fsync(fd) != 0) {
        std::cerr << "[calib-sync] fsync failed path=" << path.string() << " errno=" << errno << "\n";
    }
    ::close(fd);
}

void SyncDirPath(const fs::path &path)
{
    const int fd = ::open(path.c_str(), O_RDONLY | O_DIRECTORY);
    if (fd < 0) {
        std::cerr << "[calib-sync] open dir failed path=" << path.string() << " errno=" << errno << "\n";
        return;
    }
    if (::fsync(fd) != 0) {
        std::cerr << "[calib-sync] fsync dir failed path=" << path.string() << " errno=" << errno << "\n";
    }
    ::close(fd);
}

void FlushCalibOutputs(FILE *fCam0, FILE *fCam1, FILE *fImu, const std::vector<fs::path> &imagePaths,
                       const fs::path &root, const fs::path &cam0Dir, const fs::path &cam1Dir)
{
    FlushAndSyncFile(fCam0, "cam0.csv");
    FlushAndSyncFile(fCam1, "cam1.csv");
    FlushAndSyncFile(fImu, "imu.csv");

    for (const auto &imagePath : imagePaths) {
        SyncPathFile(imagePath);
    }

    SyncDirPath(cam0Dir);
    SyncDirPath(cam1Dir);
    SyncDirPath(root);
}

bool RunCalibSession(const UnifiedConfig &cfg, std::atomic<bool> &stop, LivePoseState &livePose,
                     std::atomic<bool> &runningFlag)
{
    const MainRuntimeAliases a = BuildRuntimeAliases(cfg.app);
    PrintStartupConfig(cfg.app, a, ControllerMode::Calib);
    livePose.SetRuntimeMode(RUNTIME_MODE_CALIB);
    const std::string outRoot = MakeCalibSessionDir(cfg.calib.root);
    const fs::path root(outRoot);
    const fs::path cam0Dir = root / "cam0";
    const fs::path cam1Dir = root / "cam1";
    EnsureDir(cam0Dir);
    EnsureDir(cam1Dir);
    FILE *fCam0 = std::fopen((cam0Dir / "data.csv").string().c_str(), "w");
    FILE *fCam1 = std::fopen((cam1Dir / "data.csv").string().c_str(), "w");
    FILE *fImu = std::fopen((root / "imu.csv").string().c_str(), "w");
    if (!fCam0 || !fCam1 || !fImu)
        return false;
    std::cerr << "[calib] out=" << outRoot << "\n";
    SetupFileBuffer(fCam0, 1 << 20);
    SetupFileBuffer(fCam1, 1 << 20);
    SetupFileBuffer(fImu, 4 << 20);
    std::fprintf(fCam0, "#timestamp [ns],filename\n");
    std::fprintf(fCam1, "#timestamp [ns],filename\n");
    std::fprintf(fImu, "#timestamp [ns],wX [rad/s],wY [rad/s],wZ [rad/s],aX [m/s^2],aY [m/s^2],aZ [m/s^2]\n");
    UdpImageSender udp;
    if (a.udpEnable && a.sendImage)
        udp.Open(a.udpIp, a.udpPort, a.udpJpegQ, a.udpPayload, a.udpQueue);
    std::atomic<bool> imuOk{false};
    std::thread imuThread = StartCalibImuWriterThread(a, fImu, imuOk, stop, runningFlag);
    LibcameraStereoOV9281_TsPair cam;
    if (!OpenCamera(cam, a)) {
        stop.store(true);
        if (imuThread.joinable())
            imuThread.join();
        std::fclose(fCam0);
        std::fclose(fCam1);
        std::fclose(fImu);
        return false;
    }
    int saved = 0;
    std::vector<fs::path> savedImagePaths;
    int64_t lastPairNs = 0;
    const int64_t maxSaveDtNs = static_cast<int64_t>(std::max(a.pairMs, 1)) * 1000000LL;
    bool sessionOk = true;
    while (runningFlag.load() && !stop.load()) {
        if (cfg.calib.maxFrames > 0 && saved >= cfg.calib.maxFrames)
            break;
        FrameItem L;
        FrameItem R;
        if (!cam.GrabPair(L, R, 1000)) {
            if (!cam.Healthy()) {
                std::cerr << "[calib] camera pipeline unhealthy, aborting session\n";
                sessionOk = false;
                break;
            }
            continue;
        }
        const int64_t absDtLr = std::llabs(static_cast<int64_t>(L.tsNs) - static_cast<int64_t>(R.tsNs));
        if (absDtLr > maxSaveDtNs) {
            static int droppedWide = 0;
            ++droppedWide;
            if ((droppedWide % 10) == 1) {
                std::cerr << "[calib-drop] dt_lr_us=" << (absDtLr / 1000.0)
                          << " exceeds max_save_dt_us=" << (maxSaveDtNs / 1000.0) << " dropped=" << droppedWide << "\n";
            }
            continue;
        }
        int64_t pairNs = static_cast<int64_t>((L.tsNs + R.tsNs) / 2);
        if (lastPairNs != 0 && pairNs <= lastPairNs)
            pairNs = lastPairNs + 1;
        lastPairNs = pairNs;
        const std::string name = TsToName(pairNs);
        const fs::path fnL = cam0Dir / name;
        const fs::path fnR = cam1Dir / name;
        if (L.gray.empty() || R.gray.empty()) {
            std::cerr << "[calib-write] empty image"
                      << " seqL=" << L.seq << " seqR=" << R.seq << " rowsL=" << L.gray.rows << " colsL=" << L.gray.cols
                      << " rowsR=" << R.gray.rows << " colsR=" << R.gray.cols << "\n";
            continue;
        }
        bool convertedL = false;
        bool convertedR = false;
        const cv::Mat calibGrayL = EnsureGray8ForCalib(L.gray, convertedL);
        const cv::Mat calibGrayR = EnsureGray8ForCalib(R.gray, convertedR);
        if (convertedL || convertedR) {
            static int conversionLogCount = 0;
            ++conversionLogCount;
            if ((conversionLogCount % 30) == 1) {
                std::cerr << "[calib-gray] converted to 8-bit"
                          << " typeL=" << L.gray.type() << " typeR=" << R.gray.type()
                          << " count=" << conversionLogCount << "\n";
            }
        }
        const bool okL = cv::imwrite(fnL.string(), calibGrayL);
        const bool okR = cv::imwrite(fnR.string(), calibGrayR);
        if (!okL || !okR) {
            std::cerr << "[calib-write] imwrite failed"
                      << " okL=" << (okL ? "true" : "false") << " okR=" << (okR ? "true" : "false")
                      << " pathL=" << fnL.string() << " pathR=" << fnR.string() << " typeL=" << L.gray.type()
                      << " typeR=" << R.gray.type() << " rowsL=" << L.gray.rows << " colsL=" << L.gray.cols
                      << " rowsR=" << R.gray.rows << " colsR=" << R.gray.cols << "\n";
            continue;
        }
        std::fprintf(fCam0, "%lld,%s\n", static_cast<long long>(pairNs), name.c_str());
        std::fprintf(fCam1, "%lld,%s\n", static_cast<long long>(pairNs), name.c_str());
        savedImagePaths.push_back(fnL);
        savedImagePaths.push_back(fnR);
        if (a.udpEnable && a.sendImage) {
            udp.Enqueue(0, static_cast<uint64_t>(L.seq), L.seq, pairNs * 1e-9, calibGrayL, {}, true, false);
            udp.Enqueue(1, static_cast<uint64_t>(R.seq), R.seq, pairNs * 1e-9, calibGrayR, {}, true, false);
        }
        if ((saved % 30) == 0) {
            std::cerr << "[calib-save] saved=" << (saved + 1) << " pathL=" << fnL.string() << " pathR=" << fnR.string()
                      << "\n";
        }
        if ((++saved % 50) == 0) {
            std::fflush(fCam0);
            std::fflush(fCam1);
        }
    }
    cam.Close();
    std::cerr << "[session] calib camera closed\n";
    stop.store(true);
    if (imuThread.joinable())
        imuThread.join();
    std::cerr << "[session] calib imu joined\n";
    if (a.udpEnable && a.sendImage) {
        udp.Close();
        std::cerr << "[session] calib udp closed\n";
    }
    std::cerr << "[calib-sync] flushing outputs on calib stop saved=" << saved << "\n";
    FlushCalibOutputs(fCam0, fCam1, fImu, savedImagePaths, root, cam0Dir, cam1Dir);
    std::fclose(fCam0);
    std::fclose(fCam1);
    std::fclose(fImu);
    std::cerr << "[calib] out=" << outRoot << " saved=" << saved << " imuOk=" << (imuOk.load() ? "true" : "false")
              << "\n";
    livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
    std::cerr << "[session] calib exit\n";
    return sessionOk;
}

} // namespace smartdrone::core::application
