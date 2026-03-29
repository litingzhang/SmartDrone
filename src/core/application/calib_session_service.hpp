#pragma once

#include <atomic>
#include <cstdio>
#include <thread>

#include <opencv2/opencv.hpp>

#include "adapters/stream/udp_image_sender.hpp"
#include "core/application/calib_storage_helpers.hpp"
#include "common/tlv/tlv_protocol.hpp"
#include "core/application/live_pose_state.hpp"
#include "core/application/runtime_session_common.hpp"
#include "core/application/sensor_runtime_helpers.hpp"

namespace smartdrone::core::application {

inline bool RunCalibSession(const UnifiedConfig& cfg,
                            std::atomic<bool>& stop,
                            LivePoseState& livePose,
                            std::atomic<bool>& runningFlag)
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
    FILE* fCam0 = std::fopen((cam0Dir / "data.csv").string().c_str(), "w");
    FILE* fCam1 = std::fopen((cam1Dir / "data.csv").string().c_str(), "w");
    FILE* fImu = std::fopen((root / "imu.csv").string().c_str(), "w");
    if (!fCam0 || !fCam1 || !fImu) return false;
    std::cerr << "[calib] out=" << outRoot << "\n";
    SetupFileBuffer(fCam0, 1 << 20);
    SetupFileBuffer(fCam1, 1 << 20);
    SetupFileBuffer(fImu, 4 << 20);
    std::fprintf(fCam0, "#timestamp [ns],filename\n");
    std::fprintf(fCam1, "#timestamp [ns],filename\n");
    std::fprintf(fImu, "#timestamp [ns],wX [rad/s],wY [rad/s],wZ [rad/s],aX [m/s^2],aY [m/s^2],aZ [m/s^2]\n");
    UdpImageSender udp;
    if (a.udpEnable && a.sendImage) udp.Open(a.udpIp, a.udpPort, a.udpJpegQ, a.udpPayload, a.udpQueue);
    std::atomic<bool> imuOk{false};
    std::thread imuThread = StartCalibImuWriterThread(a, fImu, imuOk, stop, runningFlag);
    LibcameraStereoOV9281_TsPair cam;
    if (!OpenCamera(cam, a)) {
        stop.store(true);
        if (imuThread.joinable()) imuThread.join();
        std::fclose(fCam0);
        std::fclose(fCam1);
        std::fclose(fImu);
        return false;
    }
    int saved = 0;
    int64_t lastPairNs = 0;
    const int64_t maxSaveDtNs = static_cast<int64_t>(std::max(a.pairMs, 1)) * 1000000LL;
    bool sessionOk = true;
    while (runningFlag.load() && !stop.load()) {
        if (cfg.calib.maxFrames > 0 && saved >= cfg.calib.maxFrames) break;
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
        const int64_t absDtLr = Abs64(static_cast<int64_t>(L.tsNs) - static_cast<int64_t>(R.tsNs));
        if (absDtLr > maxSaveDtNs) {
            static int droppedWide = 0;
            ++droppedWide;
            if ((droppedWide % 10) == 1) {
                std::cerr << "[calib-drop] dt_lr_us=" << (absDtLr / 1000.0)
                          << " exceeds max_save_dt_us=" << (maxSaveDtNs / 1000.0)
                          << " dropped=" << droppedWide
                          << "\n";
            }
            continue;
        }
        int64_t pairNs = static_cast<int64_t>((L.tsNs + R.tsNs) / 2);
        if (lastPairNs != 0 && pairNs <= lastPairNs) pairNs = lastPairNs + 1;
        lastPairNs = pairNs;
        const std::string name = TsToName(pairNs);
        const fs::path fnL = cam0Dir / name;
        const fs::path fnR = cam1Dir / name;
        if (L.gray.empty() || R.gray.empty()) {
            std::cerr << "[calib-write] empty image"
                      << " seqL=" << L.seq
                      << " seqR=" << R.seq
                      << " rowsL=" << L.gray.rows
                      << " colsL=" << L.gray.cols
                      << " rowsR=" << R.gray.rows
                      << " colsR=" << R.gray.cols
                      << "\n";
            continue;
        }
        const bool okL = cv::imwrite(fnL.string(), L.gray);
        const bool okR = cv::imwrite(fnR.string(), R.gray);
        if (!okL || !okR) {
            std::cerr << "[calib-write] imwrite failed"
                      << " okL=" << (okL ? "true" : "false")
                      << " okR=" << (okR ? "true" : "false")
                      << " pathL=" << fnL.string()
                      << " pathR=" << fnR.string()
                      << " typeL=" << L.gray.type()
                      << " typeR=" << R.gray.type()
                      << " rowsL=" << L.gray.rows
                      << " colsL=" << L.gray.cols
                      << " rowsR=" << R.gray.rows
                      << " colsR=" << R.gray.cols
                      << "\n";
            continue;
        }
        std::fprintf(fCam0, "%lld,%s\n", static_cast<long long>(pairNs), name.c_str());
        std::fprintf(fCam1, "%lld,%s\n", static_cast<long long>(pairNs), name.c_str());
        if (a.udpEnable && a.sendImage) {
            udp.Enqueue(0, L.seq, pairNs * 1e-9, L.gray, {}, true, false);
            udp.Enqueue(1, R.seq, pairNs * 1e-9, R.gray, {}, true, false);
        }
        if ((saved % 30) == 0) {
            std::cerr << "[calib-save] saved=" << (saved + 1)
                      << " pathL=" << fnL.string()
                      << " pathR=" << fnR.string()
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
    if (imuThread.joinable()) imuThread.join();
    std::cerr << "[session] calib imu joined\n";
    if (a.udpEnable && a.sendImage) {
        udp.Close();
        std::cerr << "[session] calib udp closed\n";
    }
    std::fflush(fCam0);
    std::fflush(fCam1);
    std::fflush(fImu);
    std::fclose(fCam0);
    std::fclose(fCam1);
    std::fclose(fImu);
    std::cerr << "[calib] out=" << outRoot << " saved=" << saved
              << " imuOk=" << (imuOk.load() ? "true" : "false") << "\n";
    livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
    std::cerr << "[session] calib exit\n";
    return sessionOk;
}

}  // namespace smartdrone::core::application
