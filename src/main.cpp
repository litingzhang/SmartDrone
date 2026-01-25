#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>

#include <libcamera/camera_manager.h>
#include <opencv2/core.hpp>

#include "ImuTypes.h"
#include "System.h"

#include "FrameSyncQueue.h"
#include "FrameTypes.h"
#include "Icm20948Poller.h"
#include "LibcameraStereo.h"
#include "TimeUtils.h"

static std::atomic<bool> g_run(true);

class CvUtils {
  public:
    static cv::Mat YPlaneToGrayMat(const FrameItem &item)
    {
        cv::Mat gray(static_cast<int>(item.m_height), static_cast<int>(item.m_width), CV_8UC1);

        const uint8_t *y = item.m_data->data();

        for (unsigned int r = 0; r < item.m_height; r++) {
            std::memcpy(gray.ptr(r), y + r * item.m_stride, item.m_width);
        }

        return gray;
    }
};

static void OnSigint(int) { g_run.store(false); }

class CamOffsetLearner {
  public:
    CamOffsetLearner() : m_haveOffset(false), m_count(0), m_offsetNs(0.0) {}

    bool UpdateAndCheckReady(uint64_t leftTsNs, uint64_t rightTsNs)
    {
        if (m_haveOffset)
            return true;

        double cur = static_cast<double>(rightTsNs) - static_cast<double>(leftTsNs);
        m_offsetNs = (m_offsetNs * m_count + cur) / static_cast<double>(m_count + 1);
        m_count++;

        if (m_count % 10 == 0) {
            std::cerr << "[CAM] learning offset: " << m_offsetNs * 1e-6 << " ms\n";
        }

        if (m_count >= 60) {
            m_haveOffset = true;
            std::cerr << "[CAM] final offset: " << m_offsetNs * 1e-6 << " ms\n";
        }
        return m_haveOffset;
    }

    bool HaveOffset() const { return m_haveOffset; }

    double GetOffsetNs() const { return m_offsetNs; }

  private:
    bool m_haveOffset;
    int m_count;
    double m_offsetNs; // cam1 - cam0
};

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    const std::string vocabPath = "./ORBvoc.txt";
    const std::string settingsPath = "./calib.yaml";

    std::signal(SIGINT, OnSigint);

    ORB_SLAM3::System slam(vocabPath, settingsPath, ORB_SLAM3::System::IMU_STEREO, false);

    Icm20948Poller::Options imuOpt;
    imuOpt.m_i2cDev = "/dev/i2c-1";
    imuOpt.m_addr = 0x68;
    imuOpt.m_rateHz = 200.0;
    imuOpt.m_calibSeconds = 2.0;
    imuOpt.m_ringSeconds = 5;
    imuOpt.m_realtimeFifo = true;
    imuOpt.m_fifoPrio = 80;

    Icm20948Poller imu(imuOpt);
    imu.Start();
    std::cerr << "[IMU] calibrating... keep still for " << imuOpt.m_calibSeconds << "s\n";

    libcamera::CameraManager cameraManager;
    if (cameraManager.start()) {
        std::cerr << "CameraManager start failed\n";
        return 1;
    }

    auto cameras = cameraManager.cameras();
    if (cameras.size() < 2) {
        std::cerr << "Need 2 cameras; found " << cameras.size() << "\n";
        cameraManager.stop();
        return 1;
    }

    const unsigned int width = 640;
    const unsigned int height = 480;

    FrameSyncQueue frameQueue;
    LibcameraStereo stereo(frameQueue);

    if (!stereo.Start(cameras[0], cameras[1], width, height)) {
        stereo.Stop();
        cameraManager.stop();
        return 1;
    }

    const uint64_t maxDtNs = 30ULL * 1000ULL * 1000ULL; // 30ms

    CamOffsetLearner offsetLearner;

    double lastT = -1.0;

    while (g_run.load()) {
        FrameItem left;
        FrameItem right;

        if (!frameQueue.PopSyncedPair(left, right, maxDtNs)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if (!offsetLearner.HaveOffset()) {
            if (offsetLearner.UpdateAndCheckReady(left.m_tsNs, right.m_tsNs)) {
                lastT = TimeUtils::NsToSec(left.m_tsNs);
            }
            continue;
        }

        cv::Mat imL = CvUtils::YPlaneToGrayMat(left);
        cv::Mat imR = CvUtils::YPlaneToGrayMat(right);

        double meanL = cv::mean(imL)[0];
        double meanR = cv::mean(imR)[0];
        std::cout << "[IMG] meanL=" << meanL << " meanR=" << meanR << " w=" << imL.cols
                  << " h=" << imL.rows << " stride=" << left.m_stride << "\n";

        const double t = TimeUtils::NsToSec(left.m_tsNs);

        std::vector<ORB_SLAM3::IMU::Point> vImu;
        if (lastT > 0.0)
            vImu = imu.GetBetween(lastT, t);

        if (lastT > 0.0 && vImu.size() < 2) {
            lastT = t;
            continue;
        }

        Sophus::SE3f tcw = slam.TrackStereo(imL, imR, t, vImu);

        int st = slam.GetTrackingState();
        auto mps = slam.GetTrackedMapPoints();
        auto kps = slam.GetTrackedKeyPointsUn();

        int nMps = 0;
        for (auto *p : mps) {
            if (p)
                nMps++;
        }

        ORB_SLAM3::IMU::Point point = vImu.size() > 0 ? vImu[0] : ORB_SLAM3::IMU::Point(0, 0, 0, 0, 0, 0, 0);
        std::cout << "[IMU] t=" << vImu[0].t << " acc=(" << point.a.x() << "," << point.a.y() << "," << point.a.z()
                  << ") gyro=(" << point.w.x() << "," << point.w.y() << "," << point.w.z() << ")\n";
        std::cout << "[TRACK] t=" << t << " state=" << st << " kp=" << kps.size() << " mp=" << nMps
                  << " imu=" << vImu.size() << "\n";

        Eigen::Matrix4f tcwM = tcw.matrix();
        bool valid = tcwM.allFinite();

        if (valid) {
            Sophus::SE3f twc = tcw.inverse();
            Eigen::Vector3f pW = twc.translation();
            Eigen::Quaternionf qW(twc.so3().unit_quaternion());

            std::cout.setf(std::ios::fixed);
            std::cout.precision(6);

            std::cout << "[POSE] t=" << t << " p=(" << pW.x() << "," << pW.y() << "," << pW.z()
                      << ")" << " q=(" << qW.x() << "," << qW.y() << "," << qW.z() << "," << qW.w()
                      << ")" << "\n";
        } else {
            std::cout << "[POSE] t=" << t << " INVALID\n";
        }

        lastT = t;
    }

    imu.Stop();
    stereo.Stop();
    cameraManager.stop();

    slam.Shutdown();
    return 0;
}
