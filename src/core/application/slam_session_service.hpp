#pragma once

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "System.h"
#include "adapters/camera/libcamera_stereo_camera.hpp"
#include "adapters/imu/icm42688_imu_provider.hpp"
#include "adapters/slam/orbslam3_engine.hpp"
#include "adapters/stream/udp_image_sender.hpp"
#include "adapters/telemetry/mavlink_pose_publisher.hpp"
#include "adapters/telemetry/px4_mavlink_gateway.hpp"
#include "common/time_utils.hpp"
#include "common/tlv/tlv_protocol.hpp"
#include "core/application/sensor_runtime_helpers.hpp"
#include "core/application/live_pose_state.hpp"
#include "core/application/perception_pipeline.hpp"
#include "core/application/pose_postprocessor.hpp"
#include "core/application/runtime_session_common.hpp"
#include "common/logger.hpp"

namespace smartdrone::core::application {

inline double DurationMs(const std::chrono::steady_clock::time_point& start,
                         const std::chrono::steady_clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

inline void ComputeImageStats(const cv::Mat& gray, double& meanOut, double& stddevOut)
{
    meanOut = 0.0;
    stddevOut = 0.0;
    if (gray.empty()) {
        return;
    }
    cv::Scalar mean{};
    cv::Scalar stddev{};
    cv::meanStdDev(gray, mean, stddev);
    meanOut = mean[0];
    stddevOut = stddev[0];
}

inline double ComputeSharpnessLaplacianVar(const cv::Mat& gray)
{
    if (gray.empty()) {
        return 0.0;
    }
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean{};
    cv::Scalar stddev{};
    cv::meanStdDev(laplacian, mean, stddev);
    return stddev[0] * stddev[0];
}

inline bool IsTrackingPoseUsable(int trackingState)
{
    return trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::OK_KLT;
}

inline std::vector<ORB_SLAM3::IMU::Point> ToOrbImuPoints(const std::vector<smartdrone::core::ports::ImuReading>& readings)
{
    std::vector<ORB_SLAM3::IMU::Point> out;
    out.reserve(readings.size());
    for (const auto& reading : readings) {
        out.emplace_back(
            cv::Point3f(reading.ax, reading.ay, reading.az),
            cv::Point3f(reading.gx, reading.gy, reading.gz),
            static_cast<double>(reading.timestampNs) * 1e-9);
    }
    return out;
}

inline std::vector<smartdrone::core::ports::ImuReading> ToImuReadings(const std::vector<ORB_SLAM3::IMU::Point>& points)
{
    std::vector<smartdrone::core::ports::ImuReading> out;
    out.reserve(points.size());
    for (const auto& point : points) {
        smartdrone::core::ports::ImuReading reading{};
        reading.timestampNs = static_cast<int64_t>(point.t * 1e9);
        reading.ax = point.a.x();
        reading.ay = point.a.y();
        reading.az = point.a.z();
        reading.gx = point.w.x();
        reading.gy = point.w.y();
        reading.gz = point.w.z();
        out.push_back(reading);
    }
    return out;
}

inline bool RunSlamSession(const UnifiedConfig& cfg,
                           LiveRuntimeTuning& tuning,
                           Px4MavlinkGateway& mav,
                           std::atomic<bool>& stop,
                           LivePoseState& livePose,
                           std::atomic<bool>& runningFlag)
{
    const MainRuntimeAliases a = BuildRuntimeAliases(cfg.app);
    PrintStartupConfig(cfg.app, a, ControllerMode::Slam);
    livePose.SetRuntimeMode(RUNTIME_MODE_SLAM);
    Logger::Init("./stereo_vslam.log", 32 * 1024 * 1024, Logger::INFO, true);
    auto slamSystem = std::make_unique<ORB_SLAM3::System>(
        cfg.app.vocab,
        cfg.app.settings,
        a.sensorMode == SensorMode::StereoImu ? ORB_SLAM3::System::IMU_STEREO
                                              : ORB_SLAM3::System::STEREO,
        false);
    smartdrone::adapters::slam::OrbSlam3Engine slamEngine(std::move(slamSystem), a.sensorMode == SensorMode::StereoImu);
    if (!slamEngine.Start()) {
        stop.store(true);
        return false;
    }
    const StereoBodyExtrinsics stereoBodyExtrinsics =
        (a.sensorMode == SensorMode::Stereo) ? LoadStereoBodyExtrinsics(cfg.app.settings) : StereoBodyExtrinsics{};
    UdpImageSender udp;
    if (a.udpEnable && (a.sendImage || a.sendFeature)) {
        udp.Open(a.udpIp, a.udpPort, a.udpJpegQ, a.udpPayload, a.udpQueue);
    }
    ImuThreadState imuState;
    std::thread imuThread;
    const bool useImu = (a.sensorMode == SensorMode::StereoImu);
    if (useImu) imuThread = StartImuThread(a, imuState, stop, runningFlag);
    LibcameraStereoOV9281_TsPair cam;
    if (!OpenCamera(cam, a)) {
        stop.store(true);
        if (imuThread.joinable()) imuThread.join();
        slamEngine.Stop();
        return false;
    }
    const int64_t imuDtNs = 1000000000LL / std::max(1, a.imuHz);
    const int64_t slackBeforeNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    const int64_t slackAfterNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    smartdrone::adapters::camera::LibcameraStereoCamera cameraProvider(cam);
    smartdrone::adapters::imu::Icm42688ImuProvider imuProvider(
        imuState.imuBuffer,
        smartdrone::adapters::imu::Icm42688ImuProviderConfig{slackBeforeNs, slackAfterNs});
    smartdrone::adapters::telemetry::MavlinkPosePublisher posePublisher(mav);
    PerceptionPipeline perceptionPipeline(PerceptionPipelineConfig{a.fps, true});
    int64_t lastFrameNs = 0;
    int lastLoggedSlamInputFps = -1;
    const uint64_t imuWarmupSamples = static_cast<uint64_t>(std::max(20, a.imuHz / 2));
    int64_t lastPointCloudUpdateNs = 0;
    Sophus::SE3f stereoReferencePose{Sophus::SE3f()};
    bool stereoReferencePoseSet = false;
    unsigned long lastRawMapId = PosePostprocessor::ContinuityMapper::kInvalidMapId;
    Sophus::SE3f lastValidTwcRaw{Sophus::SE3f()};
    bool haveLastValidTwcRaw = false;
    PosePostprocessor posePostprocessor{};
    bool sessionOk = true;
    uint64_t frameIndex = 0;
    int64_t lastPublishedFrameNs = 0;
    uint64_t rateLimitedDrops = 0;
    while (runningFlag.load() && !stop.load()) {
        const auto frameStartTp = std::chrono::steady_clock::now();
        if (useImu) {
            const uint64_t imuCnt = imuState.imuCnt.load(std::memory_order_relaxed);
            const bool imuReady = imuState.imuOk.load(std::memory_order_relaxed) && imuCnt >= imuWarmupSamples;
            if (!imuReady) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
        }
        StereoBatch stereoBatch{};
        const int slamInputFps = perceptionPipeline.ClampTargetFps(
            tuning.slamInputFps.load(std::memory_order_relaxed));
        if (slamInputFps != lastLoggedSlamInputFps) {
            std::cerr << "[slam] target_input_fps=" << slamInputFps
                      << " camera_fps=" << a.fps
                      << " frame_drop=" << (slamInputFps < a.fps ? "enabled" : "disabled") << "\n";
            lastLoggedSlamInputFps = slamInputFps;
        }
        const auto acquireStartTp = std::chrono::steady_clock::now();
        const StereoAcquireStatus acquireStatus =
            perceptionPipeline.AcquireNextStereoBatch(cameraProvider, slamInputFps, 1000, stereoBatch);
        const auto acquireEndTp = std::chrono::steady_clock::now();
        if (acquireStatus == StereoAcquireStatus::Timeout) {
            continue;
        }
        if (acquireStatus == StereoAcquireStatus::CameraUnhealthy) {
            std::cerr << "[slam] camera pipeline unhealthy, aborting session\n";
            sessionOk = false;
            break;
        }
        if (acquireStatus == StereoAcquireStatus::DroppedByRateLimiter) {
            ++rateLimitedDrops;
            continue;
        }
        auto& L = stereoBatch.stereo.left;
        auto& R = stereoBatch.stereo.right;
        const uint32_t rawSeqL = cameraProvider.LastRawSeqL();
        const uint32_t rawSeqR = cameraProvider.LastRawSeqR();
        const int64_t pairDtMs = cameraProvider.LastPairDtMs();
        const double rejectDtMs = static_cast<double>(cameraProvider.LastRejectDtUs()) / 1000.0;
        const uint64_t rawCountL = cameraProvider.RawCountL();
        const uint64_t rawCountR = cameraProvider.RawCountR();
        const uint64_t dropUnpairedL = cameraProvider.DroppedUnpairedL();
        const uint64_t dropUnpairedR = cameraProvider.DroppedUnpairedR();
        const size_t pendingL = cameraProvider.PendingL();
        const size_t pendingR = cameraProvider.PendingR();
        const double pairTolMs = static_cast<double>(cameraProvider.PairTolNs()) * 1e-6;
        const int64_t frameNs = stereoBatch.frameTimestampNs;
        const double frameTime = static_cast<double>(frameNs) * 1e-9;
        // Publish EV/VIO with the stereo frame timestamp for both stereo and stereo-imu modes.
        // PX4 EV delay should compensate processing latency relative to this capture time.
        const uint64_t publishTimestampUs = static_cast<uint64_t>(frameNs / 1000LL);
        const double frameGapMs =
            (lastPublishedFrameNs != 0) ? static_cast<double>(frameNs - lastPublishedFrameNs) * 1e-6 : 0.0;
        const double monoStepMs = static_cast<double>(stereoBatch.monotonicFrameStepNs) * 1e-6;
        double meanL = 0.0, stdL = 0.0, meanR = 0.0, stdR = 0.0;
        ComputeImageStats(L.gray, meanL, stdL);
        ComputeImageStats(R.gray, meanR, stdR);
        const double sharpL = ComputeSharpnessLaplacianVar(L.gray);
        const double sharpR = ComputeSharpnessLaplacianVar(R.gray);
        const auto imuStartTp = std::chrono::steady_clock::now();
        std::vector<ORB_SLAM3::IMU::Point> vImu;
        std::vector<smartdrone::core::ports::ImuReading> imuReadings;
        if (useImu && lastFrameNs != 0) {
            imuReadings = imuProvider.PopWindow(lastFrameNs, frameNs);
            vImu = ToOrbImuPoints(imuReadings);
            ImuWindowValidation imuWindow{};
            const double prevFrameTime = static_cast<double>(lastFrameNs) * 1e-9;
            const double expectedImuDtSec = 1.0 / std::max(1, a.imuHz);
            const bool imuWindowOk = SanitizeImuWindow(vImu, prevFrameTime, frameTime, expectedImuDtSec, imuWindow);
            if (!imuWindowOk) {
                continue;
            }
            if (vImu.empty() && !a.allowEmptyImu) continue;
            imuReadings = ToImuReadings(vImu);
        }
        const auto imuEndTp = std::chrono::steady_clock::now();
        lastFrameNs = frameNs;
        smartdrone::core::ports::SlamInputBatch slamInput{};
        slamInput.stereo = stereoBatch.stereo;
        slamInput.frameTimeSec = frameTime;
        slamInput.imu = std::move(imuReadings);
        const bool updatePointCloud =
            a.sendMap && (frameNs - lastPointCloudUpdateNs) >= kPointCloudUpdateIntervalNs;
        const auto slamStartTp = std::chrono::steady_clock::now();
        const smartdrone::core::ports::SlamOutput slamOutput = slamEngine.Process(slamInput, updatePointCloud);
        const auto slamEndTp = std::chrono::steady_clock::now();
        const auto cloudStartTp = std::chrono::steady_clock::now();
        size_t pointCount = 0;
        if (a.udpEnable && (a.sendImage || a.sendFeature || a.sendMap)) {
            if (updatePointCloud) {
                livePose.UpdatePointCloud(slamOutput.pointCloudXyz);
                lastPointCloudUpdateNs = frameNs;
            }
            pointCount = slamOutput.pointCloudXyz.size() / 3;
        }
        const auto cloudEndTp = std::chrono::steady_clock::now();
        const auto udpStartTp = std::chrono::steady_clock::now();
        if (a.udpEnable && (a.sendImage || a.sendFeature || a.sendMap)) {
            udp.Enqueue(0, L.sequence, frameTime, L.gray, slamOutput.leftFeatures, a.sendImage, a.sendFeature);
            udp.Enqueue(1, R.sequence, frameTime, R.gray, slamOutput.rightFeatures, a.sendImage, a.sendFeature);
        }
        const auto udpEndTp = std::chrono::steady_clock::now();
        const int state = slamOutput.trackingState;
        const bool trackingUsable = IsTrackingPoseUsable(state);
        const unsigned long mapId = slamOutput.mapId;
        const bool mapIdChanged = mapId != PosePostprocessor::ContinuityMapper::kInvalidMapId && mapId != lastRawMapId;
        if (mapIdChanged) {
            lastRawMapId = mapId;
            if (!useImu) {
                stereoReferencePoseSet = false;
            }
        }
        Sophus::SE3f twcRaw = haveLastValidTwcRaw ? lastValidTwcRaw : Sophus::SE3f();
        if (slamOutput.poseValid) {
            const Eigen::Quaternionf rawQ(
                slamOutput.pose.qw,
                slamOutput.pose.qx,
                slamOutput.pose.qy,
                slamOutput.pose.qz);
            twcRaw = Sophus::SE3f(Sophus::SO3f(rawQ), Eigen::Vector3f(
                slamOutput.pose.x,
                slamOutput.pose.y,
                slamOutput.pose.z));
            lastValidTwcRaw = twcRaw;
            haveLastValidTwcRaw = true;
        }
        const auto postStartTp = std::chrono::steady_clock::now();
        const auto poseResult = posePostprocessor.ProcessPose(
            twcRaw,
            useImu,
            trackingUsable,
            state,
            mapId,
            stereoBodyExtrinsics.loaded,
            stereoBodyExtrinsics.Tbc,
            stereoReferencePoseSet,
            stereoReferencePose,
            frameNs,
            mav);
        const auto postEndTp = std::chrono::steady_clock::now();
        const auto livePoseStartTp = std::chrono::steady_clock::now();
        livePose.UpdatePose(RUNTIME_MODE_SLAM, static_cast<uint8_t>(state),
                            poseResult.resetCounter, poseResult.resetMapCount,
                            poseResult.alignedPose,
                            poseResult.quality == smartdrone::core::ports::PoseQuality::Good ? OdomQualityMode::GOOD :
                            poseResult.quality == smartdrone::core::ports::PoseQuality::Weak ? OdomQualityMode::WEAK :
                                                                                               OdomQualityMode::LOST);
        const auto livePoseEndTp = std::chrono::steady_clock::now();
        const auto publishStartTp = std::chrono::steady_clock::now();
        posePublisher.PublishPose(
            publishTimestampUs,
            poseResult.poseEstimate,
            poseResult.velocityEstimate,
            poseResult.resetCounter,
            poseResult.resetMapCount,
            state,
            poseResult.quality);
        const auto publishEndTp = std::chrono::steady_clock::now();
        ++frameIndex;
        lastPublishedFrameNs = frameNs;
        char dfxLine[512];
        std::snprintf(
            dfxLine,
            sizeof(dfxLine),
            "[slam_dfx] frame=%llu seqL=%u seqR=%u state=%d pose_valid=%d quality=%d "
            "imu_samples=%zu featL=%zu featR=%zu points=%zu "
            "pair rawSeqL=%u rawSeqR=%u rawCountL=%llu rawCountR=%llu pair_dt=%.3f reject_dt=%.3f tol=%.3f pendL=%zu pendR=%zu dropL=%llu dropR=%llu rate_drop=%llu "
            "img meanL=%.2f stdL=%.2f sharpL=%.2f meanR=%.2f stdR=%.2f sharpR=%.2f "
            "timing_ms frame_gap=%.3f mono_step=%.3f acquire=%.3f imu=%.3f slam=%.3f cloud=%.3f udp=%.3f post=%.3f live=%.3f publish=%.3f total=%.3f",
            static_cast<unsigned long long>(frameIndex),
            static_cast<unsigned>(L.sequence),
            static_cast<unsigned>(R.sequence),
            state,
            poseResult.poseEstimate.valid ? 1 : 0,
            static_cast<int>(poseResult.quality),
            slamInput.imu.size(),
            slamOutput.leftFeatures.size(),
            slamOutput.rightFeatures.size(),
            pointCount,
            static_cast<unsigned>(rawSeqL),
            static_cast<unsigned>(rawSeqR),
            static_cast<unsigned long long>(rawCountL),
            static_cast<unsigned long long>(rawCountR),
            static_cast<double>(pairDtMs),
            rejectDtMs,
            pairTolMs,
            pendingL,
            pendingR,
            static_cast<unsigned long long>(dropUnpairedL),
            static_cast<unsigned long long>(dropUnpairedR),
            static_cast<unsigned long long>(rateLimitedDrops),
            meanL,
            stdL,
            sharpL,
            meanR,
            stdR,
            sharpR,
            frameGapMs,
            monoStepMs,
            DurationMs(acquireStartTp, acquireEndTp),
            DurationMs(imuStartTp, imuEndTp),
            DurationMs(slamStartTp, slamEndTp),
            DurationMs(cloudStartTp, cloudEndTp),
            DurationMs(udpStartTp, udpEndTp),
            DurationMs(postStartTp, postEndTp),
            DurationMs(livePoseStartTp, livePoseEndTp),
            DurationMs(publishStartTp, publishEndTp),
            DurationMs(frameStartTp, publishEndTp));
        LOGI("%s", dfxLine);
        std::cerr << dfxLine << "\n";
    }
    cam.Close();
    std::cerr << "[session] slam camera closed\n";
    if (imuThread.joinable()) imuThread.join();
    std::cerr << "[session] slam imu joined\n";
    if (a.udpEnable && (a.sendImage || a.sendFeature)) {
        udp.Close();
        std::cerr << "[session] slam udp closed\n";
    }
    mav.StopSetpointStream();
    std::cerr << "[session] slam setpoint stopped\n";
    slamEngine.Stop();
    std::cerr << "[session] slam shutdown complete\n";
    livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
    std::cerr << "[session] slam exit\n";
    return sessionOk;
}

}  // namespace smartdrone::core::application
