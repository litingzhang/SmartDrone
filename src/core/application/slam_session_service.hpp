#pragma once

#include <memory>
#include <thread>
#include <vector>

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
    while (runningFlag.load() && !stop.load()) {
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
        const StereoAcquireStatus acquireStatus =
            perceptionPipeline.AcquireNextStereoBatch(cameraProvider, slamInputFps, 1000, stereoBatch);
        if (acquireStatus == StereoAcquireStatus::Timeout) {
            continue;
        }
        if (acquireStatus == StereoAcquireStatus::CameraUnhealthy) {
            std::cerr << "[slam] camera pipeline unhealthy, aborting session\n";
            sessionOk = false;
            break;
        }
        if (acquireStatus == StereoAcquireStatus::DroppedByRateLimiter) {
            continue;
        }
        auto& L = stereoBatch.stereo.left;
        auto& R = stereoBatch.stereo.right;
        const int64_t frameNs = stereoBatch.frameTimestampNs;
        const double frameTime = static_cast<double>(frameNs) * 1e-9;
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
        lastFrameNs = frameNs;
        smartdrone::core::ports::SlamInputBatch slamInput{};
        slamInput.stereo = stereoBatch.stereo;
        slamInput.frameTimeSec = frameTime;
        slamInput.imu = std::move(imuReadings);
        const bool updatePointCloud =
            a.sendMap && (frameNs - lastPointCloudUpdateNs) >= kPointCloudUpdateIntervalNs;
        const smartdrone::core::ports::SlamOutput slamOutput = slamEngine.Process(slamInput, updatePointCloud);
        if (a.udpEnable && (a.sendImage || a.sendFeature || a.sendMap)) {
            if (updatePointCloud) {
                livePose.UpdatePointCloud(slamOutput.pointCloudXyz);
                lastPointCloudUpdateNs = frameNs;
            }
            udp.Enqueue(0, L.sequence, frameTime, L.gray, slamOutput.leftFeatures, a.sendImage, a.sendFeature);
            udp.Enqueue(1, R.sequence, frameTime, R.gray, slamOutput.rightFeatures, a.sendImage, a.sendFeature);
        }
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
        livePose.UpdatePose(RUNTIME_MODE_SLAM, static_cast<uint8_t>(state),
                            poseResult.resetCounter, poseResult.resetMapCount,
                            poseResult.alignedPose,
                            poseResult.quality == smartdrone::core::ports::PoseQuality::Good ? OdomQualityMode::GOOD :
                            poseResult.quality == smartdrone::core::ports::PoseQuality::Weak ? OdomQualityMode::WEAK :
                                                                                               OdomQualityMode::LOST);
        posePublisher.PublishPose(
            MonoTimeUs(),
            poseResult.poseEstimate,
            poseResult.velocityEstimate,
            poseResult.resetCounter,
            poseResult.resetMapCount,
            state,
            poseResult.quality);
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
