#include "core/application/session/runtime_session_common.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "core/application/session/sensor_runtime_helpers.h"

namespace smartdrone::core::application {

int ClampSlamInputFps(int requestedFps, int cameraFps)
{
    if (cameraFps <= 0) {
        return std::max(1, requestedFps);
    }
    if (requestedFps <= 0) {
        return cameraFps;
    }
    return std::clamp(requestedFps, 1, cameraFps);
}

bool IsFiniteImuPoint(const ORB_SLAM3::IMU::Point &p)
{
    return std::isfinite(p.t) && std::isfinite(p.a.x()) && std::isfinite(p.a.y()) && std::isfinite(p.a.z()) &&
           std::isfinite(p.w.x()) && std::isfinite(p.w.y()) && std::isfinite(p.w.z());
}

bool SanitizeImuWindow(std::vector<ORB_SLAM3::IMU::Point> &vImu, double prevFrameTime, double frameTime,
                       double expectedImuDtSec, ImuWindowValidation &stats)
{
    constexpr float kMaxAccelNormMps2 = 200.0f;
    constexpr float kMaxGyroNormRadps = 40.0f;
    constexpr double kMinSampleDtSec = 1e-6;

    stats = ImuWindowValidation{};
    stats.inputCount = vImu.size();

    std::vector<ORB_SLAM3::IMU::Point> filtered;
    filtered.reserve(vImu.size());

    double lastT = 0.0;
    bool haveLastT = false;
    for (const auto &sample : vImu) {
        if (!IsFiniteImuPoint(sample)) {
            ++stats.droppedNonFinite;
            continue;
        }

        const float accelNorm = sample.a.norm();
        const float gyroNorm = sample.w.norm();
        if (!(accelNorm <= kMaxAccelNormMps2) || !(gyroNorm <= kMaxGyroNormRadps)) {
            ++stats.droppedOutOfRange;
            continue;
        }

        if (haveLastT) {
            const double dt = sample.t - lastT;
            if (!(dt > kMinSampleDtSec)) {
                ++stats.droppedNonMonotonic;
                continue;
            }
            stats.largestGapSec = std::max(stats.largestGapSec, dt);
        }

        filtered.push_back(sample);
        lastT = sample.t;
        haveLastT = true;
    }

    vImu.swap(filtered);
    stats.outputCount = vImu.size();

    if (vImu.size() < 2) {
        stats.failureReason = "too_few_samples";
        return false;
    }

    stats.firstLeadSec = vImu.front().t - prevFrameTime;
    stats.tailLagSec = frameTime - vImu.back().t;

    const double boundarySlackSec = std::max(6.0 * expectedImuDtSec, 0.010);
    const double maxGapSec = std::max(12.0 * expectedImuDtSec, 0.030);

    if (stats.firstLeadSec > boundarySlackSec) {
        stats.failureReason = "missing_leading_coverage";
        return false;
    }
    if (stats.tailLagSec > boundarySlackSec) {
        stats.failureReason = "missing_trailing_coverage";
        return false;
    }
    if (stats.largestGapSec > maxGapSec) {
        stats.failureReason = "large_internal_gap";
        return false;
    }

    return true;
}

std::optional<Sophus::SE3f> ReadSe3Node(const cv::FileNode &node)
{
    if (node.empty())
        return std::nullopt;

    cv::Mat mat;
    node >> mat;
    if (mat.empty() || mat.rows != 4 || mat.cols != 4)
        return std::nullopt;

    cv::Mat mat32f;
    mat.convertTo(mat32f, CV_32F);
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            T(r, c) = mat32f.at<float>(r, c);
        }
    }
    return Sophus::SE3f(T);
}

StereoBodyExtrinsics LoadStereoBodyExtrinsics(const std::string &settingsPath)
{
    StereoBodyExtrinsics extrinsics;

    cv::FileStorage fs(settingsPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[pose] warning: failed to open settings for stereo body extrinsics: " << settingsPath << "\n";
        return extrinsics;
    }

    const auto maybeTbc = ReadSe3Node(fs["T_b_c1"]);
    const auto maybeImuTbc = ReadSe3Node(fs["IMU.T_b_c1"]);
    if (maybeTbc.has_value()) {
        extrinsics.Tbc = *maybeTbc;
        extrinsics.loaded = true;
    } else if (maybeImuTbc.has_value()) {
        extrinsics.Tbc = *maybeImuTbc;
        extrinsics.loaded = true;
    } else {
        std::cerr << "[pose] info: no T_b_c1/IMU.T_b_c1 in settings, pure stereo pose stays in camera frame\n";
    }

    if (extrinsics.loaded) {
        const Eigen::Vector3f t = extrinsics.Tbc.translation();
        std::cerr << "[pose] pure stereo will publish body pose using T_b_c1"
                  << " tx=" << t.x() << " ty=" << t.y() << " tz=" << t.z() << "\n";
    }

    return extrinsics;
}

OrbExtractorSettings LoadOrbExtractorSettings(const std::string &settingsPath)
{
    OrbExtractorSettings settings;

    cv::FileStorage fs(settingsPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[slam] warning: failed to open settings for ORB extractor: " << settingsPath << "\n";
        return settings;
    }

    const cv::FileNode nFeaturesNode = fs["ORBextractor.nFeatures"];
    const cv::FileNode scaleFactorNode = fs["ORBextractor.scaleFactor"];
    const cv::FileNode nLevelsNode = fs["ORBextractor.nLevels"];
    const cv::FileNode iniThFastNode = fs["ORBextractor.iniThFAST"];
    const cv::FileNode minThFastNode = fs["ORBextractor.minThFAST"];
    if (nFeaturesNode.empty() || scaleFactorNode.empty() || nLevelsNode.empty() || iniThFastNode.empty() ||
        minThFastNode.empty()) {
        std::cerr << "[slam] warning: ORBextractor.* keys missing in settings: " << settingsPath << "\n";
        return settings;
    }

    settings.nFeatures = static_cast<int>(nFeaturesNode);
    settings.scaleFactor = static_cast<float>(scaleFactorNode);
    settings.nLevels = static_cast<int>(nLevelsNode);
    settings.iniThFAST = static_cast<int>(iniThFastNode);
    settings.minThFAST = static_cast<int>(minThFastNode);
    settings.loaded = settings.nFeatures > 0 && settings.scaleFactor > 0.0f && settings.nLevels > 0 &&
                      settings.iniThFAST > 0 && settings.minThFAST > 0;
    if (!settings.loaded) {
        std::cerr << "[slam] warning: invalid ORBextractor.* values in settings: " << settingsPath << "\n";
    }
    return settings;
}

MainRuntimeAliases BuildRuntimeAliases(const AppConfig &c)
{
    MainRuntimeAliases a{};
    a.sensorMode = c.sensorMode;
    a.slamOperationMode = c.runtime.slamOperationMode;
    a.featureFrontend = c.runtime.featureFrontend;
    a.width = c.camera.width;
    a.height = c.camera.height;
    a.fps = c.camera.fps;
    a.slamInputFps = ClampSlamInputFps(c.runtime.slamInputFps, c.camera.fps);
    a.leftCamIndex = c.camera.leftCamIndex;
    a.rightCamIndex = c.camera.rightCamIndex;
    a.uvcDeviceIndex = c.camera.uvcDeviceIndex;
    a.uvcEyeWidth = c.camera.uvcEyeWidth > 0 ? c.camera.uvcEyeWidth : c.camera.width;
    a.uvcEyeHeight = c.camera.uvcEyeHeight > 0 ? c.camera.uvcEyeHeight : c.camera.height;
    a.uvcPackedStereo = c.camera.uvcPackedStereo;
    a.uvcSwapEyes = c.camera.uvcSwapEyes;
    a.aeDisable = c.camera.aeDisable;
    a.exposureUs = c.camera.exposureUs;
    a.gain = c.camera.gain;
    a.requestY8 = c.camera.requestY8;
    a.r16Norm = c.camera.r16Norm;
    a.pairMs = c.camera.pairMs;
    a.keepMs = c.camera.keepMs;
    a.pairQueue = c.camera.pairQueue;
    a.udpEnable = c.udp.enable;
    a.udpIp = c.udp.ip;
    a.udpPort = c.udp.port;
    a.cmdPort = c.udp.cmdPort;
    a.udpJpegQ = c.udp.jpegQ;
    a.udpPayload = c.udp.payload;
    a.sendImage = c.udp.sendImage;
    a.sendFeature = c.udp.sendFeature;
    a.sendMap = c.udp.sendMap;
    a.udpQueue = c.udp.queue;
    a.xfeatInputMaxWidth = c.runtime.xfeatInputMaxWidth;
    a.xfeatInputMaxHeight = c.runtime.xfeatInputMaxHeight;
    a.lkXFeatSeeding = c.runtime.lkXFeatSeeding;
    a.lkPerFrameAcceleration = c.runtime.lkPerFrameAcceleration;
    a.lkLoopClosure = c.runtime.lkLoopClosure;
    a.lkLoopScale = c.runtime.lkLoopScale;
    a.lkLoopRelaxation = c.runtime.lkLoopRelaxation;
    a.spiDev = c.imu.spiDev;
    a.spiSpeed = c.imu.spiSpeed;
    a.spiMode = c.imu.spiMode;
    a.spiBits = c.imu.spiBits;
    a.gpiochip = c.imu.gpiochip;
    a.drdyLine = c.imu.drdyLine;
    a.imuHz = c.imu.imuHz;
    a.accelFsG = c.imu.accelFsG;
    a.gyroFsDps = c.imu.gyroFsDps;
    a.imuStartReg = c.imu.imuStartReg;
    a.allowEmptyImu = c.runtime.allowEmptyImu;
    a.debugRightOnlyFeatures = c.runtime.debugRightOnlyFeatures;
    a.slamLowLightEnhance = c.runtime.slamLowLightEnhance;
    a.jsonDiagnostics = c.runtime.jsonDiagnostics;
    a.rtImu = c.imu.rtImu;
    a.rtPrio = c.imu.rtPrio;
    return a;
}

void PrintStartupConfig(const AppConfig &app, const MainRuntimeAliases &a, ControllerMode mode)
{
    std::cerr << "mode=" << smartdrone::core::domain::ToString(mode) << "\n";
    std::cerr << "sensor_mode=" << ToSensorModeText(a.sensorMode) << "\n";
    const bool packedStereo = CompiledCameraProviderUsesPackedStereo();
    std::cerr << "cam " << a.width << "x" << a.height << " @" << a.fps
              << " aeDisable=" << (a.aeDisable ? "true" : "false") << " exp_us=" << a.exposureUs << " gain=" << a.gain
              << " pixelFormat=" << (packedStereo ? "YUYV_packed_stereo" : "R16") << "\n";
    std::cerr << "cam_select left_index=" << a.leftCamIndex << " right_index=" << a.rightCamIndex
              << " uvc_device_index=" << a.uvcDeviceIndex << "\n";
    std::cerr << "stereo_input eye=" << a.uvcEyeWidth << "x" << a.uvcEyeHeight
              << " packed_stereo=" << (a.uvcPackedStereo ? "Y" : "N")
              << " swap_eyes=" << (a.uvcSwapEyes ? "Y" : "N")
              << " pair_window_ms=" << a.pairMs << " keep_window_ms=" << a.keepMs << " frame_queue=" << a.pairQueue
              << "\n";
    if (packedStereo) {
        std::cerr << "stereo_input_note=single_uvc_frame_split_left_right_no_timestamp_pairing\n";
    }
    std::cerr << "slam_input_fps=" << a.slamInputFps << " camera_fps=" << a.fps
              << " frame_drop=" << (a.slamInputFps < a.fps ? "Y" : "N") << "\n";
    std::cerr << "slam_mode=" << smartdrone::core::domain::ToString(a.slamOperationMode) << "\n";
    std::cerr << "feature_frontend=" << ToFeatureFrontendText(a.featureFrontend) << "\n";
    std::cerr << "xfeat python=" << app.runtime.xfeatPython << " repo=" << app.runtime.xfeatRepo
              << " worker=" << app.runtime.xfeatWorkerScript << " device=" << app.runtime.xfeatDevice
              << " top_k=" << app.runtime.xfeatTopK
              << " max_points=" << app.runtime.xfeatMaxPoints
              << " input_max=" << app.runtime.xfeatInputMaxWidth << "x" << app.runtime.xfeatInputMaxHeight << "\n";
    std::cerr << "orb nFeatures=" << app.runtime.orbNFeatures << " scaleFactor=" << app.runtime.orbScaleFactor
              << " nLevels=" << app.runtime.orbNLevels << " iniThFAST=" << app.runtime.orbIniThFAST
              << " minThFAST=" << app.runtime.orbMinThFAST << "\n";
    std::cerr << "lk xfeat_seeding=" << (app.runtime.lkXFeatSeeding ? "Y" : "N")
              << " loop_closure=" << (app.runtime.lkLoopClosure ? "Y" : "N")
              << " scale=" << app.runtime.lkLoopScale << " relax=" << app.runtime.lkLoopRelaxation
              << " per_frame_accel=" << app.runtime.lkPerFrameAcceleration << "\n";
    std::cerr << "debug right_only_features=" << (a.debugRightOnlyFeatures ? "Y" : "N") << "\n";
    std::cerr << "slam lowlight_enhance=" << (a.slamLowLightEnhance ? "Y" : "N") << "\n";
    std::cerr << "diagnostics json=" << (a.jsonDiagnostics ? "Y" : "N") << "\n";
    std::cerr << "imuHz=" << a.imuHz << " udp=" << (a.udpEnable ? "Y" : "N") << " udpPort=" << a.udpPort
              << " cmdPort=" << a.cmdPort << "\n";
    std::cerr << "stream img=" << (a.sendImage ? "Y" : "N") << " feat=" << (a.sendFeature ? "Y" : "N")
              << " map=" << (a.sendMap ? "Y" : "N") << "\n";
    std::cerr << "vocab=" << app.vocab << "\nsettings=" << app.settings << "\n";
}

} // namespace smartdrone::core::application
