#include "core/application/runtime/runtime_aliases.h"

#include <algorithm>
#include <iostream>

namespace smartdrone::core::application {

bool CompiledCameraProviderUsesPackedStereo();

namespace {

void ApplyCameraAliases(MainRuntimeAliases &aliases, const AppConfig &config)
{
    aliases.width = config.camera.width;
    aliases.height = config.camera.height;
    aliases.fps = config.camera.fps;
    aliases.leftCamIndex = config.camera.leftCamIndex;
    aliases.rightCamIndex = config.camera.rightCamIndex;
    aliases.uvcDeviceIndex = config.camera.uvcDeviceIndex;
    aliases.uvcEyeWidth = config.camera.uvcEyeWidth > 0 ? config.camera.uvcEyeWidth : config.camera.width;
    aliases.uvcEyeHeight = config.camera.uvcEyeHeight > 0 ? config.camera.uvcEyeHeight : config.camera.height;
    aliases.uvcPackedStereo = config.camera.uvcPackedStereo;
    aliases.uvcSwapEyes = config.camera.uvcSwapEyes;
    aliases.aeDisable = config.camera.aeDisable;
    aliases.exposureUs = config.camera.exposureUs;
    aliases.gain = config.camera.gain;
    aliases.requestY8 = config.camera.requestY8;
    aliases.r16Norm = config.camera.r16Norm;
    aliases.pairMs = config.camera.pairMs;
    aliases.keepMs = config.camera.keepMs;
    aliases.pairQueue = config.camera.pairQueue;
}

void ApplyUdpAliases(MainRuntimeAliases &aliases, const AppConfig &config)
{
    aliases.udpEnable = config.udp.enable;
    aliases.udpIp = config.udp.ip;
    aliases.udpPort = config.udp.port;
    aliases.cmdPort = config.udp.cmdPort;
    aliases.udpJpegQ = config.udp.jpegQ;
    aliases.udpPayload = config.udp.payload;
    aliases.sendImage = config.udp.sendImage;
    aliases.sendFeature = config.udp.sendFeature;
    aliases.sendMap = config.udp.sendMap;
    aliases.udpQueue = config.udp.queue;
}

void ApplyRuntimeAliases(MainRuntimeAliases &aliases, const AppConfig &config)
{
    aliases.sensorMode = config.sensorMode;
    aliases.slamOperationMode = config.runtime.slamOperationMode;
    aliases.slamBackend = config.runtime.slamBackend;
    aliases.featureFrontend = config.runtime.featureFrontend;
    aliases.slamInputFps = ClampSlamInputFps(config.runtime.slamInputFps, config.camera.fps);
    aliases.visualFeatureInputMaxWidth = config.runtime.visualFeatureInputMaxWidth;
    aliases.visualFeatureInputMaxHeight = config.runtime.visualFeatureInputMaxHeight;
    aliases.lkPerFrameAcceleration = config.runtime.lkPerFrameAcceleration;
    aliases.orbAcceleration = config.runtime.orbAcceleration;
    aliases.lkLoopClosure = config.runtime.lkLoopClosure;
    aliases.lkLoopScale = config.runtime.lkLoopScale;
    aliases.lkLoopRelaxation = config.runtime.lkLoopRelaxation;
    aliases.allowEmptyImu = config.runtime.allowEmptyImu;
    aliases.debugRightOnlyFeatures = config.runtime.debugRightOnlyFeatures;
    aliases.slamLowLightEnhance = config.runtime.slamLowLightEnhance;
    aliases.jsonDiagnostics = config.runtime.jsonDiagnostics;
}

void ApplyImuAliases(MainRuntimeAliases &aliases, const AppConfig &config)
{
    aliases.spiDev = config.imu.spiDev;
    aliases.spiSpeed = config.imu.spiSpeed;
    aliases.spiMode = config.imu.spiMode;
    aliases.spiBits = config.imu.spiBits;
    aliases.gpiochip = config.imu.gpiochip;
    aliases.drdyLine = config.imu.drdyLine;
    aliases.imuHz = config.imu.imuHz;
    aliases.accelFsG = config.imu.accelFsG;
    aliases.gyroFsDps = config.imu.gyroFsDps;
    aliases.imuStartReg = config.imu.imuStartReg;
}

void PrintCameraConfig(const MainRuntimeAliases &aliases)
{
    const bool packedStereo = CompiledCameraProviderUsesPackedStereo();
    std::cerr << "sensor_mode=" << ToSensorModeText(aliases.sensorMode) << "\n";
    std::cerr << "cam " << aliases.width << "x" << aliases.height << " @" << aliases.fps
              << " aeDisable=" << (aliases.aeDisable ? "true" : "false")
              << " exp_us=" << aliases.exposureUs << " gain=" << aliases.gain
              << " pixelFormat=" << (packedStereo ? "YUYV_packed_stereo" : "R16") << "\n";
    std::cerr << "cam_select left_index=" << aliases.leftCamIndex
              << " right_index=" << aliases.rightCamIndex
              << " uvc_device_index=" << aliases.uvcDeviceIndex << "\n";
    std::cerr << "stereo_input eye=" << aliases.uvcEyeWidth << "x" << aliases.uvcEyeHeight
              << " packed_stereo=" << (aliases.uvcPackedStereo ? "Y" : "N")
              << " swap_eyes=" << (aliases.uvcSwapEyes ? "Y" : "N")
              << " pair_window_ms=" << aliases.pairMs
              << " keep_window_ms=" << aliases.keepMs
              << " frame_queue=" << aliases.pairQueue << "\n";
    if (packedStereo) {
        std::cerr << "stereo_input_note=single_uvc_frame_split_left_right_no_timestamp_pairing\n";
    }
}

void PrintBackendConfig(const AppConfig &app, const MainRuntimeAliases &aliases)
{
    std::cerr << "slam_backend=" << ToSlamBackendText(aliases.slamBackend) << "\n";
    std::cerr << "feature_frontend=" << ToFeatureFrontendText(aliases.featureFrontend) << "\n";
    if (aliases.slamBackend == SlamBackend::DpvoTensorRt) {
        std::cerr << "dpvo_tensorrt repo=" << app.runtime.dpvoRepo
                  << " patch_engine=" << app.runtime.dpvoPatchEngine
                  << " update_engine=" << app.runtime.dpvoUpdateEngine
                  << " input=" << app.runtime.dpvoInputWidth << "x" << app.runtime.dpvoInputHeight
                  << " patches=" << app.runtime.dpvoPatchesPerFrame
                  << " opt_window=" << app.runtime.dpvoOptimizationWindow << "\n";
    }
    if (IsVisualFeatureLightGlueFrontend(aliases.featureFrontend)) {
        std::cerr << "visual_feature_frontend repo=" << app.runtime.visualFeatureRepo
                  << " device=" << app.runtime.visualFeatureDevice
                  << " top_k=" << app.runtime.visualFeatureTopK
                  << " max_points=" << app.runtime.visualFeatureMaxPoints
                  << " input_max=" << app.runtime.visualFeatureInputMaxWidth
                  << "x" << app.runtime.visualFeatureInputMaxHeight << "\n";
    }
}

void PrintOrbConfig(const AppConfig &app, const MainRuntimeAliases &aliases)
{
    if (aliases.slamBackend != SlamBackend::OrbSlam3) {
        return;
    }
    std::cerr << "orb nFeatures=" << app.runtime.orbNFeatures
              << " scaleFactor=" << app.runtime.orbScaleFactor
              << " nLevels=" << app.runtime.orbNLevels
              << " iniThFAST=" << app.runtime.orbIniThFAST
              << " minThFAST=" << app.runtime.orbMinThFAST
              << " accel=" << app.runtime.orbAcceleration << "\n";
    std::cerr << "vocab=" << app.vocab << "\n";
}

void PrintRuntimeFeatureConfig(const AppConfig &app, const MainRuntimeAliases &aliases)
{
    std::cerr << "slam_input_fps=" << aliases.slamInputFps << " camera_fps=" << aliases.fps
              << " frame_drop=" << (aliases.slamInputFps < aliases.fps ? "Y" : "N") << "\n";
    std::cerr << "slam_mode=" << smartdrone::core::domain::ToString(aliases.slamOperationMode) << "\n";
    PrintBackendConfig(app, aliases);
    PrintOrbConfig(app, aliases);
    std::cerr << "lk seed=gftt"
              << " loop_closure=" << (app.runtime.lkLoopClosure ? "Y" : "N")
              << " scale=" << app.runtime.lkLoopScale
              << " relax=" << app.runtime.lkLoopRelaxation
              << " per_frame_accel=" << app.runtime.lkPerFrameAcceleration << "\n";
}

void PrintDiagnosticsConfig(const AppConfig &app, const MainRuntimeAliases &aliases)
{
    std::cerr << "debug right_only_features=" << (aliases.debugRightOnlyFeatures ? "Y" : "N") << "\n";
    std::cerr << "slam lowlight_enhance=" << (aliases.slamLowLightEnhance ? "Y" : "N") << "\n";
    std::cerr << "diagnostics json=" << (aliases.jsonDiagnostics ? "Y" : "N") << "\n";
    std::cerr << "imuHz=" << aliases.imuHz << " udp=" << (aliases.udpEnable ? "Y" : "N")
              << " udpPort=" << aliases.udpPort << " cmdPort=" << aliases.cmdPort << "\n";
    std::cerr << "stream img=" << (aliases.sendImage ? "Y" : "N")
              << " feat=" << (aliases.sendFeature ? "Y" : "N")
              << " map=" << (aliases.sendMap ? "Y" : "N") << "\n";
    std::cerr << "settings=" << app.settings << "\n";
}

} // namespace

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

MainRuntimeAliases BuildRuntimeAliases(const AppConfig &config)
{
    MainRuntimeAliases aliases{};
    ApplyCameraAliases(aliases, config);
    ApplyUdpAliases(aliases, config);
    ApplyRuntimeAliases(aliases, config);
    ApplyImuAliases(aliases, config);
    return aliases;
}

void PrintStartupConfig(const AppConfig &app, const MainRuntimeAliases &aliases,
                        smartdrone::core::domain::RuntimeMode mode)
{
    std::cerr << "mode=" << smartdrone::core::domain::ToString(mode) << "\n";
    PrintCameraConfig(aliases);
    PrintRuntimeFeatureConfig(app, aliases);
    PrintDiagnosticsConfig(app, aliases);
}

} // namespace smartdrone::core::application
