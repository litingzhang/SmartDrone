#include "core/application/config/config_registry.h"

#include <string>

namespace smartdrone::core::application {

std::vector<domain::ConfigDescriptor> ConfigRegistry::DefaultDescriptors()
{
    return {
        Make(kCameraExposureUs, "Camera exposure time in microseconds", false, true, true),
        Make(kCameraGain, "Camera analog gain", false, true, true),
        Make(kCameraAutoExposure, "Enable camera auto exposure (CM5 ISP/libcamera controls)", false, true, true),
        Make(kCameraPairWindowMs, "Stereo pairing window in milliseconds", false, true, true),
        Make(kSlamInputFps, "Input frame rate delivered to the SLAM engine", true, false, false),
        Make(kSlamFeatureFrontend, "Feature frontend selection such as orb or xfeat", false, true, false),
        Make(kSlamPerceptionMode, "SLAM perception mode such as stereo or stereo-imu", false, true, false),
        Make(kSlamOperationMode, "SLAM operating mode such as mapping or localization", true, false, false),
        Make(kSlamUseCustomTbc, "Override T_b_c1 using runtime-configured translation and roll/pitch/yaw", true, false,
             false),
        Make(kSlamTbcTx, "Runtime T_b_c1 override translation X (meters)", true, false, false),
        Make(kSlamTbcTy, "Runtime T_b_c1 override translation Y (meters)", true, false, false),
        Make(kSlamTbcTz, "Runtime T_b_c1 override translation Z (meters)", true, false, false),
        Make(kSlamTbcRollDeg, "Runtime T_b_c1 override camera roll angle (degrees)", true, false, false),
        Make(kSlamTbcPitchDeg, "Runtime T_b_c1 override camera pitch angle (degrees)", true, false, false),
        Make(kSlamTbcYawDeg, "Runtime T_b_c1 override camera yaw angle (degrees)", true, false, false),
        Make(kSlamOrbNFeatures, "ORB extractor max features per frame", false, true, false),
        Make(kSlamOrbScaleFactor, "ORB extractor pyramid scale factor", false, true, false),
        Make(kSlamOrbNLevels, "ORB extractor pyramid levels", false, true, false),
        Make(kSlamOrbIniThFast, "ORB extractor initial FAST threshold", false, true, false),
        Make(kSlamOrbMinThFast, "ORB extractor minimum FAST threshold", false, true, false),
        Make(kStreamUdpEnabled, "Enable UDP preview and telemetry streaming", false, true, false),
        Make(kStreamUdpIp, "Destination IP for UDP preview streaming", false, true, false),
        Make(kStreamSendImage, "Enable image preview streaming", false, true, false),
        Make(kStreamSendFeature, "Enable tracked feature streaming", false, true, false),
        Make(kStreamSendMap, "Enable pose and map streaming", false, true, false),
    };
}

domain::ConfigDescriptor ConfigRegistry::Make(std::string_view key, std::string_view description, bool hotReloadable,
                                              bool requiresPipelineRestart, bool requiresDeviceRestart)
{
    domain::ConfigDescriptor out{};
    out.key = std::string(key);
    out.description = std::string(description);
    out.hotReloadable = hotReloadable;
    out.requiresPipelineRestart = requiresPipelineRestart;
    out.requiresDeviceRestart = requiresDeviceRestart;
    return out;
}

} // namespace smartdrone::core::application
