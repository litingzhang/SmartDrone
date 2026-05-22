#include "core/application/config/config_registry.h"

#include <string>

namespace SmartDrone::Core::Application {
namespace {

using DescriptorList = std::vector<Domain::ConfigDescriptor>;

Domain::ConfigDescriptor Make(std::string_view key,
                              std::string_view description,
                              bool hotReloadable,
                              bool requiresPipelineRestart,
                              bool requiresDeviceRestart)
{
    Domain::ConfigDescriptor out{};
    out.key = std::string(key);
    out.description = std::string(description);
    out.hotReloadable = hotReloadable;
    out.requiresPipelineRestart = requiresPipelineRestart;
    out.requiresDeviceRestart = requiresDeviceRestart;
    return out;
}

void AppendDescriptors(DescriptorList &target, DescriptorList descriptors)
{
    target.insert(target.end(), descriptors.begin(), descriptors.end());
}

DescriptorList CameraDescriptors()
{
    return {
        Make(ConfigRegistry::kCameraExposureUs,
             "Camera exposure time in microseconds", false, true, true),
        Make(ConfigRegistry::kCameraGain, "Camera analog gain", false, true,
             true),
        Make(ConfigRegistry::kCameraAutoExposure,
             "Enable camera auto exposure (CM5 ISP/libcamera controls)", false,
             true, true),
        Make(ConfigRegistry::kCameraPairWindowMs,
             "Stereo pairing window in milliseconds for providers that pair "
             "separate left/right streams",
             false, true, true),
        Make(ConfigRegistry::kCameraUvcDeviceIndex,
             "Single UVC device index for packed stereo capture", false, true,
             true),
        Make(ConfigRegistry::kCameraUvcEyeWidth,
             "Per-eye width for packed UVC stereo frames", false, true, true),
        Make(ConfigRegistry::kCameraUvcEyeHeight,
             "Per-eye height for packed UVC stereo frames", false, true, true),
        Make(ConfigRegistry::kCameraUvcPackedStereo,
             "Expect one UVC frame containing left-right packed stereo images",
             false, true, true),
    };
}

DescriptorList SlamModeDescriptors()
{
    return {
        Make(ConfigRegistry::kSlamInputFps,
             "Input frame rate delivered to the SLAM engine", true, false, false),
        Make(ConfigRegistry::kSlamBackend,
             "SLAM backend selection: klt, dpvo_tensorrt, or orbslam3 when "
             "compiled",
             false, true, false),
        Make(ConfigRegistry::kSlamFeatureFrontend,
             "Feature frontend selection such as lk_gftt_per_frame, lk, orb, "
             "superpoint_lightglue, or xfeat_lightglue",
             false, true, false),
        Make(ConfigRegistry::kSlamPerceptionMode,
             "SLAM perception mode such as stereo or stereo-imu", false, true,
             false),
        Make(ConfigRegistry::kSlamOperationMode,
             "SLAM operating mode such as mapping or localization", true, false,
             false),
    };
}

DescriptorList TbcDescriptors()
{
    return {
        Make(ConfigRegistry::kSlamUseCustomTbc,
             "Override T_b_c1 using runtime-configured translation and "
             "roll/pitch/yaw",
             true, false, false),
        Make(ConfigRegistry::kSlamTbcTx,
             "Runtime T_b_c1 override translation X (meters)", true, false,
             false),
        Make(ConfigRegistry::kSlamTbcTy,
             "Runtime T_b_c1 override translation Y (meters)", true, false,
             false),
        Make(ConfigRegistry::kSlamTbcTz,
             "Runtime T_b_c1 override translation Z (meters)", true, false,
             false),
        Make(ConfigRegistry::kSlamTbcRollDeg,
             "Runtime T_b_c1 override camera roll angle (degrees)", true, false,
             false),
        Make(ConfigRegistry::kSlamTbcPitchDeg,
             "Runtime T_b_c1 override camera pitch angle (degrees)", true, false,
             false),
        Make(ConfigRegistry::kSlamTbcYawDeg,
             "Runtime T_b_c1 override camera yaw angle (degrees)", true, false,
             false),
    };
}

DescriptorList OrbDescriptors()
{
    return {
        Make(ConfigRegistry::kSlamOrbNFeatures,
             "ORB extractor max features per frame", false, true, false),
        Make(ConfigRegistry::kSlamOrbScaleFactor,
             "ORB extractor pyramid scale factor", false, true, false),
        Make(ConfigRegistry::kSlamOrbNLevels, "ORB extractor pyramid levels",
             false, true, false),
        Make(ConfigRegistry::kSlamOrbIniThFast,
             "ORB extractor initial FAST threshold", false, true, false),
        Make(ConfigRegistry::kSlamOrbMinThFast,
             "ORB extractor minimum FAST threshold", false, true, false),
    };
}

DescriptorList VisualFeatureDescriptors()
{
    return {
        Make(ConfigRegistry::kSlamVisualFeatureTopK,
             "Visual feature detector top-k candidate count before filtering",
             false, true, false),
        Make(ConfigRegistry::kSlamVisualFeatureMaxPoints,
             "Visual feature maximum injected feature count per eye", false, true,
             false),
        Make(ConfigRegistry::kSlamSuperPointTopK,
             "Deprecated alias for slam.visual_feature_top_k", false, true,
             false),
        Make(ConfigRegistry::kSlamSuperPointMaxPoints,
             "Deprecated alias for slam.visual_feature_max_points", false, true,
             false),
        Make(ConfigRegistry::kSlamVisualFeatureInputMaxWidth,
             "Visual feature frontend input maximum width; zero disables "
             "width-based downscaling",
             false, true, false),
        Make(ConfigRegistry::kSlamVisualFeatureInputMaxHeight,
             "Visual feature frontend input maximum height; zero disables "
             "height-based downscaling",
             false, true, false),
        Make(ConfigRegistry::kSlamSuperPointInputMaxWidth,
             "Deprecated alias for slam.visual_feature_input_max_width", false,
             true, false),
        Make(ConfigRegistry::kSlamSuperPointInputMaxHeight,
             "Deprecated alias for slam.visual_feature_input_max_height", false,
             true, false),
    };
}

DescriptorList AccelerationDescriptors()
{
    return {
        Make(ConfigRegistry::kSlamLkSuperPointSeeding,
             "Deprecated; LK always uses GFTT/Shi-Tomasi seed points", false,
             true, false),
        Make(ConfigRegistry::kSlamLkPerFrameAcceleration,
             "Acceleration backend for KLT tracking mode: cpu or vpi-cuda", false,
             true, false),
        Make(ConfigRegistry::kSlamOrbAcceleration,
             "Acceleration backend for ORB: cpu, cuda, or vpi-remap", false, true,
             false),
    };
}

DescriptorList StreamDescriptors()
{
    return {
        Make(ConfigRegistry::kStreamUdpEnabled,
             "Enable UDP preview and telemetry streaming", false, true, false),
        Make(ConfigRegistry::kStreamUdpIp,
             "Destination IP for UDP preview streaming", false, true, false),
        Make(ConfigRegistry::kStreamSendImage, "Enable image preview streaming",
             false, true, false),
        Make(ConfigRegistry::kStreamSendFeature,
             "Enable tracked feature streaming", false, true, false),
        Make(ConfigRegistry::kStreamSendMap, "Enable pose and map streaming",
             false, true, false),
    };
}

} // namespace

std::vector<Domain::ConfigDescriptor> ConfigRegistry::DefaultDescriptors()
{
    DescriptorList descriptors;
    AppendDescriptors(descriptors, CameraDescriptors());
    AppendDescriptors(descriptors, SlamModeDescriptors());
    AppendDescriptors(descriptors, TbcDescriptors());
    AppendDescriptors(descriptors, OrbDescriptors());
    AppendDescriptors(descriptors, VisualFeatureDescriptors());
    AppendDescriptors(descriptors, AccelerationDescriptors());
    AppendDescriptors(descriptors, StreamDescriptors());
    return descriptors;
}

} // namespace SmartDrone::Core::Application
