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
        Make(ConfigRegistry::CAMERA_EXPOSURE_US,
             "Camera exposure time in microseconds", false, true, true),
        Make(ConfigRegistry::CAMERA_GAIN, "Camera analog gain", false, true,
             true),
        Make(ConfigRegistry::CAMERA_AUTO_EXPOSURE,
             "Enable camera auto exposure (CM5 ISP/libcamera controls)", false,
             true, true),
        Make(ConfigRegistry::CAMERA_PAIR_WINDOW_MS,
             "Stereo pairing window in milliseconds for providers that pair "
             "separate left/right streams",
             false, true, true),
        Make(ConfigRegistry::CAMERA_UVC_DEVICE_INDEX,
             "Single UVC device index for packed stereo capture", false, true,
             true),
        Make(ConfigRegistry::CAMERA_UVC_EYE_WIDTH,
             "Per-eye width for packed UVC stereo frames", false, true, true),
        Make(ConfigRegistry::CAMERA_UVC_EYE_HEIGHT,
             "Per-eye height for packed UVC stereo frames", false, true, true),
        Make(ConfigRegistry::CAMERA_UVC_PACKED_STEREO,
             "Expect one UVC frame containing left-right packed stereo images",
             false, true, true),
    };
}

DescriptorList SlamModeDescriptors()
{
    return {
        Make(ConfigRegistry::SLAM_INPUT_FPS,
             "Input frame rate delivered to the SLAM engine", true, false, false),
        Make(ConfigRegistry::SLAM_BACKEND,
             "SLAM backend selection: klt, dpvo_tensorrt, openvins, or "
             "orbslam3 when compiled",
             false, true, false),
        Make(ConfigRegistry::SLAM_FEATURE_FRONTEND,
             "Feature frontend selection such as lk_gftt_per_frame, lk, orb, "
             "superpoint_lightglue, or xfeat_lightglue",
             false, true, false),
        Make(ConfigRegistry::SLAM_PERCEPTION_MODE,
             "SLAM perception mode such as stereo or stereo-imu", false, true,
             false),
        Make(ConfigRegistry::SLAM_OPERATION_MODE,
             "SLAM operating mode such as mapping or localization", true, false,
             false),
    };
}

DescriptorList TbcDescriptors()
{
    return {
        Make(ConfigRegistry::SLAM_USE_CUSTOM_TBC,
             "Override T_b_c1 using runtime-configured translation and "
             "roll/pitch/yaw",
             true, false, false),
        Make(ConfigRegistry::SLAM_TBC_TX,
             "Runtime T_b_c1 override translation X (meters)", true, false,
             false),
        Make(ConfigRegistry::SLAM_TBC_TY,
             "Runtime T_b_c1 override translation Y (meters)", true, false,
             false),
        Make(ConfigRegistry::SLAM_TBC_TZ,
             "Runtime T_b_c1 override translation Z (meters)", true, false,
             false),
        Make(ConfigRegistry::SLAM_TBC_ROLL_DEG,
             "Runtime T_b_c1 override camera roll angle (degrees)", true, false,
             false),
        Make(ConfigRegistry::SLAM_TBC_PITCH_DEG,
             "Runtime T_b_c1 override camera pitch angle (degrees)", true, false,
             false),
        Make(ConfigRegistry::SLAM_TBC_YAW_DEG,
             "Runtime T_b_c1 override camera yaw angle (degrees)", true, false,
             false),
    };
}

DescriptorList OrbDescriptors()
{
    return {
        Make(ConfigRegistry::SLAM_ORB_N_FEATURES,
             "ORB extractor max features per frame", false, true, false),
        Make(ConfigRegistry::SLAM_ORB_SCALE_FACTOR,
             "ORB extractor pyramid scale factor", false, true, false),
        Make(ConfigRegistry::SLAM_ORB_N_LEVELS, "ORB extractor pyramid levels",
             false, true, false),
        Make(ConfigRegistry::SLAM_ORB_INI_TH_FAST,
             "ORB extractor initial FAST threshold", false, true, false),
        Make(ConfigRegistry::SLAM_ORB_MIN_TH_FAST,
             "ORB extractor minimum FAST threshold", false, true, false),
    };
}

DescriptorList VisualFeatureDescriptors()
{
    return {
        Make(ConfigRegistry::SLAM_VISUAL_FEATURE_TOP_K,
             "Visual feature detector top-k candidate count before filtering",
             false, true, false),
        Make(ConfigRegistry::SLAM_VISUAL_FEATURE_MAX_POINTS,
             "Visual feature maximum injected feature count per eye", false, true,
             false),
        Make(ConfigRegistry::SLAM_SUPER_POINT_TOP_K,
             "Deprecated alias for slam.visual_feature_top_k", false, true,
             false),
        Make(ConfigRegistry::SLAM_SUPER_POINT_MAX_POINTS,
             "Deprecated alias for slam.visual_feature_max_points", false, true,
             false),
        Make(ConfigRegistry::SLAM_VISUAL_FEATURE_INPUT_MAX_WIDTH,
             "Visual feature frontend input maximum width; zero disables "
             "width-based downscaling",
             false, true, false),
        Make(ConfigRegistry::SLAM_VISUAL_FEATURE_INPUT_MAX_HEIGHT,
             "Visual feature frontend input maximum height; zero disables "
             "height-based downscaling",
             false, true, false),
        Make(ConfigRegistry::SLAM_SUPER_POINT_INPUT_MAX_WIDTH,
             "Deprecated alias for slam.visual_feature_input_max_width", false,
             true, false),
        Make(ConfigRegistry::SLAM_SUPER_POINT_INPUT_MAX_HEIGHT,
             "Deprecated alias for slam.visual_feature_input_max_height", false,
             true, false),
    };
}

DescriptorList AccelerationDescriptors()
{
    return {
        Make(ConfigRegistry::SLAM_LK_SUPER_POINT_SEEDING,
             "Deprecated; LK always uses GFTT/Shi-Tomasi seed points", false,
             true, false),
        Make(ConfigRegistry::SLAM_LK_PER_FRAME_ACCELERATION,
             "Acceleration backend for KLT tracking mode: cpu or vpi-cuda", false,
             true, false),
        Make(ConfigRegistry::SLAM_ORB_ACCELERATION,
             "Acceleration backend for ORB: cpu, cuda, or vpi-remap", false, true,
             false),
    };
}

DescriptorList StreamDescriptors()
{
    return {
        Make(ConfigRegistry::STREAM_UDP_ENABLED,
             "Enable UDP preview and telemetry streaming", false, true, false),
        Make(ConfigRegistry::STREAM_UDP_IP,
             "Destination IP for UDP preview streaming", false, true, false),
        Make(ConfigRegistry::STREAM_SEND_IMAGE, "Enable image preview streaming",
             false, true, false),
        Make(ConfigRegistry::STREAM_SEND_FEATURE,
             "Enable tracked feature streaming", false, true, false),
        Make(ConfigRegistry::STREAM_SEND_MAP, "Enable pose and map streaming",
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
