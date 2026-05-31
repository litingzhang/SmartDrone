#pragma once

#include <string_view>
#include <vector>

#include "core/domain/capabilities.h"

namespace SmartDrone::Core::Application {

class ConfigRegistry {
  public:
    static constexpr std::string_view CAMERA_EXPOSURE_US = "camera.exposure_us";
    static constexpr std::string_view CAMERA_GAIN = "camera.gain";
    static constexpr std::string_view CAMERA_AUTO_EXPOSURE =
        "camera.auto_exposure";
    static constexpr std::string_view CAMERA_PAIR_WINDOW_MS =
        "camera.pair_window_ms";
    static constexpr std::string_view CAMERA_UVC_DEVICE_INDEX =
        "camera.uvc_device_index";
    static constexpr std::string_view CAMERA_UVC_EYE_WIDTH = "camera.uvc_eye_width";
    static constexpr std::string_view CAMERA_UVC_EYE_HEIGHT =
        "camera.uvc_eye_height";
    static constexpr std::string_view CAMERA_UVC_PACKED_STEREO =
        "camera.uvc_packed_stereo";
    static constexpr std::string_view SLAM_INPUT_FPS = "slam.input_fps";
    static constexpr std::string_view SLAM_BACKEND = "slam.backend";
    static constexpr std::string_view SLAM_FEATURE_FRONTEND =
        "slam.feature_frontend";
    static constexpr std::string_view SLAM_PERCEPTION_MODE =
        "slam.perception_mode";
    static constexpr std::string_view SLAM_OPERATION_MODE = "slam.operation_mode";
    static constexpr std::string_view STREAM_UDP_ENABLED = "stream.udp_enabled";
    static constexpr std::string_view STREAM_UDP_IP = "stream.udp_ip";
    static constexpr std::string_view STREAM_SEND_IMAGE = "stream.send_image";
    static constexpr std::string_view STREAM_SEND_FEATURE = "stream.send_feature";
    static constexpr std::string_view STREAM_SEND_MAP = "stream.send_map";
    static constexpr std::string_view SLAM_USE_CUSTOM_TBC =
        "slam.tbc_override_enabled";
    static constexpr std::string_view SLAM_TBC_TX = "slam.tbc_tx_m";
    static constexpr std::string_view SLAM_TBC_TY = "slam.tbc_ty_m";
    static constexpr std::string_view SLAM_TBC_TZ = "slam.tbc_tz_m";
    static constexpr std::string_view SLAM_TBC_ROLL_DEG = "slam.tbc_roll_deg";
    static constexpr std::string_view SLAM_TBC_PITCH_DEG = "slam.tbc_pitch_deg";
    static constexpr std::string_view SLAM_TBC_YAW_DEG = "slam.tbc_yaw_deg";
    static constexpr std::string_view SLAM_ORB_N_FEATURES = "slam.orb_nfeatures";
    static constexpr std::string_view SLAM_ORB_SCALE_FACTOR =
        "slam.orb_scale_factor";
    static constexpr std::string_view SLAM_ORB_N_LEVELS = "slam.orb_nlevels";
    static constexpr std::string_view SLAM_ORB_INI_TH_FAST = "slam.orb_ini_th_fast";
    static constexpr std::string_view SLAM_ORB_MIN_TH_FAST = "slam.orb_min_th_fast";
    static constexpr std::string_view SLAM_VISUAL_FEATURE_TOP_K =
        "slam.visual_feature_top_k";
    static constexpr std::string_view SLAM_VISUAL_FEATURE_MAX_POINTS =
        "slam.visual_feature_max_points";
    static constexpr std::string_view SLAM_SUPER_POINT_TOP_K =
        "slam.superpoint_top_k";
    static constexpr std::string_view SLAM_SUPER_POINT_MAX_POINTS =
        "slam.superpoint_max_points";
    static constexpr std::string_view SLAM_VISUAL_FEATURE_INPUT_MAX_WIDTH =
        "slam.visual_feature_input_max_width";
    static constexpr std::string_view SLAM_VISUAL_FEATURE_INPUT_MAX_HEIGHT =
        "slam.visual_feature_input_max_height";
    static constexpr std::string_view SLAM_SUPER_POINT_INPUT_MAX_WIDTH =
        "slam.superpoint_input_max_width";
    static constexpr std::string_view SLAM_SUPER_POINT_INPUT_MAX_HEIGHT =
        "slam.superpoint_input_max_height";
    static constexpr std::string_view SLAM_LK_SUPER_POINT_SEEDING =
        "slam.lk_superpoint_seeding";
    static constexpr std::string_view SLAM_LK_PER_FRAME_ACCELERATION =
        "slam.lk_per_frame_accel";
    static constexpr std::string_view SLAM_ORB_ACCELERATION = "slam.orb_accel";
    static constexpr std::string_view AVOIDANCE_ENABLED =
        "avoidance.enabled";
    static constexpr std::string_view AVOIDANCE_HOLD_ON_STALE_CLOUD =
        "avoidance.hold_on_stale_cloud";
    static constexpr std::string_view AVOIDANCE_RADIUS_M =
        "avoidance.radius_m";
    static constexpr std::string_view AVOIDANCE_LOOKAHEAD_M =
        "avoidance.lookahead_m";
    static constexpr std::string_view AVOIDANCE_SPEED_LOOKAHEAD_S =
        "avoidance.speed_lookahead_s";
    static constexpr std::string_view AVOIDANCE_NEAR_FIELD_RADIUS_M =
        "avoidance.near_field_radius_m";
    static constexpr std::string_view AVOIDANCE_MAX_POINT_AGE_MS =
        "avoidance.max_point_age_ms";
    static constexpr std::string_view AVOIDANCE_MIN_CLOUD_POINTS =
        "avoidance.min_cloud_points";
    static constexpr std::string_view AVOIDANCE_MIN_BLOCKING_POINTS =
        "avoidance.min_blocking_points";

    static std::vector<Domain::ConfigDescriptor> DefaultDescriptors();
};

} // namespace SmartDrone::Core::Application
