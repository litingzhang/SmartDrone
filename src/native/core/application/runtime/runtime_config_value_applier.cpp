#include "core/application/runtime/runtime_config_value_applier.h"

#include <string>
#include <variant>

#include "core/application/config/app_args.h"
#include "core/application/config/config_registry.h"
#include "core/application/runtime/runtime_config_value_assignments.h"

namespace SmartDrone::Core::Application {
namespace {

CommandResult ApplyCameraConfigValue(const std::string &key,
                                     const ConfigValue &value,
                                     RemoteRuntimeConfig &remote,
                                     bool &handled)
{
    handled = true;
    if (key == ConfigRegistry::CAMERA_EXPOSURE_US) {
        return AssignStrictInt(value, remote.exposureUs, "camera.exposure_us");
    }
    if (key == ConfigRegistry::CAMERA_GAIN) {
        return AssignFloat(value, remote.gain, "camera.gain");
    }
    if (key == ConfigRegistry::CAMERA_AUTO_EXPOSURE) {
        return AssignBool(value, remote.autoExposureEnabled, "camera.auto_exposure");
    }
    if (key == ConfigRegistry::CAMERA_PAIR_WINDOW_MS) {
        return AssignStrictInt(value, remote.pairMs, "camera.pair_window_ms");
    }
    if (key == ConfigRegistry::CAMERA_UVC_DEVICE_INDEX) {
        return AssignStrictInt(value, remote.uvcDeviceIndex,
                               "camera.uvc_device_index");
    }
    if (key == ConfigRegistry::CAMERA_UVC_EYE_WIDTH) {
        return AssignStrictInt(value, remote.uvcEyeWidth, "camera.uvc_eye_width");
    }
    if (key == ConfigRegistry::CAMERA_UVC_EYE_HEIGHT) {
        return AssignStrictInt(value, remote.uvcEyeHeight, "camera.uvc_eye_height");
    }
    if (key == ConfigRegistry::CAMERA_UVC_PACKED_STEREO) {
        return AssignBool(value, remote.uvcPackedStereo, "camera.uvc_packed_stereo");
    }
    handled = false;
    return OkResult();
}

CommandResult ApplySlamModeConfigValue(const std::string &key,
                                       const ConfigValue &value,
                                       RemoteRuntimeConfig &remote,
                                       bool &handled)
{
    handled = true;
    if (key == ConfigRegistry::SLAM_INPUT_FPS) {
        return AssignStrictInt(value, remote.slamInputFps, "slam.input_fps");
    }
    if (key == ConfigRegistry::SLAM_BACKEND) {
        if (const auto *text = std::get_if<std::string>(&value)) {
            remote.slamBackend =
                NormalizeSlamBackendForBuild(ParseSlamBackendText(*text));
            return OkResult();
        }
        return TypeMismatchResult("slam.backend");
    }
    if (key == ConfigRegistry::SLAM_FEATURE_FRONTEND) {
        if (const auto *text = std::get_if<std::string>(&value)) {
            remote.featureFrontend = ParseFeatureFrontendText(*text);
            return OkResult();
        }
        return TypeMismatchResult("slam.feature_frontend");
    }
    if (key == ConfigRegistry::SLAM_OPERATION_MODE) {
        if (const auto *text = std::get_if<std::string>(&value)) {
            remote.slamOperationMode = ParseSlamOperationModeText(*text);
            return OkResult();
        }
        return TypeMismatchResult("slam.operation_mode");
    }
    if (key == ConfigRegistry::SLAM_PERCEPTION_MODE) {
        if (const auto *text = std::get_if<std::string>(&value)) {
            remote.sensorMode = ParseSensorModeText(*text);
            return OkResult();
        }
        return TypeMismatchResult("slam.perception_mode");
    }
    handled = false;
    return OkResult();
}

CommandResult ApplyStreamConfigValue(const std::string &key,
                                     const ConfigValue &value,
                                     RemoteRuntimeConfig &remote,
                                     bool &handled)
{
    handled = true;
    if (key == ConfigRegistry::STREAM_UDP_ENABLED) {
        return AssignBool(value, remote.udpEnabled, "stream.udp_enabled");
    }
    if (key == ConfigRegistry::STREAM_UDP_IP) {
        return AssignString(value, remote.udpIp, "stream.udp_ip");
    }
    if (key == ConfigRegistry::STREAM_SEND_IMAGE) {
        return AssignBool(value, remote.sendImage, "stream.send_image");
    }
    if (key == ConfigRegistry::STREAM_SEND_FEATURE) {
        return AssignBool(value, remote.sendFeature, "stream.send_feature");
    }
    if (key == ConfigRegistry::STREAM_SEND_MAP) {
        return AssignBool(value, remote.sendMap, "stream.send_map");
    }
    handled = false;
    return OkResult();
}

CommandResult ApplyTbcConfigValue(const std::string &key,
                                  const ConfigValue &value,
                                  RemoteRuntimeConfig &remote,
                                  bool &handled)
{
    handled = true;
    if (key == ConfigRegistry::SLAM_USE_CUSTOM_TBC) {
        return AssignBool(value, remote.useCustomTbc, "slam.tbc_override_enabled");
    }
    if (key == ConfigRegistry::SLAM_TBC_TX) {
        return AssignFloat(value, remote.tbcTx, "slam.tbc_tx_m");
    }
    if (key == ConfigRegistry::SLAM_TBC_TY) {
        return AssignFloat(value, remote.tbcTy, "slam.tbc_ty_m");
    }
    if (key == ConfigRegistry::SLAM_TBC_TZ) {
        return AssignFloat(value, remote.tbcTz, "slam.tbc_tz_m");
    }
    if (key == ConfigRegistry::SLAM_TBC_ROLL_DEG) {
        return AssignFloat(value, remote.tbcRollDeg, "slam.tbc_roll_deg");
    }
    if (key == ConfigRegistry::SLAM_TBC_PITCH_DEG) {
        return AssignFloat(value, remote.tbcPitchDeg, "slam.tbc_pitch_deg");
    }
    if (key == ConfigRegistry::SLAM_TBC_YAW_DEG) {
        return AssignFloat(value, remote.tbcYawDeg, "slam.tbc_yaw_deg");
    }
    handled = false;
    return OkResult();
}

CommandResult ApplyOrbConfigValue(const std::string &key,
                                  const ConfigValue &value,
                                  RemoteRuntimeConfig &remote,
                                  bool &handled)
{
    handled = true;
    if (key == ConfigRegistry::SLAM_ORB_N_FEATURES) {
        return AssignNumericInt(value, remote.orbNFeatures, "slam.orb_nfeatures");
    }
    if (key == ConfigRegistry::SLAM_ORB_SCALE_FACTOR) {
        return AssignFloat(value, remote.orbScaleFactor, "slam.orb_scale_factor");
    }
    if (key == ConfigRegistry::SLAM_ORB_N_LEVELS) {
        return AssignNumericInt(value, remote.orbNLevels, "slam.orb_nlevels");
    }
    if (key == ConfigRegistry::SLAM_ORB_INI_TH_FAST) {
        return AssignNumericInt(value, remote.orbIniThFAST, "slam.orb_ini_th_fast");
    }
    if (key == ConfigRegistry::SLAM_ORB_MIN_TH_FAST) {
        return AssignNumericInt(value, remote.orbMinThFAST, "slam.orb_min_th_fast");
    }
    handled = false;
    return OkResult();
}

CommandResult ApplyVisualFeatureConfigValue(const std::string &key,
                                            const ConfigValue &value,
                                            RemoteRuntimeConfig &remote,
                                            bool &handled)
{
    handled = true;
    if (key == ConfigRegistry::SLAM_VISUAL_FEATURE_TOP_K ||
        key == ConfigRegistry::SLAM_SUPER_POINT_TOP_K) {
        return AssignNumericInt(value, remote.visualFeatureTopK,
                                "slam.visual_feature_top_k");
    }
    if (key == ConfigRegistry::SLAM_VISUAL_FEATURE_MAX_POINTS ||
        key == ConfigRegistry::SLAM_SUPER_POINT_MAX_POINTS) {
        return AssignNumericInt(value, remote.visualFeatureMaxPoints,
                                "slam.visual_feature_max_points");
    }
    if (key == ConfigRegistry::SLAM_VISUAL_FEATURE_INPUT_MAX_WIDTH ||
        key == ConfigRegistry::SLAM_SUPER_POINT_INPUT_MAX_WIDTH) {
        return AssignNumericInt(value, remote.visualFeatureInputMaxWidth,
                                "slam.visual_feature_input_max_width");
    }
    if (key == ConfigRegistry::SLAM_VISUAL_FEATURE_INPUT_MAX_HEIGHT ||
        key == ConfigRegistry::SLAM_SUPER_POINT_INPUT_MAX_HEIGHT) {
        return AssignNumericInt(value, remote.visualFeatureInputMaxHeight,
                                "slam.visual_feature_input_max_height");
    }
    handled = false;
    return OkResult();
}

CommandResult ApplyAccelerationConfigValue(const std::string &key,
                                           const ConfigValue &value,
                                           RemoteRuntimeConfig &remote,
                                           bool &handled)
{
    handled = true;
    if (key == ConfigRegistry::SLAM_LK_SUPER_POINT_SEEDING) {
        if (std::holds_alternative<bool>(value)) {
            return OkResult();
        }
        return TypeMismatchResult("slam.lk_superpoint_seeding");
    }
    if (key == ConfigRegistry::SLAM_LK_PER_FRAME_ACCELERATION) {
        return AssignString(value, remote.lkPerFrameAcceleration,
                            "slam.lk_per_frame_accel");
    }
    if (key == ConfigRegistry::SLAM_ORB_ACCELERATION) {
        return AssignString(value, remote.orbAcceleration, "slam.orb_accel");
    }
    handled = false;
    return OkResult();
}

} // namespace

CommandResult ApplyConfigValue(const std::string &key,
                               const ConfigValue &value,
                               RemoteRuntimeConfig &remote)
{
    bool handled = false;
    CommandResult result = ApplyCameraConfigValue(key, value, remote, handled);
    if (handled || !result.ok) {
        return result;
    }
    result = ApplySlamModeConfigValue(key, value, remote, handled);
    if (handled || !result.ok) {
        return result;
    }
    result = ApplyStreamConfigValue(key, value, remote, handled);
    if (handled || !result.ok) {
        return result;
    }
    result = ApplyTbcConfigValue(key, value, remote, handled);
    if (handled || !result.ok) {
        return result;
    }
    result = ApplyOrbConfigValue(key, value, remote, handled);
    if (handled || !result.ok) {
        return result;
    }
    result = ApplyVisualFeatureConfigValue(key, value, remote, handled);
    if (handled || !result.ok) {
        return result;
    }
    result = ApplyAccelerationConfigValue(key, value, remote, handled);
    if (handled || !result.ok) {
        return result;
    }
    return {false, "unsupported config key: " + key};
}

} // namespace SmartDrone::Core::Application
