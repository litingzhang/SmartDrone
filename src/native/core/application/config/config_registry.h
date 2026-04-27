#pragma once

#include <string_view>
#include <vector>

#include "core/domain/capabilities.h"

namespace smartdrone::core::application {

class ConfigRegistry {
  public:
    static constexpr std::string_view kCameraExposureUs = "camera.exposure_us";
    static constexpr std::string_view kCameraGain = "camera.gain";
    static constexpr std::string_view kCameraAutoExposure = "camera.auto_exposure";
    static constexpr std::string_view kCameraPairWindowMs = "camera.pair_window_ms";
    static constexpr std::string_view kCameraUvcDeviceIndex = "camera.uvc_device_index";
    static constexpr std::string_view kCameraUvcEyeWidth = "camera.uvc_eye_width";
    static constexpr std::string_view kCameraUvcEyeHeight = "camera.uvc_eye_height";
    static constexpr std::string_view kCameraUvcPackedStereo = "camera.uvc_packed_stereo";
    static constexpr std::string_view kSlamInputFps = "slam.input_fps";
    static constexpr std::string_view kSlamFeatureFrontend = "slam.feature_frontend";
    static constexpr std::string_view kSlamPerceptionMode = "slam.perception_mode";
    static constexpr std::string_view kSlamOperationMode = "slam.operation_mode";
    static constexpr std::string_view kStreamUdpEnabled = "stream.udp_enabled";
    static constexpr std::string_view kStreamUdpIp = "stream.udp_ip";
    static constexpr std::string_view kStreamSendImage = "stream.send_image";
    static constexpr std::string_view kStreamSendFeature = "stream.send_feature";
    static constexpr std::string_view kStreamSendMap = "stream.send_map";
    static constexpr std::string_view kSlamUseCustomTbc = "slam.tbc_override_enabled";
    static constexpr std::string_view kSlamTbcTx = "slam.tbc_tx_m";
    static constexpr std::string_view kSlamTbcTy = "slam.tbc_ty_m";
    static constexpr std::string_view kSlamTbcTz = "slam.tbc_tz_m";
    static constexpr std::string_view kSlamTbcRollDeg = "slam.tbc_roll_deg";
    static constexpr std::string_view kSlamTbcPitchDeg = "slam.tbc_pitch_deg";
    static constexpr std::string_view kSlamTbcYawDeg = "slam.tbc_yaw_deg";
    static constexpr std::string_view kSlamOrbNFeatures = "slam.orb_nfeatures";
    static constexpr std::string_view kSlamOrbScaleFactor = "slam.orb_scale_factor";
    static constexpr std::string_view kSlamOrbNLevels = "slam.orb_nlevels";
    static constexpr std::string_view kSlamOrbIniThFast = "slam.orb_ini_th_fast";
    static constexpr std::string_view kSlamOrbMinThFast = "slam.orb_min_th_fast";
    static constexpr std::string_view kSlamXFeatTopK = "slam.xfeat_top_k";
    static constexpr std::string_view kSlamXFeatMaxPoints = "slam.xfeat_max_points";
    static constexpr std::string_view kSlamXFeatInputMaxWidth = "slam.xfeat_input_max_width";
    static constexpr std::string_view kSlamXFeatInputMaxHeight = "slam.xfeat_input_max_height";
    static constexpr std::string_view kSlamLkXFeatSeeding = "slam.lk_xfeat_seeding";
    static constexpr std::string_view kSlamLkPerFrameAcceleration = "slam.lk_per_frame_accel";

    static std::vector<domain::ConfigDescriptor> DefaultDescriptors();

  private:
    static domain::ConfigDescriptor Make(std::string_view key, std::string_view description, bool hotReloadable,
                                         bool requiresPipelineRestart, bool requiresDeviceRestart);
};

} // namespace smartdrone::core::application
