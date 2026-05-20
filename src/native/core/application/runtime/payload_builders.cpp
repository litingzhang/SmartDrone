#include "core/application/runtime/payload_builders.h"

#include <sstream>

#include "core/application/config/capability_catalog.h"
#include "core/application/sensors/camera_runtime_provider.h"

namespace smartdrone::core::application {

std::vector<uint8_t> TextPayloadFromString(const std::string &text) {
  return std::vector<uint8_t>(text.begin(), text.end());
}

std::string JoinStrings(const std::vector<std::string> &values,
                        const char *sep) {
  std::string out;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0)
      out += sep;
    out += values[i];
  }
  return out;
}

namespace {

const char *BoolText(bool value) { return value ? "true" : "false"; }

void AppendCameraConfig(std::ostringstream &oss, const UnifiedConfig &cfg) {
  const bool packedStereo = CompiledCameraProviderUsesPackedStereo();
  oss << "camera.exposure_us=" << cfg.app.camera.exposureUs << "\n";
  oss << "camera.gain=" << cfg.app.camera.gain << "\n";
  oss << "camera.auto_exposure=" << BoolText(!cfg.app.camera.aeDisable)
      << "\n";
  oss << "camera.auto_exposure_note="
      << (packedStereo ? "uvc_camera_firmware_auto_exposure"
                       : "sensor_provider_auto_exposure")
      << "\n";
  oss << "camera.pair_window_ms=" << cfg.app.camera.pairMs << "\n";
  oss << "camera.pair_window_ms_note="
      << (packedStereo ? "not_used_for_single_uvc_packed_stereo_frame"
                       : "provider_specific_for_separate_left_right_pairing")
      << "\n";
  oss << "camera.uvc_device_index=" << cfg.app.camera.uvcDeviceIndex << "\n";
  oss << "camera.uvc_eye_width=" << cfg.app.camera.uvcEyeWidth << "\n";
  oss << "camera.uvc_eye_height=" << cfg.app.camera.uvcEyeHeight << "\n";
  oss << "camera.uvc_packed_stereo="
      << BoolText(cfg.app.camera.uvcPackedStereo) << "\n";
}

void AppendSlamModeConfig(std::ostringstream &oss, const UnifiedConfig &cfg) {
  oss << "slam.input_fps=" << cfg.app.runtime.slamInputFps << "\n";
  oss << "slam.backend=" << ToSlamBackendText(cfg.app.runtime.slamBackend)
      << "\n";
  oss << "slam.feature_frontend="
      << ToFeatureFrontendText(cfg.app.runtime.featureFrontend) << "\n";
  oss << "slam.perception_mode=" << ToSensorModeText(cfg.app.sensorMode)
      << "\n";
  oss << "slam.operation_mode="
      << smartdrone::core::domain::ToString(cfg.app.runtime.slamOperationMode)
      << "\n";
}

void AppendTbcConfig(std::ostringstream &oss, const UnifiedConfig &cfg) {
  oss << "slam.tbc_override_enabled="
      << BoolText(cfg.app.runtime.useCustomTbc) << "\n";
  oss << "slam.tbc_tx_m=" << cfg.app.runtime.tbcTx << "\n";
  oss << "slam.tbc_ty_m=" << cfg.app.runtime.tbcTy << "\n";
  oss << "slam.tbc_tz_m=" << cfg.app.runtime.tbcTz << "\n";
  oss << "slam.tbc_roll_deg=" << cfg.app.runtime.tbcRollDeg << "\n";
  oss << "slam.tbc_pitch_deg=" << cfg.app.runtime.tbcPitchDeg << "\n";
  oss << "slam.tbc_yaw_deg=" << cfg.app.runtime.tbcYawDeg << "\n";
}

void AppendOrbConfig(std::ostringstream &oss, const UnifiedConfig &cfg) {
  oss << "slam.orb_nfeatures=" << cfg.app.runtime.orbNFeatures << "\n";
  oss << "slam.orb_scale_factor=" << cfg.app.runtime.orbScaleFactor << "\n";
  oss << "slam.orb_nlevels=" << cfg.app.runtime.orbNLevels << "\n";
  oss << "slam.orb_ini_th_fast=" << cfg.app.runtime.orbIniThFAST << "\n";
  oss << "slam.orb_min_th_fast=" << cfg.app.runtime.orbMinThFAST << "\n";
}

void AppendVisualFeatureConfig(std::ostringstream &oss,
                               const UnifiedConfig &cfg) {
  oss << "slam.visual_feature_top_k=" << cfg.app.runtime.visualFeatureTopK
      << "\n";
  oss << "slam.visual_feature_max_points="
      << cfg.app.runtime.visualFeatureMaxPoints << "\n";
  oss << "slam.superpoint_top_k=" << cfg.app.runtime.visualFeatureTopK << "\n";
  oss << "slam.superpoint_max_points=" << cfg.app.runtime.visualFeatureMaxPoints
      << "\n";
  oss << "slam.visual_feature_input_max_width="
      << cfg.app.runtime.visualFeatureInputMaxWidth << "\n";
  oss << "slam.visual_feature_input_max_height="
      << cfg.app.runtime.visualFeatureInputMaxHeight << "\n";
  oss << "slam.superpoint_input_max_width="
      << cfg.app.runtime.visualFeatureInputMaxWidth << "\n";
  oss << "slam.superpoint_input_max_height="
      << cfg.app.runtime.visualFeatureInputMaxHeight << "\n";
  oss << "slam.lk_superpoint_seeding=false\n";
  oss << "slam.lk_per_frame_accel=" << cfg.app.runtime.lkPerFrameAcceleration
      << "\n";
  oss << "slam.orb_accel=" << cfg.app.runtime.orbAcceleration << "\n";
  oss << "slam.settings=" << cfg.app.settings << "\n";
}

void AppendStreamConfig(std::ostringstream &oss, const UnifiedConfig &cfg) {
  oss << "stream.udp_enabled=" << BoolText(cfg.app.udp.enable) << "\n";
  oss << "stream.udp_ip=" << cfg.app.udp.ip << "\n";
  oss << "stream.send_image=" << BoolText(cfg.app.udp.sendImage) << "\n";
  oss << "stream.send_feature=" << BoolText(cfg.app.udp.sendFeature) << "\n";
  oss << "stream.send_map=" << BoolText(cfg.app.udp.sendMap) << "\n";
}

} // namespace

std::vector<uint8_t> BuildCapabilitiesPayload() {
  const auto capabilities = CapabilityCatalog::BuildDefault();
  std::vector<std::string> runtimeModes;
  for (const auto mode : capabilities.runtimeModes)
    runtimeModes.emplace_back(smartdrone::core::domain::ToString(mode));
  std::vector<std::string> perceptionModes;
  for (const auto mode : capabilities.perceptionModes)
    perceptionModes.emplace_back(smartdrone::core::domain::ToString(mode));
  std::vector<std::string> slamModes;
  for (const auto mode : capabilities.slamModes)
    slamModes.emplace_back(smartdrone::core::domain::ToString(mode));
  std::vector<std::string> configKeys;
  configKeys.reserve(capabilities.configKeys.size());
  for (const auto &item : capabilities.configKeys)
    configKeys.push_back(item.key);

  std::ostringstream oss;
  oss << "runtime_modes=" << JoinStrings(runtimeModes, ",") << "\n";
  oss << "perception_modes=" << JoinStrings(perceptionModes, ",") << "\n";
  oss << "slam_modes=" << JoinStrings(slamModes, ",") << "\n";
  oss << "camera_providers=" << JoinStrings(capabilities.cameraProviders, ",")
      << "\n";
  oss << "imu_providers=" << JoinStrings(capabilities.imuProviders, ",")
      << "\n";
  oss << "slam_engines=" << JoinStrings(capabilities.slamEngines, ",") << "\n";
  oss << "command_channels=" << JoinStrings(capabilities.commandChannels, ",")
      << "\n";
  oss << "behavior_notes=" << JoinStrings(capabilities.behaviorNotes, ";")
      << "\n";
  oss << "config_keys=" << JoinStrings(configKeys, ",") << "\n";
  return TextPayloadFromString(oss.str());
}

std::vector<uint8_t>
BuildConfigPayload(const UnifiedConfig &cfg,
                   smartdrone::core::domain::RuntimeMode runtimeMode) {
  std::ostringstream oss;
  oss << "runtime.mode=" << smartdrone::core::domain::ToString(runtimeMode)
      << "\n";
  AppendCameraConfig(oss, cfg);
  AppendSlamModeConfig(oss, cfg);
  AppendTbcConfig(oss, cfg);
  AppendOrbConfig(oss, cfg);
  AppendVisualFeatureConfig(oss, cfg);
  AppendStreamConfig(oss, cfg);
  return TextPayloadFromString(oss.str());
}

} // namespace smartdrone::core::application
