#include "core/application/config/capability_catalog.h"
#include "core/application/sensors/camera_runtime_provider.h"

namespace smartdrone::core::application {
namespace {

std::vector<domain::PerceptionMode> DefaultPerceptionModes() {
  if (CompiledCameraProviderUsesPackedStereo()) {
    return {
        domain::PerceptionMode::Stereo,
    };
  }
  return {
      domain::PerceptionMode::Stereo,
      domain::PerceptionMode::StereoImu,
      domain::PerceptionMode::Mono,
      domain::PerceptionMode::MonoImu,
  };
}

std::vector<std::string> DefaultSlamEngines() {
  std::vector<std::string> engines{"klt", "dpvo_tensorrt"};
#if defined(SMART_DRONE_ENABLE_ORB_SLAM3)
  engines.push_back("orbslam3");
#endif
  return engines;
}

std::vector<std::string> DefaultBehaviorNotes(
    std::string_view compiledCameraProvider) {
  std::vector<std::string> notes{
      std::string("camera.provider.compiled=") +
          std::string(compiledCameraProvider),
      "slam_mode.relocalization=maps_to_backend_localization_only",
      "slam_mode.tracking_only=maps_to_backend_localization_only",
      "slam_mode.auto=runtime_adaptive_switch_between_mapping_and_localization",
      "slam.backend.klt=native_cpp_klt_pnp_visual_odometry_backend",
      "slam.backend.dpvo_tensorrt=native_cpp_tensorrt_dpvo_backend",
      "slam.feature_frontend.lk=grid_lk_pnp_vo",
      "slam.feature_frontend.lk_gftt_per_frame=klt_tracking_pnp_vo",
#if defined(SMART_DRONE_ENABLE_ORB_SLAM3)
      "slam.backend.orbslam3=orb_slam3_backend_with_selectable_frontends",
      "slam.feature_frontend.orb=legacy_orb_slam3_tracking_path",
      "slam.feature_frontend.superpoint_lightglue=tensorrt_cpp_superpoint_"
      "stereo_injection",
      "slam.feature_frontend.xfeat_lightglue=visual_feature_lightglue_slot_"
      "pending_native_client",
#else
      "slam.feature_frontend.orb=disabled_at_build_time",
      "slam.feature_frontend.superpoint_lightglue=disabled_at_build_time",
      "slam.feature_frontend.xfeat_lightglue=disabled_at_build_time",
#endif
      "slam.lk_seed.gftt=shi_tomasi_good_features_to_track",
  };
  if (CompiledCameraProviderUsesPackedStereo()) {
    notes.push_back(
        "camera.uvc_stereo_v4l2=single_uvc_device_packed_left_right_frame");
    notes.push_back("camera.uvc_timestamp=v4l2_buffer_timestamp_from_vidioc_dqbuf");
    notes.push_back(
        "camera.uvc_pairing=not_required_single_capture_provides_both_eyes");
  }
  return notes;
}

void PopulateStaticModes(domain::RuntimeCapabilities &capabilities) {
  capabilities.runtimeModes = {
      domain::RuntimeMode::Idle,
      domain::RuntimeMode::Slam,
      domain::RuntimeMode::Calib,
  };
  capabilities.slamModes = {
      domain::SlamOperationMode::Mapping,
      domain::SlamOperationMode::Localization,
      domain::SlamOperationMode::Auto,
  };
}

void PopulateProviders(domain::RuntimeCapabilities &capabilities,
                       std::string_view compiledCameraProvider) {
  capabilities.cameraProviders = {std::string(compiledCameraProvider)};
  capabilities.imuProviders = {"icm42688_spi"};
  capabilities.commandChannels = {"udp_tlv"};
}

} // namespace

domain::RuntimeCapabilities CapabilityCatalog::BuildDefault() {
  domain::RuntimeCapabilities capabilities{};
  const std::string_view compiledCameraProvider = CompiledCameraProviderName();
  PopulateStaticModes(capabilities);
  PopulateProviders(capabilities, compiledCameraProvider);
  capabilities.perceptionModes = DefaultPerceptionModes();
  capabilities.slamEngines = DefaultSlamEngines();
  capabilities.behaviorNotes = DefaultBehaviorNotes(compiledCameraProvider);
  capabilities.configKeys = ConfigRegistry::DefaultDescriptors();
  return capabilities;
}

} // namespace smartdrone::core::application
