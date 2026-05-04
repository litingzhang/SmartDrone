#include "core/application/config/capability_catalog.h"
#include "core/application/session/sensor_runtime_helpers.h"

namespace smartdrone::core::application {

domain::RuntimeCapabilities CapabilityCatalog::BuildDefault()
{
    domain::RuntimeCapabilities capabilities{};
    const std::string_view compiledCameraProvider = CompiledCameraProviderName();
    capabilities.runtimeModes = {
        domain::RuntimeMode::Idle,
        domain::RuntimeMode::Slam,
        domain::RuntimeMode::Calib,
    };
    if (CompiledCameraProviderUsesPackedStereo()) {
        capabilities.perceptionModes = {
            domain::PerceptionMode::Stereo,
        };
    } else {
        capabilities.perceptionModes = {
            domain::PerceptionMode::Stereo,
            domain::PerceptionMode::StereoImu,
            domain::PerceptionMode::Mono,
            domain::PerceptionMode::MonoImu,
        };
    }
    capabilities.slamModes = {
        domain::SlamOperationMode::Mapping,
        domain::SlamOperationMode::Localization,
        domain::SlamOperationMode::Auto,
    };
    capabilities.cameraProviders = {std::string(compiledCameraProvider)};
    capabilities.imuProviders = {"icm42688_spi"};
    capabilities.slamEngines = {"slam_adapter"};
    capabilities.commandChannels = {"udp_tlv"};
    capabilities.behaviorNotes = {
        std::string("camera.provider.compiled=") + std::string(compiledCameraProvider),
        "slam_mode.relocalization=maps_to_backend_localization_only",
        "slam_mode.tracking_only=maps_to_backend_localization_only",
        "slam_mode.auto=runtime_adaptive_switch_between_mapping_and_localization",
        "slam.feature_frontend.orb=full_slam_tracking_path",
        "slam.feature_frontend.lk=grid_lk_pnp_vo",
        "slam.feature_frontend.lk_gftt_per_frame=klt_tracking_pnp_vo",
        "slam.feature_frontend.superpoint_lightglue=tensorrt_cpp_superpoint_stereo_injection",
        "slam.lk_seed.gftt=shi_tomasi_good_features_to_track",
    };
    if (CompiledCameraProviderUsesPackedStereo()) {
        capabilities.behaviorNotes.push_back("camera.uvc_stereo_v4l2=single_uvc_device_packed_left_right_frame");
        capabilities.behaviorNotes.push_back("camera.uvc_timestamp=v4l2_buffer_timestamp_from_vidioc_dqbuf");
        capabilities.behaviorNotes.push_back("camera.uvc_pairing=not_required_single_capture_provides_both_eyes");
    }
    capabilities.configKeys = ConfigRegistry::DefaultDescriptors();
    return capabilities;
}

} // namespace smartdrone::core::application
