#include "core/application/config/capability_catalog.h"
#include "core/application/config/slam_backend_availability.h"
#include "core/application/runtime/runtime_provider_metadata.h"

namespace SmartDrone::Core::Application {
namespace {

std::vector<Domain::PerceptionMode> DefaultPerceptionModes(
    const CameraRuntimeProviderMetadata &cameraProvider)
{
    if (cameraProvider.usesPackedStereo) {
        return {
            Domain::PerceptionMode::Stereo,
        };
    }
    return {
        Domain::PerceptionMode::Stereo,
        Domain::PerceptionMode::StereoImu,
        Domain::PerceptionMode::Mono,
        Domain::PerceptionMode::MonoImu,
    };
}

std::vector<std::string> DefaultSlamEngines()
{
    std::vector<std::string> engines{"klt", "dpvo_tensorrt"};
    if (OrbSlam3BackendAvailable()) {
        engines.push_back("orbslam3");
    }
    return engines;
}

std::vector<std::string> DefaultBehaviorNotes(
    const CameraRuntimeProviderMetadata &cameraProvider)
{
    const bool orbAvailable = OrbSlam3BackendAvailable();
    std::vector<std::string> notes{
        std::string("camera.provider.compiled=") +
            cameraProvider.providerName,
        "slam_mode.relocalization=maps_to_backend_localization_only",
        "slam_mode.tracking_only=maps_to_backend_localization_only",
        "slam_mode.auto=runtime_adaptive_switch_between_mapping_and_localization",
        "slam.backend.klt=native_cpp_klt_pnp_visual_odometry_backend",
        "slam.backend.dpvo_tensorrt=native_cpp_tensorrt_dpvo_backend",
        "slam.feature_frontend.lk=grid_lk_pnp_vo",
        "slam.feature_frontend.lk_gftt_per_frame=klt_tracking_pnp_vo",
        "slam.lk_seed.gftt=shi_tomasi_good_features_to_track",
    };
    if (orbAvailable) {
        notes.push_back(
            "slam.backend.orbslam3=orb_slam3_backend_with_selectable_frontends");
        notes.push_back("slam.feature_frontend.orb=legacy_orb_slam3_tracking_path");
        notes.push_back("slam.feature_frontend.superpoint_lightglue="
                        "tensorrt_cpp_superpoint_stereo_injection");
        notes.push_back("slam.feature_frontend.xfeat_lightglue="
                        "visual_feature_lightglue_slot_pending_native_client");
    } else {
        notes.push_back("slam.feature_frontend.orb=disabled_at_build_time");
        notes.push_back(
            "slam.feature_frontend.superpoint_lightglue=disabled_at_build_time");
        notes.push_back("slam.feature_frontend.xfeat_lightglue=disabled_at_build_time");
    }
    if (cameraProvider.usesPackedStereo) {
        notes.push_back(
            "camera.uvc_stereo_v4l2=single_uvc_device_packed_left_right_frame");
        notes.push_back("camera.uvc_timestamp=v4l2_buffer_timestamp_from_vidioc_dqbuf");
        notes.push_back(
            "camera.uvc_pairing=not_required_single_capture_provides_both_eyes");
    }
    return notes;
}

void PopulateStaticModes(Domain::RuntimeCapabilities &capabilities)
{
    capabilities.runtimeModes = {
        Domain::RuntimeMode::Idle,
        Domain::RuntimeMode::Slam,
        Domain::RuntimeMode::Calib,
    };
    capabilities.slamModes = {
        Domain::SlamOperationMode::Mapping,
        Domain::SlamOperationMode::Localization,
        Domain::SlamOperationMode::Auto,
    };
}

void PopulateProviders(Domain::RuntimeCapabilities &capabilities,
                       const CameraRuntimeProviderMetadata &cameraProvider)
{
    capabilities.cameraProviders = {cameraProvider.providerName};
    capabilities.imuProviders = {"icm42688_spi"};
    capabilities.commandChannels = {"udp_tlv"};
}

} // namespace

Domain::RuntimeCapabilities CapabilityCatalog::BuildDefault(
    const CameraRuntimeProviderMetadata &cameraProvider)
{
    Domain::RuntimeCapabilities capabilities{};
    PopulateStaticModes(capabilities);
    PopulateProviders(capabilities, cameraProvider);
    capabilities.perceptionModes = DefaultPerceptionModes(cameraProvider);
    capabilities.slamEngines = DefaultSlamEngines();
    capabilities.behaviorNotes = DefaultBehaviorNotes(cameraProvider);
    capabilities.configKeys = ConfigRegistry::DefaultDescriptors();
    return capabilities;
}

} // namespace SmartDrone::Core::Application
