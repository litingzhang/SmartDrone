#include "core/application/config/capability_catalog.h"

namespace smartdrone::core::application {

domain::RuntimeCapabilities CapabilityCatalog::BuildDefault()
{
    domain::RuntimeCapabilities capabilities{};
    capabilities.runtimeModes = {
        domain::RuntimeMode::Idle,
        domain::RuntimeMode::Slam,
        domain::RuntimeMode::Calib,
    };
    capabilities.perceptionModes = {
        domain::PerceptionMode::Stereo,
        domain::PerceptionMode::StereoImu,
        domain::PerceptionMode::Mono,
        domain::PerceptionMode::MonoImu,
    };
    capabilities.slamModes = {
        domain::SlamOperationMode::Mapping,
        domain::SlamOperationMode::Localization,
        domain::SlamOperationMode::Auto,
    };
    capabilities.cameraProviders = {"libcamera_stereo_ov9281"};
    capabilities.imuProviders = {"icm42688_spi"};
    capabilities.slamEngines = {"orbslam3"};
    capabilities.commandChannels = {"udp_tlv"};
    capabilities.behaviorNotes = {
        "slam_mode.relocalization=maps_to_orbslam3_localization_only",
        "slam_mode.tracking_only=maps_to_orbslam3_localization_only",
        "slam_mode.auto=runtime_adaptive_switch_between_mapping_and_localization",
    };
    capabilities.configKeys = ConfigRegistry::DefaultDescriptors();
    return capabilities;
}

} // namespace smartdrone::core::application
