#pragma once

#include "config_registry.hpp"
#include "core/domain/capabilities.hpp"

namespace smartdrone::core::application {

class CapabilityCatalog {
public:
    static domain::RuntimeCapabilities BuildDefault()
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
        capabilities.configKeys = ConfigRegistry::DefaultDescriptors();
        return capabilities;
    }
};

}  // namespace smartdrone::core::application
