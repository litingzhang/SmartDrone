#pragma once

#include <string>

namespace SmartDrone::core::application {

struct CameraRuntimeProviderMetadata {
    std::string providerName;
    bool usesPackedStereo{false};
};

} // namespace SmartDrone::core::application
