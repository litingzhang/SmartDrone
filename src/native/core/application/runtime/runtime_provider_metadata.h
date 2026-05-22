#pragma once

#include <string>

namespace SmartDrone::Core::Application {

struct CameraRuntimeProviderMetadata {
    std::string providerName;
    bool usesPackedStereo{false};
};

} // namespace SmartDrone::Core::Application
