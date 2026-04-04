#pragma once

#include <string>
#include <vector>

#include "runtime_mode.h"

namespace smartdrone::core::domain {

struct ConfigDescriptor {
    std::string key;
    std::string description;
    bool hotReloadable{false};
    bool requiresPipelineRestart{false};
    bool requiresDeviceRestart{false};
};

struct RuntimeCapabilities {
    std::vector<RuntimeMode> runtimeModes;
    std::vector<PerceptionMode> perceptionModes;
    std::vector<SlamOperationMode> slamModes;
    std::vector<std::string> cameraProviders;
    std::vector<std::string> imuProviders;
    std::vector<std::string> slamEngines;
    std::vector<std::string> commandChannels;
    std::vector<ConfigDescriptor> configKeys;
};

} // namespace smartdrone::core::domain
