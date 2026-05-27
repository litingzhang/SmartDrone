#include "adapters/slam/openvins/openvins_runtime_driver.h"

namespace SmartDrone::Adapters::Slam {

std::unique_ptr<OpenVinsRuntimeDriver>
CreateOpenVinsRuntimeDriver(const std::string &settingsPath)
{
    (void)settingsPath;
    return nullptr;
}

} // namespace SmartDrone::Adapters::Slam
