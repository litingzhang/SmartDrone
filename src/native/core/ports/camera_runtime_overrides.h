#pragma once

#include <string>

namespace SmartDrone::Core::Ports {

struct CameraRuntimeOverrides {
    int width{0};
    int height{0};
    int fps{0};
    std::string settingsPath;

    bool Valid() const
    {
        return width > 0 && height > 0 && fps > 0 && !settingsPath.empty();
    }
};

} // namespace SmartDrone::Core::Ports
