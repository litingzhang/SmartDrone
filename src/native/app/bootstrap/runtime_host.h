#pragma once

#include <string>

#include "core/application/config/runtime_app_types.h"

namespace SmartDrone::App::Bootstrap {

class RuntimeHost {
  public:
    int Run(const SmartDrone::Core::Application::UnifiedConfig &cfg, const std::string &autoModeText);
};

} // namespace SmartDrone::App::Bootstrap
