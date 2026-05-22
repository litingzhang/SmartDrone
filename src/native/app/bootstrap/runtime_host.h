#pragma once

#include <string>

#include "core/application/config/runtime_app_types.h"

namespace SmartDrone::app::bootstrap {

class RuntimeHost {
  public:
    int Run(const SmartDrone::core::application::UnifiedConfig &cfg, const std::string &autoModeText);
};

} // namespace SmartDrone::app::bootstrap
