#pragma once

#include <string>

#include "core/application/config/runtime_app_types.h"

namespace smartdrone::app::bootstrap {

class RuntimeHost {
  public:
    int Run(const smartdrone::core::application::UnifiedConfig &cfg, const std::string &autoModeText);
};

} // namespace smartdrone::app::bootstrap
