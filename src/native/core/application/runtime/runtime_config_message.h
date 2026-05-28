#pragma once

#include <string>

#include "core/application/config/runtime_app_types.h"

namespace SmartDrone::Core::Application {

std::string BuildRuntimeConfigMessage(const RemoteRuntimeConfig &remote,
                                      const UnifiedConfig &currentConfig);

} // namespace SmartDrone::Core::Application
