#pragma once

#include "core/application/config/runtime_app_types.h"

namespace SmartDrone::Core::Application {

RemoteRuntimeConfig BuildRemoteConfig(const UnifiedConfig &currentConfig);

} // namespace SmartDrone::Core::Application
