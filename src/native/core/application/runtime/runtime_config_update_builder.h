#pragma once

#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/runtime_config_update.h"

namespace SmartDrone::Core::Application {

ConfigUpdate BuildRuntimeConfigUpdate(const RemoteRuntimeConfig &remote);

} // namespace SmartDrone::Core::Application
