#pragma once

#include <string>

#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/runtime_command_result.h"
#include "core/application/runtime/runtime_config_update.h"

namespace SmartDrone::Core::Application {

CommandResult ApplyConfigValue(const std::string &key,
                               const ConfigValue &value,
                               RemoteRuntimeConfig &remote);

} // namespace SmartDrone::Core::Application
