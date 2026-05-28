#pragma once

#include <string>

#include "core/application/runtime/runtime_command_result.h"
#include "core/application/runtime/runtime_config_update.h"

namespace SmartDrone::Core::Application {

CommandResult OkResult();
CommandResult TypeMismatchResult(const char *key);
CommandResult AssignStrictInt(const ConfigValue &value, int &out,
                              const char *key);
CommandResult AssignNumericInt(const ConfigValue &value, int &out,
                               const char *key);
CommandResult AssignFloat(const ConfigValue &value, float &out,
                          const char *key);
CommandResult AssignBool(const ConfigValue &value, bool &out,
                         const char *key);
CommandResult AssignString(const ConfigValue &value, std::string &out,
                           const char *key);

} // namespace SmartDrone::Core::Application
