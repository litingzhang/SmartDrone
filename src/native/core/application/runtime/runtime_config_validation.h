#pragma once

#include <string>

#include "core/application/config/runtime_app_types.h"

namespace SmartDrone::Core::Application {

void NormalizeRemoteRuntimeConfig(RemoteRuntimeConfig &remote);
bool ValidateRemoteRuntimeConfig(RemoteRuntimeConfig &remote,
                                 std::string *err);

} // namespace SmartDrone::Core::Application
