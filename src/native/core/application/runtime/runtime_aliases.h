#pragma once

#include "core/application/config/runtime_app_types.h"
#include "core/domain/runtime_mode.h"

namespace smartdrone::core::application {

int ClampSlamInputFps(int requestedFps, int cameraFps);
MainRuntimeAliases BuildRuntimeAliases(const AppConfig &config);
void PrintStartupConfig(const AppConfig &app, const MainRuntimeAliases &aliases,
                        smartdrone::core::domain::RuntimeMode mode);

} // namespace smartdrone::core::application
