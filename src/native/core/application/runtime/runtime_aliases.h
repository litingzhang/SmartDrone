#pragma once

#include "core/application/config/runtime_app_types.h"
#include "core/domain/runtime_mode.h"

namespace SmartDrone::Core::Application {

struct CameraRuntimeProviderMetadata;

int ClampSlamInputFps(int requestedFps, int cameraFps);
MainRuntimeAliases BuildRuntimeAliases(const AppConfig &config);
void PrintStartupConfig(const AppConfig &app, const MainRuntimeAliases &aliases,
                        const CameraRuntimeProviderMetadata &cameraProvider,
                        SmartDrone::Core::Domain::RuntimeMode mode);

} // namespace SmartDrone::Core::Application
