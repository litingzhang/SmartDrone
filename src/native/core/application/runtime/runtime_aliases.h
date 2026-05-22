#pragma once

#include "core/application/config/runtime_app_types.h"
#include "core/domain/runtime_mode.h"

namespace SmartDrone::core::application {

struct CameraRuntimeProviderMetadata;

int ClampSlamInputFps(int requestedFps, int cameraFps);
MainRuntimeAliases BuildRuntimeAliases(const AppConfig &config);
void PrintStartupConfig(const AppConfig &app, const MainRuntimeAliases &aliases,
                        const CameraRuntimeProviderMetadata &cameraProvider,
                        SmartDrone::core::domain::RuntimeMode mode);

} // namespace SmartDrone::core::application
