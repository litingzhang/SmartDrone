#pragma once

#include <memory>
#include <string_view>

#include "core/application/config/runtime_app_types.h"
#include "core/ports/camera_provider.h"

namespace SmartDrone::adapters::camera {

std::unique_ptr<SmartDrone::core::ports::ICameraProvider>
CreateCameraProvider();
SmartDrone::core::ports::CameraOpenConfig MakeCameraOpenConfig(
    const SmartDrone::core::application::MainRuntimeAliases &aliases);
std::string_view CompiledCameraProviderName();
bool CompiledCameraProviderUsesPackedStereo();

} // namespace SmartDrone::adapters::camera
