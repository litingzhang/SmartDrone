#pragma once

#include <memory>
#include <string_view>

#include "core/application/config/runtime_app_types.h"
#include "core/ports/camera_provider.h"

namespace SmartDrone::Adapters::Camera {

std::unique_ptr<SmartDrone::Core::Ports::ICameraProvider>
CreateCameraProvider();
SmartDrone::Core::Ports::CameraOpenConfig MakeCameraOpenConfig(
    const SmartDrone::Core::Application::MainRuntimeAliases &aliases);
std::string_view CompiledCameraProviderName();
bool CompiledCameraProviderUsesPackedStereo();

} // namespace SmartDrone::Adapters::Camera
