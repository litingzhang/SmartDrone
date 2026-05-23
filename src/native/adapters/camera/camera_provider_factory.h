#pragma once

#include <memory>
#include <string_view>

#include "core/ports/camera_provider.h"

namespace SmartDrone::Adapters::Camera {

std::unique_ptr<SmartDrone::Core::Ports::ICameraProvider>
CreateCameraProvider();
std::string_view CompiledCameraProviderName();
bool CompiledCameraProviderUsesPackedStereo();

} // namespace SmartDrone::Adapters::Camera
