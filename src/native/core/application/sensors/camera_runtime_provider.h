#pragma once

#include <memory>
#include <string_view>

#include "core/ports/camera_provider.h"

namespace smartdrone::core::application {

std::unique_ptr<smartdrone::core::ports::ICameraProvider>
CreateCameraProvider();
std::string_view CompiledCameraProviderName();
bool CompiledCameraProviderUsesPackedStereo();

} // namespace smartdrone::core::application
