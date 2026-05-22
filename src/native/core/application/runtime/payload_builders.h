#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/application/config/runtime_app_types.h"
#include "core/domain/runtime_mode.h"

namespace SmartDrone::core::application {

struct CameraRuntimeProviderMetadata;

std::vector<uint8_t> TextPayloadFromString(const std::string &text);
std::string JoinStrings(const std::vector<std::string> &values, const char *sep);
std::vector<uint8_t> BuildCapabilitiesPayload(
    const CameraRuntimeProviderMetadata &cameraProvider);
std::vector<uint8_t> BuildConfigPayload(
    const UnifiedConfig &cfg, SmartDrone::core::domain::RuntimeMode runtimeMode,
    const CameraRuntimeProviderMetadata &cameraProvider);

} // namespace SmartDrone::core::application
