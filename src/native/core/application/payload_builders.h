#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/application/runtime_app_types.h"
#include "core/domain/runtime_mode.h"

namespace smartdrone::core::application {

std::vector<uint8_t> TextPayloadFromString(const std::string& text);
std::string JoinStrings(const std::vector<std::string>& values, const char* sep);
std::vector<uint8_t> BuildCapabilitiesPayload();
std::vector<uint8_t> BuildConfigPayload(const UnifiedConfig& cfg, smartdrone::core::domain::RuntimeMode runtimeMode);

}  // namespace smartdrone::core::application
