#pragma once

#include <string>

#include "core/application/config/runtime_app_types.h"

namespace smartdrone::core::application {

std::string BuildEffectiveSlamSettingsPath(const UnifiedConfig &cfg);
void ApplyOrbAccelerationEnvironment(const std::string &acceleration);

} // namespace smartdrone::core::application
