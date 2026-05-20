#pragma once

#include <string>

namespace smartdrone::core::application {

struct UnifiedConfig;

std::string BuildEffectiveSlamSettingsPath(const UnifiedConfig &cfg);
void ApplyOrbAccelerationEnvironment(const std::string &acceleration);

} // namespace smartdrone::core::application
