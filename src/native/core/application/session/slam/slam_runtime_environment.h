#pragma once

#include <string>

namespace SmartDrone::core::application {

struct UnifiedConfig;

std::string BuildEffectiveSlamSettingsPath(const UnifiedConfig &cfg);
void ApplyOrbAccelerationEnvironment(const std::string &acceleration);

} // namespace SmartDrone::core::application
