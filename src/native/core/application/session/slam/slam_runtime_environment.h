#pragma once

#include <string>

namespace SmartDrone::Core::Application {

struct UnifiedConfig;

std::string BuildEffectiveSlamSettingsPath(const UnifiedConfig &cfg);
void ApplyOrbAccelerationEnvironment(const std::string &acceleration);

} // namespace SmartDrone::Core::Application
