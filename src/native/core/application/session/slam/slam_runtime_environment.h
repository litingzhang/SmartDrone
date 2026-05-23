#pragma once

#include <string>

namespace SmartDrone::Core::Application {

struct UnifiedConfig;

std::string BuildEffectiveSlamSettingsPath(const UnifiedConfig &cfg);

} // namespace SmartDrone::Core::Application
