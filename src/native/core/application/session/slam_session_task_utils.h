#pragma once

#include <chrono>

#include "common/epg/epg.h"
#include "core/application/config/runtime_app_types.h"

namespace smartdrone::core::application {

constexpr std::chrono::milliseconds kSlamResourcePollInterval{100};

std::chrono::milliseconds SlamInputInterval(int slamInputFps, int cameraFps);
void ApplySlamRuntimePacing(epg::GraphConfig &config, const UnifiedConfig &cfg);

} // namespace smartdrone::core::application
