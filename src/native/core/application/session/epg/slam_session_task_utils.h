#pragma once

#include <chrono>

#include "common/epg/epg.h"

namespace SmartDrone::core::application {

struct UnifiedConfig;

constexpr std::chrono::milliseconds kSlamResourcePollInterval{100};

std::chrono::milliseconds SlamInputInterval(int slamInputFps, int cameraFps);
void ApplySlamRuntimePacing(Epg::GraphConfig &config, const UnifiedConfig &cfg);

} // namespace SmartDrone::core::application
