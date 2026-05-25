#pragma once

#include <chrono>

#include "common/epg/epg.h"

namespace SmartDrone::Core::Application {

struct UnifiedConfig;

constexpr std::chrono::milliseconds SLAM_RESOURCE_POLL_INTERVAL{100};

std::chrono::milliseconds SlamInputInterval(int slamInputFps, int cameraFps);
void ApplySlamRuntimePacing(Epg::GraphConfig &config, const UnifiedConfig &cfg);

} // namespace SmartDrone::Core::Application
