#include "core/application/session/slam_session_task_utils.h"

#include <algorithm>
#include <string>

#include "core/application/runtime/runtime_aliases.h"

namespace smartdrone::core::application {
namespace {

void OverrideTaskInterval(epg::GraphConfig &config, const std::string &taskName,
                          std::chrono::milliseconds interval)
{
    for (auto &task : config.tasks) {
        if (task.name == taskName) {
            task.trigger.interval = interval;
            return;
        }
    }
}

void OverrideTaskScheduling(epg::GraphConfig &config, const std::string &taskName,
                            bool realtime, int priority)
{
    for (auto &task : config.tasks) {
        if (task.name == taskName) {
            task.scheduling.realtime = realtime;
            task.scheduling.priority = priority;
            return;
        }
    }
}

} // namespace

std::chrono::milliseconds SlamInputInterval(int slamInputFps, int cameraFps)
{
    const int clampedFps = ClampSlamInputFps(slamInputFps, cameraFps);
    const int intervalMs = std::max(1, (1000 + clampedFps - 1) / clampedFps);
    return std::chrono::milliseconds(intervalMs);
}

void ApplySlamRuntimePacing(epg::GraphConfig &config, const UnifiedConfig &cfg)
{
    OverrideTaskInterval(config, "SlamResourceTask", kSlamResourcePollInterval);
    OverrideTaskInterval(config, "SlamClockTask",
                         SlamInputInterval(cfg.app.runtime.slamInputFps,
                                           cfg.app.camera.fps));
    OverrideTaskScheduling(config, "SlamImuPollTask", cfg.app.imu.rtImu,
                           cfg.app.imu.rtPrio);
}

} // namespace smartdrone::core::application
