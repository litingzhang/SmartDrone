#include "core/application/session/epg/slam_session_task_utils.h"

#include <algorithm>
#include <string>
#include <vector>

#include "core/application/config/runtime_app_types.h"
#include "core/application/epg/epg_task_manifest.h"
#include "core/application/runtime/runtime_aliases.h"

namespace SmartDrone::Core::Application {
namespace {

const std::vector<EpgTaskRuntimeTuningEntry> SLAM_RUNTIME_TUNING{
    {"SlamResourceTask", true, false, false},
    {"SlamClockTask", true, false, false},
    {"SlamImuPollTask", false, true, true},
};

void OverrideTaskInterval(Epg::GraphConfig &config, const std::string &taskName,
                          std::chrono::milliseconds interval)
{
    for (auto &task : config.tasks) {
        if (task.name == taskName) {
            task.trigger.interval = interval;
            return;
        }
    }
}

void OverrideTaskScheduling(Epg::GraphConfig &config, const std::string &taskName,
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

void ApplySlamRuntimePacing(Epg::GraphConfig &config, const UnifiedConfig &cfg)
{
    ValidateEpgTaskRuntimeTuning(EpgManifestForDomain(EpgDomain::SlamSession),
                                 config, SLAM_RUNTIME_TUNING);
    OverrideTaskInterval(config, "SlamResourceTask", SLAM_RESOURCE_POLL_INTERVAL);
    OverrideTaskInterval(config, "SlamClockTask",
                         SlamInputInterval(cfg.app.runtime.slamInputFps,
                                           cfg.app.camera.fps));
    OverrideTaskScheduling(config, "SlamImuPollTask", cfg.app.imu.rtImu,
                           cfg.app.imu.rtPrio);
}

} // namespace SmartDrone::Core::Application
