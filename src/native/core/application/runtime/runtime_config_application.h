#pragma once

#include "core/application/config/runtime_app_types.h"

namespace SmartDrone::Core::Application {

struct RuntimeTbcValues {
    float tx{0.0f};
    float ty{0.0f};
    float tz{0.0f};
    float rollDeg{0.0f};
    float pitchDeg{0.0f};
    float yawDeg{0.0f};
};

struct AppliedRuntimeConfig {
    RuntimeTbcValues tbc{};
    bool restartNeeded{false};
};

AppliedRuntimeConfig ApplyRemoteRuntimeConfig(UnifiedConfig &config,
                                              const RemoteRuntimeConfig &remote);
void SyncRuntimeTuning(LiveRuntimeTuning &tuning,
                       const RemoteRuntimeConfig &remote,
                       const RuntimeTbcValues &tbc);

} // namespace SmartDrone::Core::Application
