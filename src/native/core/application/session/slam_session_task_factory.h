#pragma once

#include <atomic>
#include <memory>

#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/epg_dfx_snapshot.h"
#include "core/application/session/epg_registry.h"
#include "core/application/session/slam_session_runtime_service.h"

namespace smartdrone::core::application {

struct SlamTaskFactoryDeps {
    std::shared_ptr<SlamSessionRuntimeService> service;
    std::atomic<bool> &stop;
    std::atomic<bool> &runningFlag;
    std::atomic<bool> &sessionOk;
    LiveRuntimeTuning &tuning;
    int cameraFps;
    std::shared_ptr<EpgGraphRef> graphRef;
};

EpgTaskFactoryResolver MakeSlamGraphTaskFactoryResolver(
    SlamTaskFactoryDeps deps);

} // namespace smartdrone::core::application
