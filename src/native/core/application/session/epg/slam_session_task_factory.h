#pragma once

#include <atomic>
#include <memory>

#include "core/application/epg/epg_registry.h"

namespace smartdrone::core::application {

struct EpgGraphRef;
struct LiveRuntimeTuning;
class SlamSessionRuntimeService;

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
