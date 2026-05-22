#pragma once

#include <atomic>
#include <memory>

#include "core/application/session/calib/calib_runtime_state.h"
#include "core/application/epg/epg_registry.h"

namespace SmartDrone::Core::Application {

struct EpgGraphRef;

struct CalibTaskFactoryDeps {
    std::shared_ptr<CalibRuntimeState> state;
    std::atomic<bool> &stop;
    std::atomic<bool> &runningFlag;
    std::atomic<bool> &sessionOk;
    std::atomic<bool> &completed;
    std::shared_ptr<EpgGraphRef> graphRef;
};

EpgTaskFactoryResolver MakeCalibGraphTaskFactoryResolver(
    CalibTaskFactoryDeps deps);

} // namespace SmartDrone::Core::Application
