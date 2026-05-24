#pragma once

#include "common/epg/epg_types.h"

namespace SmartDrone::Core::Application {

struct EpgTaskManifest;

void ValidateEpgQueueGlobalOptimum(
    const EpgTaskManifest &manifest,
    const Epg::SolverReport &report,
    const Epg::QueueConfig &queue,
    const Epg::QueueProfileMetrics *profileStats);

} // namespace SmartDrone::Core::Application
