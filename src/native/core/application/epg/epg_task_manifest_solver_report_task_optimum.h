#pragma once

#include "common/epg/epg_types.h"

namespace SmartDrone::Core::Application {

struct EpgTaskManifest;

struct EpgTaskGlobalOptimumContext {
    const EpgTaskManifest *manifest{nullptr};
    const Epg::SolverReport *report{nullptr};
    const Epg::GraphConfig *sourceGraphConfig{nullptr};
    const Epg::GraphProfileDiagnostics *diagnostics{nullptr};
    const Epg::TaskConfig *optimizedTask{nullptr};
    const Epg::TaskProfileMetrics *profileStats{nullptr};
};

void ValidateEpgTaskGlobalOptimum(
    const EpgTaskGlobalOptimumContext &context);

} // namespace SmartDrone::Core::Application
