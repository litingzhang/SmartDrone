#pragma once

#include "common/epg/epg_types.h"

namespace SmartDrone::Core::Application {

struct EpgTaskManifest;

bool EpgSolverReportUsesGlobalObjective(const Epg::SolverReport &report);

void ValidateEpgSolverReportGlobalOptimum(
    const EpgTaskManifest &manifest,
    const Epg::OptimizedGraph &optimizedGraph,
    const Epg::SolverReport &report,
    const Epg::GraphProfile *sourceProfile);

} // namespace SmartDrone::Core::Application
