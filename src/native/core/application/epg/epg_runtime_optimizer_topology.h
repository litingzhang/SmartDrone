#pragma once

#include "common/epg/epg_types.h"
#include "core/application/epg/epg_solver_primitives.h"

#include <vector>

namespace SmartDrone::Core::Application {

void ApplyEpgOptimizerTaskTopologySchedule(
    Epg::GraphConfig &config,
    std::vector<Epg::SolverReportDecision> &decisions,
    const EpgSolverPrimitives::TaskTopologySchedule &schedule);

} // namespace SmartDrone::Core::Application
