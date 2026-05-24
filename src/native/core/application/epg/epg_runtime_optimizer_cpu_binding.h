#pragma once

#include "common/epg/epg_types.h"
#include "core/application/epg/epg_solver_primitives.h"

#include <vector>

namespace SmartDrone::Core::Application {

void ApplyEpgOptimizerCpuBindingSchedule(
    Epg::GraphConfig &config,
    std::vector<Epg::SolverReportDecision> &decisions,
    const EpgSolverPrimitives::CpuBindingSchedule &schedule);

} // namespace SmartDrone::Core::Application
