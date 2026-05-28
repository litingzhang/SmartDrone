#pragma once

#include "common/epg/epg_types.h"
#include "core/application/epg/epg_solver_primitives.h"

#include <set>
#include <string>
#include <vector>

namespace SmartDrone::Core::Application {

void ApplyEpgOptimizerCpuBindingSchedule(
    Epg::GraphConfig &config,
    std::vector<Epg::SolverReportDecision> &decisions,
    const EpgSolverPrimitives::CpuBindingSchedule &schedule,
    const std::set<std::string> &preserveAccuracyTasks);

} // namespace SmartDrone::Core::Application
