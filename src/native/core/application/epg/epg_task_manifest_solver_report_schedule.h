#pragma once

#include "common/epg/epg_types.h"

namespace SmartDrone::Core::Application {

void ValidateEpgSolverReportCpuBindingSchedule(
    const Epg::GraphConfig &graphConfig,
    const Epg::SolverReport &report);

void ValidateEpgSolverReportTopologySchedule(
    const Epg::GraphConfig &graphConfig,
    const Epg::SolverReport &report);

} // namespace SmartDrone::Core::Application
