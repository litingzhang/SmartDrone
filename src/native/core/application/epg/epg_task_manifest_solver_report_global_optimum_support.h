#pragma once

#include "common/epg/epg_types.h"

#include <string>

namespace SmartDrone::Core::Application {

const Epg::SolverReportDecision &RequireEpgSolverReportDecision(
    const Epg::SolverReport &report,
    const std::string &kind,
    const std::string &name);

} // namespace SmartDrone::Core::Application
