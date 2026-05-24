#pragma once

#include "common/epg/epg_types.h"

#include <iosfwd>
#include <vector>

namespace SmartDrone::Core::Application {

void WriteEpgOptimizerSolverReportDecisions(
    std::ostringstream &out,
    const std::vector<Epg::SolverReportDecision> &decisions);

} // namespace SmartDrone::Core::Application
