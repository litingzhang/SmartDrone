#pragma once

#include "common/epg/epg_types.h"

namespace SmartDrone::Core::Application {

struct EpgTaskManifest;

void ValidateEpgTaskSolverReportDecision(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig *sourceGraphConfig,
    const Epg::GraphConfig &graphConfig,
    const Epg::SolverReportConstraints &constraints,
    const Epg::SolverReportDecision &decision);

} // namespace SmartDrone::Core::Application
