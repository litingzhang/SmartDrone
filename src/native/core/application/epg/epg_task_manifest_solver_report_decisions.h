#pragma once

#include "common/epg/epg_types.h"

namespace SmartDrone::Core::Application {

struct EpgTaskManifest;

void ValidateEpgSolverReportScore(const Epg::SolverReport &report);

void ValidateEpgSolverReportDecisionCoverage(
    const Epg::GraphConfig &graphConfig,
    const Epg::SolverReport &report);

void ValidateEpgSolverReportDecisionDetails(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig *sourceGraphConfig,
    const Epg::GraphConfig &graphConfig,
    const Epg::SolverReport &report);

} // namespace SmartDrone::Core::Application
