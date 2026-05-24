#pragma once

#include "common/epg/epg_types.h"

#include <vector>

namespace SmartDrone::Core::Application {

struct EpgTaskManifest;

struct EpgRuntimeOptimizerPlan {
    Epg::GraphConfig config;
    std::vector<Epg::SolverReportDecision> decisions;
};

EpgRuntimeOptimizerPlan BuildEpgOptimizerPlan(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig &profileTopology,
    const Epg::GraphProfileDiagnostics &diagnostics);

} // namespace SmartDrone::Core::Application
