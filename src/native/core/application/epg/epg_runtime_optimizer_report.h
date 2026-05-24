#pragma once

#include "common/epg/epg_types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace SmartDrone::Core::Application {

struct EpgTaskManifest;

std::string BuildEpgOptimizerSolverReport(
    const EpgTaskManifest &manifest,
    const Epg::GraphProfileMetadata &metadata,
    std::uint64_t nowMs,
    const std::vector<Epg::SolverReportDecision> &decisions);

} // namespace SmartDrone::Core::Application
