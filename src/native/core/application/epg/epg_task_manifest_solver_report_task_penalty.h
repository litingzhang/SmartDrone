#pragma once

#include "common/epg/epg_types.h"

namespace SmartDrone::Core::Application {

struct EpgTaskCatalogEntry;

struct EpgTaskOptimumPenaltyContext {
    const Epg::SolverReportConstraints *constraints{nullptr};
    const Epg::GraphConfig *sourceGraphConfig{nullptr};
    const Epg::GraphProfileDiagnostics *diagnostics{nullptr};
    const Epg::TaskConfig *task{nullptr};
    const Epg::TaskProfileMetrics *stats{nullptr};
    const Epg::SolverReportDecision *decision{nullptr};
    const EpgTaskCatalogEntry *catalog{nullptr};
};

std::uint64_t BuildEpgTaskActualPenalty(
    const EpgTaskOptimumPenaltyContext &context);

std::uint64_t BuildEpgTaskBestPenalty(
    const EpgTaskOptimumPenaltyContext &context);

std::uint64_t BuildEpgTaskTopologyPenalty(
    const EpgTaskOptimumPenaltyContext &context);

} // namespace SmartDrone::Core::Application
