#pragma once

#include "common/epg/epg_types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace SmartDrone::Core::Application {
struct EpgTaskCatalogEntry;
}

namespace SmartDrone::Core::Application::EpgOptimizerPlanInternal {

using SolverDecision = Epg::SolverReportDecision;

struct QueueCandidate {
    std::uint64_t depth{0}, pressureAfter{0}, penalty{0};
};

struct TaskCandidate {
    std::uint64_t intervalMs{0};
    std::string resource;
    int cpuAffinity{-1};
    std::vector<Epg::PortId> backpressureOutputs;
    std::uint64_t predictedResourceWaitUs{0};
    std::uint64_t topologyPenalty{0}, penalty{0};
};

struct QueueSolverNode {
    std::size_t index{0};
    std::uint64_t depthBefore{0}, pressureBefore{0};
    Epg::QueueProfileMetrics stats;
    std::vector<QueueCandidate> candidates;
};

struct TaskSolverNode {
    std::size_t index{0};
    std::uint64_t intervalBeforeMs{0}, effectiveLoopUs{0};
    const EpgTaskCatalogEntry *catalog{nullptr};
    Epg::TaskProfileMetrics stats;
    std::vector<TaskCandidate> candidates;
};

struct TaskSolverNodeBuildContext {
    std::size_t index{0};
    const Epg::GraphConfig *config{nullptr};
    const Epg::GraphProfileDiagnostics *diagnostics{nullptr};
    const Epg::TaskConfig *task{nullptr};
    const Epg::TaskProfileMetrics *stats{nullptr};
    const EpgTaskCatalogEntry *catalog{nullptr};
};

struct TaskCandidateBuildContext {
    const Epg::GraphConfig *config{nullptr};
    const Epg::GraphProfileDiagnostics *diagnostics{nullptr};
    const Epg::TaskConfig *task{nullptr};
    const Epg::TaskProfileMetrics *stats{nullptr};
    const EpgTaskCatalogEntry *catalog{nullptr};
    std::uint64_t effectiveLoopUs{0};
    std::uint64_t intervalBeforeMs{0};
    std::uint64_t maxIntervalMs{0};
    bool replaceable{false};
    std::vector<Epg::PortId> backpressureBefore;
    std::vector<Epg::PortId> backpressureOptimized;
    std::vector<std::string> resourceCandidates;
};

struct TaskSolutionContext {
    const Epg::TaskConfig *task{nullptr};
    const TaskSolverNode *node{nullptr};
    const TaskCandidate *candidate{nullptr};
    std::uint64_t intervalAfterMs{0};
    int cpuAffinityBefore{-1};
    int cpuAffinityAfter{-1};
    std::string resourceBefore;
    std::string resourceAfter;
    std::vector<Epg::PortId> backpressureBefore;
    std::vector<Epg::PortId> backpressureAfter;
};

struct TopologySolution {
    std::vector<std::size_t> queueCandidateIndexes, taskCandidateIndexes;
};

} // namespace SmartDrone::Core::Application::EpgOptimizerPlanInternal
