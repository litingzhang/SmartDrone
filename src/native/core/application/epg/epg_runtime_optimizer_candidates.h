#pragma once

#include "core/application/epg/epg_runtime_optimizer_plan_types.h"

#include <map>

namespace SmartDrone::Core::Application {

struct EpgTaskManifest;

namespace EpgOptimizerPlanInternal {

std::vector<QueueSolverNode> BuildQueueSolverNodes(
    const Epg::GraphConfig &config,
    const Epg::GraphProfileDiagnostics &diagnostics);

std::vector<TaskSolverNode> BuildTaskSolverNodes(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig &config,
    const Epg::GraphProfileDiagnostics &diagnostics);

TopologySolution SolveGlobalTopology(
    const std::vector<QueueSolverNode> &queues,
    const std::vector<TaskSolverNode> &tasks);

std::map<std::string, std::uint64_t>
BuildTaskDurationsMs(const std::vector<TaskSolverNode> &taskNodes,
                     const Epg::GraphConfig &config);

} // namespace EpgOptimizerPlanInternal
} // namespace SmartDrone::Core::Application
