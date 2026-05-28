#include "core/application/epg/epg_runtime_optimizer_plan.h"

#include "core/application/epg/epg_runtime_optimizer_apply.h"
#include "core/application/epg/epg_runtime_optimizer_candidates.h"
#include "core/application/epg/epg_runtime_optimizer_cpu_binding.h"
#include "core/application/epg/epg_runtime_optimizer_topology.h"
#include "core/application/epg/epg_solver_primitives.h"
#include "core/application/epg/epg_task_manifest.h"

#include <algorithm>
#include <map>
#include <set>
#include <unistd.h>
#include <vector>

namespace SmartDrone::Core::Application {
namespace {

namespace Solver = EpgSolverPrimitives;
using namespace EpgOptimizerPlanInternal;

const auto SOLVER_LIMITS = Solver::DefaultSolverLimits();

std::size_t HardwareCpuCount()
{
    const auto detected = sysconf(_SC_NPROCESSORS_ONLN);
    if (detected <= 0) {
        return 1;
    }
    return std::min<std::size_t>(static_cast<std::size_t>(detected),
                                 SOLVER_LIMITS.maxCpuBindingCores);
}

std::set<std::string> PreserveAccuracyTaskNames(
    const Epg::GraphConfig &config,
    const std::vector<TaskSolverNode> &nodes)
{
    std::set<std::string> names;
    for (const auto &node : nodes) {
        if (node.catalog && node.catalog->preserveAccuracy) {
            names.insert(config.tasks[node.index].name);
        }
    }
    return names;
}

void ApplySelectedQueueCandidates(EpgRuntimeOptimizerPlan &plan,
                                  const std::vector<QueueSolverNode> &nodes,
                                  const TopologySolution &solution)
{
    for (std::size_t index = 0; index < nodes.size(); ++index) {
        const auto &node = nodes[index];
        const auto &candidate =
            node.candidates[solution.queueCandidateIndexes[index]];
        ApplyQueueSolution(plan.config.queues[node.index], node, candidate,
                           plan.decisions);
    }
}

void ApplySelectedTaskCandidates(EpgRuntimeOptimizerPlan &plan,
                                 const std::vector<TaskSolverNode> &nodes,
                                 const TopologySolution &solution)
{
    for (std::size_t index = 0; index < nodes.size(); ++index) {
        const auto &node = nodes[index];
        const auto &candidate =
            node.candidates[solution.taskCandidateIndexes[index]];
        ApplyTaskSolution(plan.config.tasks[node.index], node, candidate,
                          plan.decisions);
    }
}

void ApplyCpuAndTopologySchedule(
    EpgRuntimeOptimizerPlan &plan,
    const std::vector<TaskSolverNode> &taskNodes,
    const std::map<std::string, std::uint64_t> &durations)
{
    const auto cpuBinding = Solver::BuildCpuBindingSchedule(
        plan.config, durations, HardwareCpuCount(),
        SOLVER_LIMITS.maxExactCpuBindingStates);
    ApplyEpgOptimizerCpuBindingSchedule(
        plan.config, plan.decisions, cpuBinding,
        PreserveAccuracyTaskNames(plan.config, taskNodes));
    if (!HasExecutionPlanChange(plan.decisions)) {
        return;
    }
    const auto topologySchedule =
        Solver::BuildTaskTopologySchedule(plan.config, durations);
    ApplyEpgOptimizerTaskTopologySchedule(plan.config, plan.decisions,
                                          topologySchedule);
}

} // namespace

EpgRuntimeOptimizerPlan BuildEpgOptimizerPlan(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig &profileTopology,
    const Epg::GraphProfileDiagnostics &diagnostics)
{
    EpgRuntimeOptimizerPlan plan;
    plan.config = profileTopology;
    ApplyEpgTaskCatalogDefaults(manifest, plan.config);
    const auto queueNodes = BuildQueueSolverNodes(plan.config, diagnostics);
    const auto taskNodes = BuildTaskSolverNodes(manifest, plan.config, diagnostics);
    const auto solution = SolveGlobalTopology(queueNodes, taskNodes);
    ApplySelectedQueueCandidates(plan, queueNodes, solution);
    ApplySelectedTaskCandidates(plan, taskNodes, solution);
    ApplyCpuAndTopologySchedule(
        plan, taskNodes, BuildTaskDurationsMs(taskNodes, plan.config));
    return plan;
}

} // namespace SmartDrone::Core::Application
