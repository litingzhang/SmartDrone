#include "core/application/epg/epg_runtime_optimizer_candidates.h"

#include "core/application/epg/epg_solver_primitives.h"
#include "core/application/epg/epg_task_manifest.h"
#include "core/application/epg/epg_task_manifest_internal.h"

#include <string>

namespace SmartDrone::Core::Application::EpgOptimizerPlanInternal {
namespace {

namespace Solver = EpgSolverPrimitives;

const auto SOLVER_LIMITS = Solver::DefaultSolverLimits();

using EpgTaskManifestInternal::FindCatalogEntry;

TaskCandidateBuildContext MakeTaskCandidateBuildContext(
    const TaskSolverNodeBuildContext &input,
    std::uint64_t effectiveLoopUs)
{
    const auto &task = *input.task;
    TaskCandidateBuildContext context;
    context.config = input.config;
    context.diagnostics = input.diagnostics;
    context.task = input.task;
    context.stats = input.stats;
    context.catalog = input.catalog;
    context.effectiveLoopUs = effectiveLoopUs;
    context.intervalBeforeMs =
        static_cast<std::uint64_t>(task.trigger.interval.count());
    context.replaceable = input.catalog && input.catalog->replaceable;
    context.backpressureBefore =
        Solver::SortedUniquePorts(task.scheduling.backpressureOutputs);
    const std::vector<std::string> *alternates =
        input.catalog ? &input.catalog->resourceAlternates : nullptr;
    const auto options = Solver::BuildTaskCandidateOptionSpace(
        {input.config,
         input.diagnostics,
         &task,
         input.stats,
         alternates,
         context.intervalBeforeMs,
         context.effectiveLoopUs,
         SOLVER_LIMITS.targetUtilizationPpm,
         SOLVER_LIMITS.maxPeriodicIntervalMs,
         SOLVER_LIMITS.resourceWaitPressureUs,
         &task.scheduling.resource,
         &context.backpressureBefore,
         context.replaceable,
         input.catalog && input.catalog->preserveAccuracy});
    context.backpressureOptimized = options.backpressureOptimized;
    context.resourceCandidates = options.resourceCandidates;
    context.maxIntervalMs = options.maxIntervalMs;
    return context;
}

TaskCandidate MakeTaskCandidate(
    const TaskCandidateBuildContext &context,
    std::uint64_t intervalMs,
    const std::string &resource,
    int cpuAffinity,
    const std::vector<Epg::PortId> &backpressureOutputs)
{
    const auto &resourceBefore = context.task->scheduling.resource;
    const auto evaluation = Solver::EvaluateTaskCandidate(
        {context.config,
         context.diagnostics,
         context.task,
         context.stats,
         &context.backpressureBefore,
         &backpressureOutputs,
         context.intervalBeforeMs,
         intervalMs,
         context.effectiveLoopUs,
         SOLVER_LIMITS.targetUtilizationPpm,
         &resourceBefore,
         &resource,
         context.replaceable});
    return {intervalMs,
            resource,
            cpuAffinity,
            backpressureOutputs,
            evaluation.predictedResourceWaitUs,
            evaluation.topologyPenalty,
            evaluation.penalty};
}

void AppendTaskCandidatePair(std::vector<TaskCandidate> &candidates,
                             const TaskCandidateBuildContext &context,
                             std::uint64_t intervalMs,
                             const std::string &resource)
{
    const auto cpuAffinity = context.task->scheduling.cpuAffinity;
    candidates.push_back(MakeTaskCandidate(
        context, intervalMs, resource, cpuAffinity,
        context.backpressureBefore));
    if (context.backpressureOptimized == context.backpressureBefore) {
        return;
    }
    candidates.push_back(MakeTaskCandidate(
        context, intervalMs, resource, cpuAffinity,
        context.backpressureOptimized));
}

std::vector<TaskCandidate> BuildTaskCandidates(
    const TaskCandidateBuildContext &context)
{
    std::vector<TaskCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(
        (context.maxIntervalMs - context.intervalBeforeMs + 1) *
        context.resourceCandidates.size() * 2));
    for (std::uint64_t interval = context.intervalBeforeMs;
         interval <= context.maxIntervalMs; ++interval) {
        for (const auto &resource : context.resourceCandidates) {
            AppendTaskCandidatePair(candidates, context, interval, resource);
        }
    }
    return candidates;
}

TaskSolverNode BuildTaskSolverNode(
    const TaskSolverNodeBuildContext &input)
{
    TaskSolverNode node;
    node.index = input.index;
    node.intervalBeforeMs =
        static_cast<std::uint64_t>(input.task->trigger.interval.count());
    node.effectiveLoopUs = Solver::EffectiveLoopUs(*input.stats);
    node.catalog = input.catalog;
    node.stats = *input.stats;
    node.candidates = BuildTaskCandidates(
        MakeTaskCandidateBuildContext(input, node.effectiveLoopUs));
    return node;
}

} // namespace

std::vector<TaskSolverNode> BuildTaskSolverNodes(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig &config,
    const Epg::GraphProfileDiagnostics &diagnostics)
{
    std::vector<TaskSolverNode> nodes;
    nodes.reserve(config.tasks.size());
    for (std::size_t index = 0; index < config.tasks.size(); ++index) {
        const auto &task = config.tasks[index];
        nodes.push_back(BuildTaskSolverNode({
            index,
            &config,
            &diagnostics,
            &task,
            &diagnostics.tasks.at(task.name),
            FindCatalogEntry(manifest, task.type),
        }));
    }
    return nodes;
}

std::map<std::string, std::uint64_t>
BuildTaskDurationsMs(const std::vector<TaskSolverNode> &taskNodes,
                     const Epg::GraphConfig &config)
{
    std::map<std::string, std::uint64_t> durations;
    for (const auto &node : taskNodes) {
        durations[config.tasks[node.index].name] =
            Solver::CeilDiv(node.effectiveLoopUs, 1000);
    }
    return durations;
}

} // namespace SmartDrone::Core::Application::EpgOptimizerPlanInternal
