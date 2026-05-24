#include "core/application/epg/epg_runtime_optimizer_apply.h"

#include "core/application/epg/epg_solver_primitives.h"
#include "core/application/epg/epg_task_manifest.h"

#include <chrono>
#include <string>

namespace SmartDrone::Core::Application::EpgOptimizerPlanInternal {
namespace {

namespace Solver = EpgSolverPrimitives;

constexpr int RESOURCE_ISOLATION_PRIORITY = 20;
const auto SOLVER_LIMITS = Solver::DefaultSolverLimits();

std::string OptimizedTaskReason(const TaskSolutionContext &context)
{
    const auto &node = *context.node;
    const auto &task = *context.task;
    return Solver::BuildTaskDecisionReason(
        {&node.stats,
         node.intervalBeforeMs,
         context.intervalAfterMs,
         node.effectiveLoopUs,
         SOLVER_LIMITS.targetUtilizationPpm,
         task.scheduling.budgetUs,
         task.scheduling.deadlineUs,
         SOLVER_LIMITS.resourceWaitPressureUs,
         &context.resourceBefore,
         &context.resourceAfter,
         &context.backpressureBefore,
         &context.backpressureAfter,
         node.catalog ? node.catalog->replaceable : false,
         context.cpuAffinityBefore,
         context.cpuAffinityAfter,
         -1});
}

void ApplyResourceIsolation(Epg::TaskConfig &task,
                            const Epg::TaskProfileMetrics &stats,
                            const EpgTaskCatalogEntry *catalog,
                            const std::string &resourceBefore)
{
    if (!catalog || !catalog->replaceable || catalog->preserveAccuracy ||
        task.scheduling.resource == resourceBefore ||
        !Solver::HasResourceWaitPressure(stats,
                                         SOLVER_LIMITS.resourceWaitPressureUs)) {
        return;
    }
    if (!task.scheduling.realtime &&
        task.trigger.interval.count() > 0) {
        task.scheduling.realtime = true;
        task.scheduling.priority = RESOURCE_ISOLATION_PRIORITY;
    }
}

TaskSolutionContext MakeTaskSolutionContext(
    const Epg::TaskConfig &task,
    const TaskSolverNode &node,
    const TaskCandidate &candidate)
{
    TaskSolutionContext context;
    context.task = &task;
    context.node = &node;
    context.candidate = &candidate;
    context.intervalAfterMs = candidate.intervalMs;
    context.backpressureBefore =
        Solver::SortedUniquePorts(task.scheduling.backpressureOutputs);
    context.resourceBefore = task.scheduling.resource;
    context.cpuAffinityBefore = task.scheduling.cpuAffinity;
    return context;
}

void ApplyTaskCandidateConfig(Epg::TaskConfig &task,
                              const TaskSolutionContext &context)
{
    const auto &node = *context.node;
    const auto &candidate = *context.candidate;
    task.trigger.interval =
        std::chrono::milliseconds(static_cast<int>(candidate.intervalMs));
    task.scheduling.resource = candidate.resource;
    task.scheduling.cpuAffinity = candidate.cpuAffinity;
    task.scheduling.backpressureOutputs =
        Solver::SortedUniquePorts(candidate.backpressureOutputs);
    ApplyResourceIsolation(task, node.stats, node.catalog,
                           context.resourceBefore);
}

void CaptureTaskSolutionAfter(const Epg::TaskConfig &task,
                              TaskSolutionContext &context)
{
    context.backpressureAfter = task.scheduling.backpressureOutputs;
    context.resourceAfter = task.scheduling.resource;
    context.cpuAffinityAfter = task.scheduling.cpuAffinity;
}

void FillTaskDecisionMetrics(SolverDecision &decision,
                             const TaskSolutionContext &context)
{
    const auto &node = *context.node;
    const auto &task = *context.task;
    decision.maxLoopUs = node.stats.maxLoopUs;
    decision.averageLoopUs = node.stats.averageLoopUs;
    decision.p90LoopUs = node.stats.p90LoopUs;
    decision.p99LoopUs = node.stats.p99LoopUs;
    decision.effectiveLoopUs = node.effectiveLoopUs;
    decision.resourceWaitCount = node.stats.resourceWaitCount;
    decision.maxResourceWaitUs = node.stats.maxResourceWaitUs;
    decision.averageResourceWaitUs = node.stats.averageResourceWaitUs;
    decision.totalResourceWaitUs = node.stats.totalResourceWaitUs;
    decision.utilizationPpm = node.stats.utilizationPpm;
    decision.targetUtilizationPpm = SOLVER_LIMITS.targetUtilizationPpm;
    decision.budgetUs = task.scheduling.budgetUs;
    decision.deadlineUs = task.scheduling.deadlineUs;
    decision.budgetOverrunCount = node.stats.budgetOverrunCount;
    decision.deadlineMissCount = node.stats.deadlineMissCount;
    decision.schedulingErrorCount = node.stats.schedulingErrorCount;
}

SolverDecision MakeTaskDecision(const TaskSolutionContext &context)
{
    const auto &task = *context.task;
    const auto &node = *context.node;
    const auto &candidate = *context.candidate;
    SolverDecision decision;
    decision.kind = "task";
    decision.name = task.name;
    decision.reason = OptimizedTaskReason(context);
    decision.catalogRole = node.catalog ? node.catalog->role : "";
    decision.replaceable = node.catalog ? node.catalog->replaceable : false;
    decision.intervalBeforeMs = node.intervalBeforeMs;
    decision.intervalAfterMs = candidate.intervalMs;
    decision.predictedResourceWaitUs = candidate.predictedResourceWaitUs;
    decision.cpuAffinityBefore = context.cpuAffinityBefore;
    decision.cpuAffinityAfter = context.cpuAffinityAfter;
    decision.resourceBefore = context.resourceBefore;
    decision.resourceAfter = context.resourceAfter;
    decision.backpressureBefore = context.backpressureBefore;
    decision.backpressureAfter = context.backpressureAfter;
    decision.topologyPenalty = candidate.topologyPenalty;
    decision.durationMs = Solver::CeilDiv(node.effectiveLoopUs, 1000);
    FillTaskDecisionMetrics(decision, context);
    return decision;
}

} // namespace

void ApplyTaskSolution(Epg::TaskConfig &task,
                       const TaskSolverNode &node,
                       const TaskCandidate &candidate,
                       std::vector<SolverDecision> &decisions)
{
    auto context = MakeTaskSolutionContext(task, node, candidate);
    ApplyTaskCandidateConfig(task, context);
    CaptureTaskSolutionAfter(task, context);
    decisions.push_back(MakeTaskDecision(context));
}

} // namespace SmartDrone::Core::Application::EpgOptimizerPlanInternal
