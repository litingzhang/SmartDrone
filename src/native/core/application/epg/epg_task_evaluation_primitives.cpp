#include "core/application/epg/epg_solver_primitives.h"
#include "core/application/epg/epg_solver_task_evaluation_primitives.h"

#include <vector>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {
namespace {

bool TaskUsesFixedInterval(const TaskCandidateOptionSpaceInput &input)
{
    return !input.replaceable || input.preserveAccuracy ||
           input.intervalBeforeMs == 0;
}

std::uint64_t MaxTaskCandidateIntervalMs(
    const TaskCandidateOptionSpaceInput &input)
{
    if (TaskUsesFixedInterval(input)) {
        return input.intervalBeforeMs;
    }
    return TaskFeasibleIntervalLimit(
        input.intervalBeforeMs, *input.stats, input.effectiveLoopUs,
        input.targetUtilizationPpm, input.maxPeriodicIntervalMs);
}

bool TaskCanUseResourceAlternates(
    const TaskCandidateOptionSpaceInput &input)
{
    return input.replaceable && !input.preserveAccuracy &&
           HasResourceSplitPressure(*input.stats, input.resourceWaitPressureUs,
                                    input.targetUtilizationPpm);
}

std::vector<std::string> TaskResourceCandidates(
    const TaskCandidateOptionSpaceInput &input)
{
    std::vector<std::string> resources{*input.resourceBefore};
    if (!TaskCanUseResourceAlternates(input) || !input.resourceAlternates) {
        return resources;
    }
    for (const auto &resource : *input.resourceAlternates) {
        if (resource != *input.resourceBefore) {
            resources.push_back(resource);
        }
    }
    return resources;
}

std::vector<Epg::PortId> OptimizedBackpressure(
    const TaskCandidateOptionSpaceInput &input)
{
    if (!input.sourceGraphConfig || input.preserveAccuracy) {
        return *input.backpressureBefore;
    }
    return CandidateBackpressurePorts(
        *input.sourceGraphConfig, input.diagnostics, *input.task,
        *input.backpressureBefore, input.replaceable);
}

} // namespace

std::uint64_t TaskCandidatePenalty(
    const TaskCandidatePenaltyInput &input)
{
    const auto &stats = *input.stats;
    return TaskPeriodicOverloadUs(input.intervalAfterMs,
                                  input.effectiveLoopUs) +
           input.predictedResourceWaitUs +
           stats.schedulingErrorCount * 10000 +
           stats.budgetOverrunCount * 2000 +
           stats.deadlineMissCount * 5000 +
           TaskUtilizationOverPpm(input.intervalBeforeMs,
                                  input.intervalAfterMs,
                                  stats.utilizationPpm,
                                  input.targetUtilizationPpm) +
           input.intervalAfterMs + input.topologyPenalty;
}

TaskCandidateEvaluation EvaluateTaskCandidate(
    const TaskCandidateEvaluationInput &input)
{
    TaskCandidateEvaluation evaluation;
    evaluation.predictedResourceWaitUs = PredictedResourceWaitUs(
        input.stats->totalResourceWaitUs, *input.resourceBefore,
        *input.resourceAfter);
    evaluation.topologyPenalty =
        BackpressureTopologyPenalty(
            {input.sourceGraphConfig, input.diagnostics, input.task,
             input.backpressureBefore, input.backpressureAfter,
             input.stats->totalResourceWaitUs, input.replaceable}) +
        ResourceTopologyPenalty(*input.resourceBefore, *input.resourceAfter);
    evaluation.penalty = TaskCandidatePenalty(
        {input.intervalBeforeMs, input.intervalAfterMs, input.stats,
         input.effectiveLoopUs, evaluation.predictedResourceWaitUs,
         evaluation.topologyPenalty, input.targetUtilizationPpm});
    return evaluation;
}

TaskCandidateOptionSpace BuildTaskCandidateOptionSpace(
    const TaskCandidateOptionSpaceInput &input)
{
    return {
        MaxTaskCandidateIntervalMs(input),
        TaskResourceCandidates(input),
        OptimizedBackpressure(input),
    };
}

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
