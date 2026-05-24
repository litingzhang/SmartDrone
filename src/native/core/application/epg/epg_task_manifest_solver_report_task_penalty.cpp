#include "core/application/epg/epg_task_manifest_solver_report_task_penalty.h"

#include "core/application/epg/epg_solver_primitives.h"
#include "core/application/epg/epg_task_manifest.h"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace SmartDrone::Core::Application {
namespace {

namespace Solver = EpgSolverPrimitives;

const auto SOLVER_LIMITS = Solver::DefaultSolverLimits();

struct TaskPenaltyContext {
    const Epg::SolverReportDecision *decision{nullptr};
    const Epg::TaskProfileMetrics *stats{nullptr};
    std::uint64_t effectiveLoopUs{0};
    const Epg::GraphConfig *sourceGraphConfig{nullptr};
    const Epg::GraphProfileDiagnostics *diagnostics{nullptr};
    const Epg::TaskConfig *task{nullptr};
};

struct TaskCandidatePenaltyInput {
    std::uint64_t intervalMs{0};
    std::string resource;
    int cpuAffinity{-1};
    std::vector<Epg::PortId> backpressureOutputs;
};

std::uint64_t SolverTaskCandidatePenalty(
    const TaskPenaltyContext &context,
    const TaskCandidatePenaltyInput &input)
{
    const auto &decision = *context.decision;
    const auto &stats = *context.stats;
    return Solver::EvaluateTaskCandidate(
               {context.sourceGraphConfig,
                context.diagnostics,
                context.task,
                &stats,
                &decision.backpressureBefore,
                &input.backpressureOutputs,
                decision.intervalBeforeMs,
                input.intervalMs,
                context.effectiveLoopUs,
                decision.targetUtilizationPpm,
                &decision.resourceBefore,
                &input.resource,
                decision.replaceable})
        .penalty;
}

std::vector<int> TaskCpuAffinityCandidates(
    const Epg::SolverReportDecision &decision,
    const Epg::TaskProfileMetrics &stats,
    const std::string &resource)
{
    (void)stats;
    (void)resource;
    return {decision.cpuAffinityBefore};
}

Solver::TaskCandidateOptionSpace TaskPenaltyOptionSpace(
    const EpgTaskOptimumPenaltyContext &context)
{
    const auto &decision = *context.decision;
    return Solver::BuildTaskCandidateOptionSpace(
        {context.sourceGraphConfig,
         context.diagnostics,
         context.task,
         context.stats,
         &context.catalog->resourceAlternates,
         decision.intervalBeforeMs,
         decision.effectiveLoopUs,
         decision.targetUtilizationPpm,
         context.constraints->maxPeriodicIntervalMs,
         SOLVER_LIMITS.resourceWaitPressureUs,
         &decision.resourceBefore,
         &decision.backpressureBefore,
         decision.replaceable,
         context.catalog->preserveAccuracy});
}

TaskPenaltyContext MakeTaskPenaltyContext(
    const EpgTaskOptimumPenaltyContext &context)
{
    return {
        context.decision,
        context.stats,
        context.decision->effectiveLoopUs,
        context.sourceGraphConfig,
        context.diagnostics,
        context.task,
    };
}

std::uint64_t CandidatePenalty(
    const TaskPenaltyContext &context,
    std::uint64_t intervalMs,
    const std::string &resource,
    int cpuAffinity,
    const std::vector<Epg::PortId> &backpressureOutputs)
{
    return SolverTaskCandidatePenalty(
        context, {intervalMs, resource, cpuAffinity, backpressureOutputs});
}

void ConsiderTaskPenaltyCandidate(
    const TaskPenaltyContext &context,
    const TaskCandidatePenaltyInput &input,
    std::uint64_t &bestPenalty)
{
    bestPenalty = std::min(bestPenalty,
                           SolverTaskCandidatePenalty(context, input));
}

void ConsiderTaskIntervalPenalty(
    const TaskPenaltyContext &context,
    const TaskCandidatePenaltyInput &input,
    const std::vector<Epg::PortId> &optimizedBackpressure,
    std::uint64_t &bestPenalty)
{
    auto candidate = input;
    ConsiderTaskPenaltyCandidate(context, candidate, bestPenalty);
    if (optimizedBackpressure == input.backpressureOutputs) {
        return;
    }
    candidate.backpressureOutputs = optimizedBackpressure;
    ConsiderTaskPenaltyCandidate(context, candidate, bestPenalty);
}

} // namespace

std::uint64_t BuildEpgTaskActualPenalty(
    const EpgTaskOptimumPenaltyContext &context)
{
    const auto &decision = *context.decision;
    const TaskPenaltyContext penaltyContext{
        &decision,
        context.stats,
        Solver::EffectiveLoopUs(*context.stats),
        context.sourceGraphConfig,
        context.diagnostics,
        context.task,
    };
    return CandidatePenalty(
        penaltyContext, decision.intervalAfterMs, decision.resourceAfter,
        decision.cpuAffinityAfter, decision.backpressureAfter);
}

std::uint64_t BuildEpgTaskBestPenalty(
    const EpgTaskOptimumPenaltyContext &context)
{
    const auto &decision = *context.decision;
    const auto &stats = *context.stats;
    const auto options = TaskPenaltyOptionSpace(context);
    const auto penaltyContext = MakeTaskPenaltyContext(context);
    auto bestPenalty = std::numeric_limits<std::uint64_t>::max();
    for (const auto &resource : options.resourceCandidates) {
        for (const auto cpuAffinity :
             TaskCpuAffinityCandidates(decision, stats, resource)) {
            for (std::uint64_t interval = decision.intervalBeforeMs;
                 interval <= options.maxIntervalMs; ++interval) {
                ConsiderTaskIntervalPenalty(
                    penaltyContext,
                    {interval, resource, cpuAffinity,
                     decision.backpressureBefore},
                    options.backpressureOptimized, bestPenalty);
            }
        }
    }
    return bestPenalty;
}

std::uint64_t BuildEpgTaskTopologyPenalty(
    const EpgTaskOptimumPenaltyContext &context)
{
    const auto &decision = *context.decision;
    return Solver::EvaluateTaskCandidate(
               {context.sourceGraphConfig,
                context.diagnostics,
                context.task,
                context.stats,
                &decision.backpressureBefore,
                &decision.backpressureAfter,
                decision.intervalBeforeMs,
                decision.intervalAfterMs,
                decision.effectiveLoopUs,
                decision.targetUtilizationPpm,
                &decision.resourceBefore,
                &decision.resourceAfter,
                decision.replaceable})
        .topologyPenalty;
}

} // namespace SmartDrone::Core::Application
