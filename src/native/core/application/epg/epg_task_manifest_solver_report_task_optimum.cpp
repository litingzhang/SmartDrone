#include "core/application/epg/epg_task_manifest_solver_report_task_optimum.h"

#include "core/application/epg/epg_solver_primitives.h"
#include "core/application/epg/epg_task_manifest.h"
#include "core/application/epg/epg_task_manifest_internal.h"
#include "core/application/epg/epg_task_manifest_solver_report_global_optimum_support.h"
#include "core/application/epg/epg_task_manifest_solver_report_task_penalty.h"

#include <stdexcept>

namespace SmartDrone::Core::Application {
namespace {

namespace Solver = EpgSolverPrimitives;

using EpgTaskManifestInternal::RequireCatalogEntry;

bool TaskDecisionMetricsMatch(const Epg::SolverReportDecision &decision,
                              const Epg::TaskProfileMetrics &stats)
{
    return decision.maxLoopUs == stats.maxLoopUs &&
           decision.averageLoopUs == stats.averageLoopUs &&
           decision.p90LoopUs == stats.p90LoopUs &&
           decision.p99LoopUs == stats.p99LoopUs &&
           decision.resourceWaitCount == stats.resourceWaitCount &&
           decision.maxResourceWaitUs == stats.maxResourceWaitUs &&
           decision.averageResourceWaitUs == stats.averageResourceWaitUs &&
           decision.totalResourceWaitUs == stats.totalResourceWaitUs &&
           decision.utilizationPpm == stats.utilizationPpm &&
           decision.budgetOverrunCount == stats.budgetOverrunCount &&
           decision.deadlineMissCount == stats.deadlineMissCount &&
           decision.schedulingErrorCount == stats.schedulingErrorCount;
}

void ValidateTaskMetrics(
    const Epg::TaskConfig &optimizedTask,
    const Epg::SolverReportDecision &decision,
    const Epg::TaskProfileMetrics &stats,
    bool hasProfileStats)
{
    if (hasProfileStats && !TaskDecisionMetricsMatch(decision, stats)) {
        throw std::runtime_error("solver report task metrics mismatch: " +
                                 optimizedTask.name);
    }
    if (decision.effectiveLoopUs != Solver::EffectiveLoopUs(stats)) {
        throw std::runtime_error("solver report task metrics mismatch: " +
                                 optimizedTask.name);
    }
}

void ValidateTaskResourcePrediction(
    const Epg::TaskConfig &optimizedTask,
    const Epg::SolverReportDecision &decision,
    const Epg::TaskProfileMetrics &stats)
{
    const auto expected = Solver::PredictedResourceWaitUs(
        stats.totalResourceWaitUs, decision.resourceBefore,
        decision.resourceAfter);
    if (decision.predictedResourceWaitUs != expected) {
        throw std::runtime_error("solver report task resource mismatch: " +
                                 optimizedTask.name);
    }
}

} // namespace

void ValidateEpgTaskGlobalOptimum(
    const EpgTaskGlobalOptimumContext &context)
{
    const auto &optimizedTask = *context.optimizedTask;
    const auto &report = *context.report;
    const auto &decision = RequireEpgSolverReportDecision(
        report, "task", optimizedTask.name);
    const auto &catalog =
        RequireCatalogEntry(*context.manifest, optimizedTask.type);
    if (decision.durationMs != Solver::CeilDiv(decision.effectiveLoopUs, 1000)) {
        throw std::runtime_error("solver report task schedule mismatch: " +
                                 optimizedTask.name);
    }
    const auto *sourceTask = context.sourceGraphConfig
                                 ? Solver::FindTaskConfig(
                                       *context.sourceGraphConfig,
                                       optimizedTask.name)
                                 : nullptr;
    const auto &candidateTask = sourceTask ? *sourceTask : optimizedTask;
    const auto decisionStats = Solver::TaskMetricsFromDecision(decision);
    const auto &stats = context.profileStats ? *context.profileStats
                                             : decisionStats;
    ValidateTaskMetrics(optimizedTask, decision, stats, context.profileStats);
    ValidateTaskResourcePrediction(optimizedTask, decision, stats);
    const EpgTaskOptimumPenaltyContext penaltyContext{
        &report.constraints,
        context.sourceGraphConfig,
        context.diagnostics,
        &candidateTask,
        &stats,
        &decision,
        &catalog,
    };
    const auto actualPenalty = BuildEpgTaskActualPenalty(penaltyContext);
    const auto bestPenalty = BuildEpgTaskBestPenalty(penaltyContext);
    if (actualPenalty != bestPenalty) {
        throw std::runtime_error("solver report task is not optimal: " +
                                 optimizedTask.name);
    }
    const auto expectedTopologyPenalty =
        BuildEpgTaskTopologyPenalty(penaltyContext);
    if (decision.topologyPenalty != expectedTopologyPenalty) {
        throw std::runtime_error("solver report task topology mismatch: " +
                                 optimizedTask.name);
    }
}

} // namespace SmartDrone::Core::Application
