#include "core/application/epg/epg_task_manifest_solver_report_queue_optimum.h"

#include "core/application/epg/epg_solver_primitives.h"
#include "core/application/epg/epg_task_manifest_solver_report_global_optimum_support.h"

#include <algorithm>
#include <stdexcept>

namespace SmartDrone::Core::Application {
namespace {

namespace Solver = EpgSolverPrimitives;

Epg::QueueProfileMetrics QueueMetricsFromDecision(
    const Epg::SolverReportDecision &decision)
{
    Epg::QueueProfileMetrics stats;
    stats.maxDepthObserved = decision.maxDepthObserved;
    stats.droppedNewest = decision.droppedNewest;
    stats.overwrittenOldest = decision.overwrittenOldest;
    stats.pushedPerSecond = decision.pushedPerSecond;
    stats.poppedPerSecond = decision.poppedPerSecond;
    stats.droppedPerSecond = decision.droppedPerSecond;
    if (stats.maxDepthObserved == 0 && decision.pressureBefore > 0) {
        stats.maxDepthObserved =
            decision.depthBefore + decision.pressureBefore;
    }
    return stats;
}

std::uint64_t BestQueuePenalty(
    const Epg::QueueProfileMetrics &stats,
    const Epg::SolverReportDecision &decision)
{
    return Solver::QueueCandidatePenalty(
        std::max<std::uint64_t>(1, decision.depthBefore), stats);
}

void ValidateQueueProfileMetrics(
    const Epg::QueueConfig &queue,
    const Epg::SolverReportDecision &decision,
    const Epg::QueueProfileMetrics &stats,
    bool hasProfileStats)
{
    if (hasProfileStats && (decision.maxDepthObserved != stats.maxDepthObserved ||
                            decision.droppedNewest != stats.droppedNewest ||
                            decision.overwrittenOldest !=
                                stats.overwrittenOldest)) {
        throw std::runtime_error("solver report queue metrics mismatch: " +
                                 queue.name);
    }
    const auto expectedPressureBefore =
        Solver::QueuePressureAtDepth(decision.depthBefore, stats);
    const auto expectedPressureAfter =
        Solver::QueuePressureAtDepth(decision.depthAfter, stats);
    if (decision.pressureBefore != expectedPressureBefore ||
        decision.pressureAfter != expectedPressureAfter ||
        decision.pushedPerSecond != stats.pushedPerSecond ||
        decision.poppedPerSecond != stats.poppedPerSecond ||
        decision.droppedPerSecond != stats.droppedPerSecond) {
        throw std::runtime_error("solver report queue metrics mismatch: " +
                                 queue.name);
    }
}

void ValidateQueuePenalty(
    const Epg::QueueConfig &queue,
    const Epg::SolverReportDecision &decision,
    const Epg::QueueProfileMetrics &stats)
{
    const auto actualPenalty =
        Solver::QueueCandidatePenalty(decision.depthAfter, stats);
    const auto bestPenalty = BestQueuePenalty(stats, decision);
    if (actualPenalty != bestPenalty) {
        throw std::runtime_error("solver report queue is not optimal: " +
                                 queue.name);
    }
}

} // namespace

void ValidateEpgQueueGlobalOptimum(
    const EpgTaskManifest &manifest,
    const Epg::SolverReport &report,
    const Epg::QueueConfig &queue,
    const Epg::QueueProfileMetrics *profileStats)
{
    const auto &decision =
        RequireEpgSolverReportDecision(report, "queue", queue.name);
    const auto decisionStats = QueueMetricsFromDecision(decision);
    const auto &stats = profileStats ? *profileStats : decisionStats;
    (void)manifest;
    ValidateQueueProfileMetrics(queue, decision, stats, profileStats);
    ValidateQueuePenalty(queue, decision, stats);
}

} // namespace SmartDrone::Core::Application
