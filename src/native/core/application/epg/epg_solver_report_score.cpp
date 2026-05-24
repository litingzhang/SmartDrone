#include "core/application/epg/epg_solver_primitives.h"

namespace SmartDrone::Core::Application::EpgSolverPrimitives {
namespace {

std::uint64_t SolverReportTotalPenalty(
    const Epg::SolverReportScore &score)
{
    return score.queuePressure * 1000 + score.periodicOverloadUs +
           score.resourceWaitUs + score.schedulingErrors * 10000 +
           score.budgetOverruns * 2000 + score.deadlineMisses * 5000 +
           score.utilizationOverPpm + score.topologyPenalty;
}

void AddQueueDecisionScore(const Epg::SolverReportDecision &decision,
                           Epg::SolverReportScore &score)
{
    score.queuePressure += decision.pressureAfter;
}

void AddTaskDecisionScore(const Epg::SolverReportDecision &decision,
                          Epg::SolverReportScore &score)
{
    score.periodicOverloadUs += TaskPeriodicOverloadUs(
        decision.intervalAfterMs, decision.effectiveLoopUs);
    score.resourceWaitUs += decision.predictedResourceWaitUs;
    score.schedulingErrors += decision.schedulingErrorCount;
    score.budgetOverruns += decision.budgetOverrunCount;
    score.deadlineMisses += decision.deadlineMissCount;
    score.utilizationOverPpm += TaskUtilizationOverPpm(
        decision.intervalBeforeMs, decision.intervalAfterMs,
        decision.utilizationPpm, decision.targetUtilizationPpm);
    score.topologyPenalty += decision.topologyPenalty;
}

} // namespace

Epg::SolverReportScore BuildSolverReportScore(
    const std::vector<Epg::SolverReportDecision> &decisions)
{
    Epg::SolverReportScore score;
    for (const auto &decision : decisions) {
        if (decision.kind == "queue") {
            AddQueueDecisionScore(decision, score);
            continue;
        }
        if (decision.kind == "task") {
            AddTaskDecisionScore(decision, score);
        }
    }
    score.totalPenalty = SolverReportTotalPenalty(score);
    return score;
}

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
