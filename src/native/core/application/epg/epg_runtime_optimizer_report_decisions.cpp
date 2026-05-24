#include "core/application/epg/epg_runtime_optimizer_report_decisions.h"

#include "core/application/epg/epg_solver_primitives.h"

#include <cstddef>
#include <sstream>

namespace SmartDrone::Core::Application {
namespace {

namespace Solver = EpgSolverPrimitives;

const auto SOLVER_LIMITS = Solver::DefaultSolverLimits();

void WritePortArray(std::ostringstream &out,
                    const std::vector<Epg::PortId> &ports)
{
    out << "[";
    for (std::size_t index = 0; index < ports.size(); ++index) {
        if (index != 0) {
            out << ", ";
        }
        out << ports[index];
    }
    out << "]";
}

void WriteQueueDecisionJson(std::ostringstream &out,
                            const Epg::SolverReportDecision &decision)
{
    out << "\"depthBefore\": " << decision.depthBefore << ", ";
    out << "\"depthAfter\": " << decision.depthAfter << ", ";
    out << "\"pressureBefore\": " << decision.pressureBefore << ", ";
    out << "\"pressureAfter\": " << decision.pressureAfter << ", ";
    out << "\"maxDepthObserved\": " << decision.maxDepthObserved << ", ";
    out << "\"droppedNewest\": " << decision.droppedNewest << ", ";
    out << "\"overwrittenOldest\": " << decision.overwrittenOldest << ", ";
    out << "\"pushedPerSecond\": " << decision.pushedPerSecond << ", ";
    out << "\"poppedPerSecond\": " << decision.poppedPerSecond << ", ";
    out << "\"droppedPerSecond\": " << decision.droppedPerSecond << ", ";
}

void WriteTaskMetricJson(std::ostringstream &out,
                         const Epg::SolverReportDecision &decision)
{
    out << "\"intervalBeforeMs\": " << decision.intervalBeforeMs
        << ", \"intervalAfterMs\": " << decision.intervalAfterMs
        << ", \"maxLoopUs\": " << decision.maxLoopUs
        << ", \"averageLoopUs\": " << decision.averageLoopUs
        << ", \"p90LoopUs\": " << decision.p90LoopUs
        << ", \"p99LoopUs\": " << decision.p99LoopUs
        << ", \"effectiveLoopUs\": " << decision.effectiveLoopUs
        << ", \"resourceWaitCount\": " << decision.resourceWaitCount
        << ", \"maxResourceWaitUs\": " << decision.maxResourceWaitUs
        << ", \"averageResourceWaitUs\": " << decision.averageResourceWaitUs
        << ", ";
}

void WriteTaskResourceJson(std::ostringstream &out,
                           const Epg::SolverReportDecision &decision)
{
    out << "\"totalResourceWaitUs\": " << decision.totalResourceWaitUs
        << ", \"predictedResourceWaitUs\": " << decision.predictedResourceWaitUs
        << ", \"utilizationPpm\": " << decision.utilizationPpm
        << ", \"targetUtilizationPpm\": " << SOLVER_LIMITS.targetUtilizationPpm
        << ", \"budgetUs\": " << decision.budgetUs
        << ", \"deadlineUs\": " << decision.deadlineUs
        << ", \"catalogRole\": \"" << Solver::JsonEscape(decision.catalogRole)
        << "\", \"replaceable\": " << (decision.replaceable ? "true" : "false")
        << ", \"resourceBefore\": \""
        << Solver::JsonEscape(decision.resourceBefore)
        << "\", \"resourceAfter\": \""
        << Solver::JsonEscape(decision.resourceAfter) << "\", ";
}

void WriteTaskScheduleJson(std::ostringstream &out,
                           const Epg::SolverReportDecision &decision)
{
    out << "\"budgetOverrunCount\": " << decision.budgetOverrunCount
        << ", \"deadlineMissCount\": " << decision.deadlineMissCount
        << ", \"schedulingErrorCount\": " << decision.schedulingErrorCount
        << ", \"topologyPenalty\": " << decision.topologyPenalty
        << ", \"topologyLevel\": " << decision.topologyLevel
        << ", \"phaseOffsetMs\": " << decision.phaseOffsetMs
        << ", \"durationMs\": " << decision.durationMs
        << ", \"cpuBindingAffinity\": " << decision.cpuBindingAffinity
        << ", \"cpuBindingStartMs\": " << decision.cpuBindingStartMs
        << ", \"cpuBindingFinishMs\": " << decision.cpuBindingFinishMs
        << ", \"cpuBindingMakespanMs\": "
        << decision.cpuBindingMakespanMs << ", ";
}

void WriteTaskDecisionJson(std::ostringstream &out,
                           const Epg::SolverReportDecision &decision)
{
    WriteTaskMetricJson(out, decision);
    WriteTaskResourceJson(out, decision);
    WriteTaskScheduleJson(out, decision);
    out << "\"cpuAffinityBefore\": " << decision.cpuAffinityBefore
        << ", \"cpuAffinityAfter\": " << decision.cpuAffinityAfter << ", ";
    out << "\"backpressureBefore\": ";
    WritePortArray(out, decision.backpressureBefore);
    out << ", \"backpressureAfter\": ";
    WritePortArray(out, decision.backpressureAfter);
    out << ", ";
}

void WriteDecisionJson(std::ostringstream &out,
                       const Epg::SolverReportDecision &decision)
{
    out << "    {";
    out << "\"kind\": \"" << Solver::JsonEscape(decision.kind) << "\", ";
    out << "\"name\": \"" << Solver::JsonEscape(decision.name) << "\", ";
    if (decision.kind == "queue") {
        WriteQueueDecisionJson(out, decision);
    } else {
        WriteTaskDecisionJson(out, decision);
    }
    out << "\"reason\": \"" << Solver::JsonEscape(decision.reason) << "\"";
    out << "}";
}

} // namespace

void WriteEpgOptimizerSolverReportDecisions(
    std::ostringstream &out,
    const std::vector<Epg::SolverReportDecision> &decisions)
{
    out << "  \"decisions\": [\n";
    for (std::size_t index = 0; index < decisions.size(); ++index) {
        if (index != 0) {
            out << ",\n";
        }
        WriteDecisionJson(out, decisions[index]);
    }
    out << "\n  ]\n";
}

} // namespace SmartDrone::Core::Application
