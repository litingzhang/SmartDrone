#include "core/application/epg/epg_runtime_optimizer_report.h"

#include "core/application/epg/epg_runtime_optimizer.h"
#include "core/application/epg/epg_runtime_optimizer_report_decisions.h"
#include "core/application/epg/epg_solver_primitives.h"
#include "core/application/epg/epg_task_manifest.h"

#include <sstream>

namespace SmartDrone::Core::Application {
namespace {

namespace Solver = EpgSolverPrimitives;

const auto SOLVER_LIMITS = Solver::DefaultSolverLimits();
constexpr const char *REPORT_OBJECT_BEGIN = "{\n";

void WriteScoreJson(std::ostringstream &out,
                    const Epg::SolverReportScore &score)
{
    out << "\"queuePressure\": " << score.queuePressure << ", ";
    out << "\"periodicOverloadUs\": " << score.periodicOverloadUs << ", ";
    out << "\"resourceWaitUs\": " << score.resourceWaitUs << ", ";
    out << "\"schedulingErrors\": " << score.schedulingErrors << ", ";
    out << "\"budgetOverruns\": " << score.budgetOverruns << ", ";
    out << "\"deadlineMisses\": " << score.deadlineMisses << ", ";
    out << "\"utilizationOverPpm\": " << score.utilizationOverPpm << ", ";
    out << "\"topologyPenalty\": " << score.topologyPenalty << ", ";
    out << "\"totalPenalty\": " << score.totalPenalty << "}\n";
}

void WriteSolverReportHeader(std::ostringstream &out,
                             const EpgTaskManifest &manifest,
                             const Epg::GraphProfileMetadata &metadata,
                             std::uint64_t nowMs)
{
    out << REPORT_OBJECT_BEGIN;
    out << "  \"schema\": \"" << Epg::SOLVER_REPORT_SCHEMA << "\",\n";
    out << "  \"targetGraph\": \"" << Solver::JsonEscape(manifest.subgraphName)
        << "\",\n";
    out << "  \"topologyVersion\": \""
        << Solver::JsonEscape(manifest.topologyVersion) << "\",\n";
    out << "  \"sourceProfile\": \""
        << Solver::JsonEscape(manifest.subgraphName) << "\",\n";
    out << "  \"sourceTimestampMs\": " << metadata.timestampMs << ",\n";
    out << "  \"generatedAtMs\": " << nowMs << ",\n";
    out << "  \"solverVersion\": \"" << Epg::NATIVE_EXACT_SOLVER_VERSION
        << "\",\n";
}

void WriteSolverReportObjective(std::ostringstream &out,
                                const Epg::SolverReportScore &score)
{
    out << "  \"objective\": {\n";
    out << "    \"name\": \"" << EPG_EXACT_SOLVER_OBJECTIVE << "\",\n";
    out << "    \"score\": {";
    WriteScoreJson(out, score);
    out << "  },\n";
}

void WriteSolverReportConstraints(std::ostringstream &out)
{
    out << "  \"constraints\": {";
    out << "\"maxQueueDepth\": " << SOLVER_LIMITS.maxQueueDepth << ", ";
    out << "\"maxPeriodicIntervalMs\": "
        << SOLVER_LIMITS.maxPeriodicIntervalMs << ", ";
    out << "\"targetUtilizationPpm\": "
        << SOLVER_LIMITS.targetUtilizationPpm << "},\n";
}

} // namespace

std::string BuildEpgOptimizerSolverReport(
    const EpgTaskManifest &manifest,
    const Epg::GraphProfileMetadata &metadata,
    std::uint64_t nowMs,
    const std::vector<Epg::SolverReportDecision> &decisions)
{
    std::ostringstream out;
    WriteSolverReportHeader(out, manifest, metadata, nowMs);
    WriteSolverReportObjective(out, Solver::BuildSolverReportScore(decisions));
    WriteSolverReportConstraints(out);
    WriteEpgOptimizerSolverReportDecisions(out, decisions);
    out << "}\n";
    return out.str();
}

} // namespace SmartDrone::Core::Application
