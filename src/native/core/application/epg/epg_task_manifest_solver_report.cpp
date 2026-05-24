#include "core/application/epg/epg_task_manifest.h"
#include "core/application/epg/epg_task_manifest_solver_report_decisions.h"
#include "core/application/epg/epg_task_manifest_solver_report_global_optimum.h"
#include "core/application/epg/epg_task_manifest_solver_report_metadata.h"
#include "core/application/epg/epg_task_manifest_solver_report_schedule.h"

#include <stdexcept>

namespace SmartDrone::Core::Application {
namespace {

void ValidateSourceProfileDiagnostics(
    const Epg::GraphProfile &sourceProfile)
{
    for (const auto &queue : sourceProfile.topology.queues) {
        if (sourceProfile.diagnostics.queues.find(queue.name) !=
            sourceProfile.diagnostics.queues.end()) {
            continue;
        }
        throw std::runtime_error("solver report queue diagnostics missing: " +
                                 queue.name);
    }
    for (const auto &task : sourceProfile.topology.tasks) {
        if (sourceProfile.diagnostics.tasks.find(task.name) !=
            sourceProfile.diagnostics.tasks.end()) {
            continue;
        }
        throw std::runtime_error("solver report task diagnostics missing: " +
                                 task.name);
    }
}

} // namespace

void ValidateEpgSolverReport(
    const EpgTaskManifest &manifest,
    const Epg::OptimizedGraph &optimizedGraph,
    const Epg::SolverReport &report)
{
    ValidateEpgSolverReportManifest(manifest, optimizedGraph.metadata,
                                    report.metadata);
    if (report.objectiveName.empty()) {
        throw std::runtime_error("solver report objective missing");
    }
    ValidateEpgSolverReportScore(report);
    ValidateEpgSolverReportDecisionCoverage(optimizedGraph.config, report);
    ValidateEpgSolverReportDecisionDetails(
        manifest, nullptr, optimizedGraph.config, report);
    ValidateEpgSolverReportTopologySchedule(optimizedGraph.config, report);
    if (EpgSolverReportUsesGlobalObjective(report)) {
        ValidateEpgSolverReportGlobalOptimum(
            manifest, optimizedGraph, report, nullptr);
        ValidateEpgSolverReportCpuBindingSchedule(optimizedGraph.config, report);
    }
}

void ValidateEpgSolverReport(
    const EpgTaskManifest &manifest,
    const Epg::GraphProfile &sourceProfile,
    const Epg::OptimizedGraph &optimizedGraph,
    const Epg::SolverReport &report)
{
    ValidateEpgSolverReportManifest(manifest, optimizedGraph.metadata,
                                    report.metadata);
    ValidateEpgSolverReportProfile(manifest, sourceProfile.metadata,
                                   optimizedGraph.metadata, report.metadata);
    ValidateSourceProfileDiagnostics(sourceProfile);
    if (report.objectiveName.empty()) {
        throw std::runtime_error("solver report objective missing");
    }
    ValidateEpgSolverReportScore(report);
    ValidateEpgSolverReportDecisionCoverage(optimizedGraph.config, report);
    ValidateEpgSolverReportDecisionDetails(
        manifest, &sourceProfile.topology, optimizedGraph.config, report);
    ValidateEpgSolverReportTopologySchedule(optimizedGraph.config, report);
    if (EpgSolverReportUsesGlobalObjective(report)) {
        ValidateEpgSolverReportGlobalOptimum(manifest, optimizedGraph, report,
                                             &sourceProfile);
        ValidateEpgSolverReportCpuBindingSchedule(optimizedGraph.config, report);
    }
}

} // namespace SmartDrone::Core::Application
