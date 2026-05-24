#include "core/application/epg/epg_task_manifest_solver_report_global_optimum.h"

#include "core/application/epg/epg_task_manifest.h"
#include "core/application/epg/epg_task_manifest_internal.h"
#include "core/application/epg/epg_task_manifest_solver_report_queue_optimum.h"
#include "core/application/epg/epg_task_manifest_solver_report_task_optimum.h"

#include <stdexcept>

namespace SmartDrone::Core::Application {
namespace {

using EpgTaskManifestInternal::GLOBAL_TOPOLOGY_OBJECTIVE;

} // namespace

bool EpgSolverReportUsesGlobalObjective(const Epg::SolverReport &report)
{
    return report.objectiveName == GLOBAL_TOPOLOGY_OBJECTIVE;
}

void ValidateEpgSolverReportGlobalOptimum(
    const EpgTaskManifest &manifest,
    const Epg::OptimizedGraph &optimizedGraph,
    const Epg::SolverReport &report,
    const Epg::GraphProfile *sourceProfile)
{
    if (!EpgSolverReportUsesGlobalObjective(report)) {
        throw std::runtime_error("solver report objective mismatch");
    }
    const auto *sourceGraphConfig =
        sourceProfile ? &sourceProfile->topology : nullptr;
    const auto *diagnostics =
        sourceProfile ? &sourceProfile->diagnostics : nullptr;
    for (const auto &queue : optimizedGraph.config.queues) {
        const Epg::QueueProfileMetrics *stats = nullptr;
        if (sourceProfile) {
            stats = &sourceProfile->diagnostics.queues.at(queue.name);
        }
        ValidateEpgQueueGlobalOptimum(manifest, report, queue, stats);
    }
    for (const auto &task : optimizedGraph.config.tasks) {
        const Epg::TaskProfileMetrics *stats = nullptr;
        if (sourceProfile) {
            stats = &sourceProfile->diagnostics.tasks.at(task.name);
        }
        ValidateEpgTaskGlobalOptimum({
            &manifest,
            &report,
            sourceGraphConfig,
            diagnostics,
            &task,
            stats,
        });
    }
}

} // namespace SmartDrone::Core::Application
