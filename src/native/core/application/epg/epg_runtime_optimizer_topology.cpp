#include "core/application/epg/epg_runtime_optimizer_topology.h"

#include <utility>

namespace SmartDrone::Core::Application {
namespace {

namespace Solver = EpgSolverPrimitives;

Epg::TaskConfig TaskWithTopologySchedule(
    const Epg::TaskConfig &task,
    const EpgSolverPrimitives::TaskTopologyScheduleEntry &entry)
{
    auto scheduledTask = task;
    scheduledTask.scheduling.topologyLevel = entry.level;
    scheduledTask.scheduling.phaseOffsetMs = entry.phaseOffsetMs;
    scheduledTask.scheduling.phaseOffsetConfigured = true;
    return scheduledTask;
}

void ApplyTopologyDecision(Epg::SolverReportDecision &decision,
                           const EpgSolverPrimitives::TaskTopologyScheduleEntry &entry)
{
    decision.topologyLevel = entry.level;
    decision.phaseOffsetMs = entry.phaseOffsetMs;
}

} // namespace

void ApplyEpgOptimizerTaskTopologySchedule(
    Epg::GraphConfig &config,
    std::vector<Epg::SolverReportDecision> &decisions,
    const EpgSolverPrimitives::TaskTopologySchedule &schedule)
{
    std::vector<Epg::TaskConfig> reorderedTasks;
    reorderedTasks.reserve(config.tasks.size());
    for (const auto &entry : schedule.entries) {
        reorderedTasks.push_back(
            TaskWithTopologySchedule(config.tasks[entry.sourceIndex], entry));
        auto *decision =
            Solver::FindSolverReportTaskDecision(decisions, entry.name);
        if (!decision) {
            continue;
        }
        ApplyTopologyDecision(*decision, entry);
    }
    config.tasks = std::move(reorderedTasks);
}

} // namespace SmartDrone::Core::Application
