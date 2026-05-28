#include "core/application/epg/epg_runtime_optimizer_cpu_binding.h"

#include <set>
#include <string>

#include "common/environment.h"

namespace SmartDrone::Core::Application {
namespace {

namespace Solver = EpgSolverPrimitives;

const Solver::CpuBindingScheduleEntry *FindCpuBindingEntry(
    const Solver::CpuBindingSchedule &schedule,
    const std::string &taskName)
{
    for (const auto &entry : schedule.entries) {
        if (entry.name == taskName) {
            return &entry;
        }
    }
    return nullptr;
}

bool CpuBindingApplyEnabled()
{
    return SmartDrone::Common::EnvStringValue(
               "SMART_DRONE_EPG_APPLY_CPU_BINDING", "") == "1";
}

bool ShouldApplyCpuBindingAffinity(const Epg::TaskConfig &task)
{
    return CpuBindingApplyEnabled() &&
           task.scheduling.resource.rfind("cpu", 0) == 0;
}

bool CanApplyCpuBindingAffinity(
    const Epg::TaskConfig &task,
    const std::set<std::string> &preserveAccuracyTasks)
{
    return ShouldApplyCpuBindingAffinity(task) &&
           preserveAccuracyTasks.find(task.name) == preserveAccuracyTasks.end();
}

void RefreshTopologyDecisionReason(Epg::SolverReportDecision &decision)
{
    const auto stats = Solver::TaskMetricsFromDecision(decision);
    decision.reason = Solver::BuildTaskDecisionReason(
        {&stats,
         decision.intervalBeforeMs,
         decision.intervalAfterMs,
         decision.effectiveLoopUs,
         decision.targetUtilizationPpm,
         decision.budgetUs,
         decision.deadlineUs,
         Solver::DefaultSolverLimits().resourceWaitPressureUs,
         &decision.resourceBefore,
         &decision.resourceAfter,
         &decision.backpressureBefore,
         &decision.backpressureAfter,
         decision.replaceable,
         decision.cpuAffinityBefore,
         decision.cpuAffinityAfter,
         decision.cpuBindingAffinity});
}

void ApplyCpuBindingEntry(Epg::TaskConfig &task,
                          Epg::SolverReportDecision &decision,
                          const Solver::CpuBindingScheduleEntry &entry,
                          const Solver::CpuBindingSchedule &schedule,
                          const std::set<std::string> &preserveAccuracyTasks)
{
    decision.cpuBindingAffinity = entry.cpuAffinity;
    decision.cpuBindingStartMs = entry.startMs;
    decision.cpuBindingFinishMs = entry.finishMs;
    decision.cpuBindingMakespanMs = schedule.makespanMs;
    if (CanApplyCpuBindingAffinity(task, preserveAccuracyTasks)) {
        task.scheduling.cpuAffinity = entry.cpuAffinity;
        decision.cpuAffinityAfter = entry.cpuAffinity;
    }
    RefreshTopologyDecisionReason(decision);
}

} // namespace

void ApplyEpgOptimizerCpuBindingSchedule(
    Epg::GraphConfig &config,
    std::vector<Epg::SolverReportDecision> &decisions,
    const EpgSolverPrimitives::CpuBindingSchedule &schedule,
    const std::set<std::string> &preserveAccuracyTasks)
{
    for (auto &task : config.tasks) {
        const auto *entry = FindCpuBindingEntry(schedule, task.name);
        if (!entry) {
            continue;
        }
        auto *decision =
            Solver::FindSolverReportTaskDecision(decisions, task.name);
        if (!decision) {
            continue;
        }
        ApplyCpuBindingEntry(task, *decision, *entry, schedule,
                             preserveAccuracyTasks);
    }
}

} // namespace SmartDrone::Core::Application
