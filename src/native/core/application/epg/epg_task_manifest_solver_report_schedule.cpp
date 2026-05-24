#include "core/application/epg/epg_task_manifest_solver_report_schedule.h"

#include "core/application/epg/epg_solver_primitives.h"

#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace SmartDrone::Core::Application {
namespace {

namespace Solver = EpgSolverPrimitives;

const auto SOLVER_LIMITS = Solver::DefaultSolverLimits();

std::map<std::string, std::uint64_t> TaskDurationsFromReport(
    const Epg::SolverReport &report)
{
    std::map<std::string, std::uint64_t> durations;
    for (const auto &decision : report.decisions) {
        if (decision.kind == "task") {
            durations[decision.name] = decision.durationMs;
        }
    }
    return durations;
}

bool GraphConfigHasTopologySchedule(const Epg::GraphConfig &graphConfig)
{
    for (const auto &task : graphConfig.tasks) {
        if (task.scheduling.phaseOffsetConfigured) {
            return true;
        }
    }
    return false;
}

std::size_t HardwareCpuCount()
{
    const auto detected = sysconf(_SC_NPROCESSORS_ONLN);
    if (detected <= 0) {
        return 1;
    }
    return std::min<std::size_t>(static_cast<std::size_t>(detected),
                                 SOLVER_LIMITS.maxCpuBindingCores);
}

bool ShouldApplyCpuBindingAffinity(const Epg::TaskConfig &task)
{
    return task.scheduling.cpuAffinity >= 0 &&
           task.scheduling.resource.rfind("cpu", 0) == 0;
}

const Epg::SolverReportDecision &RequireTaskDecision(
    const Epg::SolverReport &report,
    const std::string &taskName)
{
    for (const auto &decision : report.decisions) {
        if (decision.kind == "task" && decision.name == taskName) {
            return decision;
        }
    }
    throw std::runtime_error("solver report decision missing: task:" +
                             taskName);
}

} // namespace

void ValidateEpgSolverReportCpuBindingSchedule(
    const Epg::GraphConfig &graphConfig,
    const Epg::SolverReport &report)
{
    const auto schedule = Solver::BuildCpuBindingSchedule(
        graphConfig, TaskDurationsFromReport(report),
        HardwareCpuCount(), SOLVER_LIMITS.maxExactCpuBindingStates);
    if (schedule.entries.empty() && !graphConfig.tasks.empty()) {
        throw std::runtime_error("solver report CPU binding missing");
    }
    for (const auto &entry : schedule.entries) {
        const auto &decision = RequireTaskDecision(report, entry.name);
        const auto *task = Solver::FindTaskConfig(graphConfig, entry.name);
        const auto expectedAppliedAffinity =
            task && ShouldApplyCpuBindingAffinity(*task) ? entry.cpuAffinity : -1;
        if (decision.cpuBindingAffinity != entry.cpuAffinity ||
            decision.cpuAffinityAfter != expectedAppliedAffinity ||
            decision.cpuBindingStartMs != entry.startMs ||
            decision.cpuBindingFinishMs != entry.finishMs ||
            decision.cpuBindingMakespanMs != schedule.makespanMs) {
            throw std::runtime_error("solver report CPU binding mismatch: " +
                                     entry.name);
        }
    }
}

void ValidateEpgSolverReportTopologySchedule(
    const Epg::GraphConfig &graphConfig,
    const Epg::SolverReport &report)
{
    if (!GraphConfigHasTopologySchedule(graphConfig)) {
        return;
    }
    const auto expected = Solver::BuildTaskTopologySchedule(
        graphConfig, TaskDurationsFromReport(report));
    for (const auto &entry : expected.entries) {
        const auto *task = Solver::FindTaskConfig(graphConfig, entry.name);
        if (!task) {
            throw std::runtime_error(
                "solver report task schedule missing: " + entry.name);
        }
        if (task->scheduling.topologyLevel != entry.level ||
            !task->scheduling.phaseOffsetConfigured ||
            task->scheduling.phaseOffsetMs != entry.phaseOffsetMs) {
            throw std::runtime_error(
                "solver report topology schedule mismatch: " + entry.name);
        }
    }
}

} // namespace SmartDrone::Core::Application
