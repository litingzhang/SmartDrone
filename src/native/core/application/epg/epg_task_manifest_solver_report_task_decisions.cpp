#include "core/application/epg/epg_task_manifest_solver_report_task_decisions.h"

#include "core/application/epg/epg_solver_primitives.h"
#include "core/application/epg/epg_task_manifest.h"
#include "core/application/epg/epg_task_manifest_internal.h"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace SmartDrone::Core::Application {
namespace {

namespace Solver = EpgSolverPrimitives;

using EpgTaskManifestInternal::RequireCatalogEntry;

bool StringVectorContains(const std::vector<std::string> &values,
                          const std::string &value)
{
    return std::find(values.begin(), values.end(), value) != values.end();
}

bool PreserveAccuracySchedulingAllowed(
    const EpgTaskCatalogEntry &catalog,
    const Epg::SolverReportDecision &decision,
    const Epg::TaskConfig &task,
    const Epg::TaskConfig &sourceTask)
{
    if (!catalog.preserveAccuracy) {
        return true;
    }
    return decision.intervalAfterMs == decision.intervalBeforeMs &&
           decision.resourceAfter == decision.resourceBefore &&
           decision.backpressureAfter == decision.backpressureBefore &&
           task.scheduling.realtime == sourceTask.scheduling.realtime &&
           task.scheduling.priority == sourceTask.scheduling.priority;
}

std::string ExpectedTaskDecisionReason(
    const Epg::SolverReportDecision &decision)
{
    const auto stats = Solver::TaskMetricsFromDecision(decision);
    return Solver::BuildTaskDecisionReason(
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

const Epg::TaskConfig &ValidateTaskSourceDecision(
    const Epg::GraphConfig &sourceGraphConfig,
    const Epg::SolverReportDecision &decision)
{
    const auto *sourceTask =
        Solver::FindTaskConfig(sourceGraphConfig, decision.name);
    if (!sourceTask ||
        decision.intervalBeforeMs !=
            static_cast<std::uint64_t>(
                sourceTask->trigger.interval.count()) ||
        decision.resourceBefore != sourceTask->scheduling.resource ||
        decision.cpuAffinityBefore != sourceTask->scheduling.cpuAffinity ||
        decision.backpressureBefore !=
            Solver::SortedUniquePorts(
                sourceTask->scheduling.backpressureOutputs)) {
        throw std::runtime_error("solver report task source mismatch: " +
                                 decision.name);
    }
    return *sourceTask;
}

void ValidateTaskSchedulingDecision(
    const Epg::TaskConfig &task,
    const Epg::SolverReportConstraints &constraints,
    const Epg::SolverReportDecision &decision)
{
    if (decision.intervalAfterMs > constraints.maxPeriodicIntervalMs) {
        throw std::runtime_error(
            "solver report task interval constraint mismatch: " +
            decision.name);
    }
    if (decision.intervalAfterMs !=
            static_cast<std::uint64_t>(task.trigger.interval.count()) ||
        decision.budgetUs != task.scheduling.budgetUs ||
        decision.deadlineUs != task.scheduling.deadlineUs ||
        decision.resourceAfter != task.scheduling.resource ||
        decision.cpuAffinityAfter != task.scheduling.cpuAffinity ||
        decision.targetUtilizationPpm != constraints.targetUtilizationPpm ||
        decision.backpressureAfter !=
            Solver::SortedUniquePorts(task.scheduling.backpressureOutputs)) {
        throw std::runtime_error("solver report task decision mismatch: " +
                                 decision.name);
    }
}

void ValidateTaskScheduleMetadata(
    const Epg::TaskConfig &task,
    const Epg::SolverReportDecision &decision)
{
    if (task.scheduling.phaseOffsetConfigured &&
        (decision.topologyLevel != task.scheduling.topologyLevel ||
         decision.phaseOffsetMs != task.scheduling.phaseOffsetMs)) {
        throw std::runtime_error("solver report task schedule mismatch: " +
                                 decision.name);
    }
    if (decision.durationMs != Solver::CeilDiv(decision.effectiveLoopUs, 1000)) {
        throw std::runtime_error("solver report task schedule mismatch: " +
                                 decision.name);
    }
}

void ValidateTaskCatalogDecision(
    const EpgTaskCatalogEntry &catalog,
    const Epg::SolverReportDecision &decision)
{
    if (decision.catalogRole != catalog.role ||
        decision.replaceable != catalog.replaceable) {
        throw std::runtime_error("solver report task catalog mismatch: " +
                                 decision.name);
    }
    if (decision.resourceAfter != catalog.resource &&
        !StringVectorContains(catalog.resourceAlternates,
                              decision.resourceAfter)) {
        throw std::runtime_error("solver report task resource mismatch: " +
                                 decision.name);
    }
}

void ValidateTaskAccuracyDecision(
    const EpgTaskCatalogEntry &catalog,
    const Epg::GraphConfig *sourceGraphConfig,
    const Epg::TaskConfig &task,
    const Epg::SolverReportDecision &decision)
{
    if (!sourceGraphConfig) {
        return;
    }
    const auto &sourceTask =
        ValidateTaskSourceDecision(*sourceGraphConfig, decision);
    if (!PreserveAccuracySchedulingAllowed(catalog, decision, task,
                                           sourceTask)) {
        throw std::runtime_error(
            "solver report task accuracy constraint mismatch: " +
            decision.name);
    }
}

} // namespace

void ValidateEpgTaskSolverReportDecision(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig *sourceGraphConfig,
    const Epg::GraphConfig &graphConfig,
    const Epg::SolverReportConstraints &constraints,
    const Epg::SolverReportDecision &decision)
{
    const auto *task = Solver::FindTaskConfig(graphConfig, decision.name);
    if (!task) {
        throw std::runtime_error(
            "solver report task decision target missing: " + decision.name);
    }
    const auto &catalog = RequireCatalogEntry(manifest, task->type);
    ValidateTaskSchedulingDecision(*task, constraints, decision);
    ValidateTaskScheduleMetadata(*task, decision);
    ValidateTaskAccuracyDecision(catalog, sourceGraphConfig, *task, decision);
    ValidateTaskCatalogDecision(catalog, decision);
    if (decision.reason != ExpectedTaskDecisionReason(decision)) {
        throw std::runtime_error("solver report task reason mismatch: " +
                                 decision.name);
    }
}

} // namespace SmartDrone::Core::Application
