#include "core/application/epg/epg_task_manifest_solver_report_decisions.h"

#include "core/application/epg/epg_solver_primitives.h"
#include "core/application/epg/epg_task_manifest.h"
#include "core/application/epg/epg_task_manifest_solver_report_task_decisions.h"

#include <set>
#include <stdexcept>
#include <string>

namespace SmartDrone::Core::Application {
namespace {

namespace Solver = EpgSolverPrimitives;

void ValidateSolverReportConstraints(const Epg::SolverReport &report)
{
    const auto &constraints = report.constraints;
    if (constraints.maxQueueDepth == 0 ||
        constraints.maxPeriodicIntervalMs == 0 ||
        constraints.targetUtilizationPpm == 0) {
        throw std::runtime_error("solver report constraint invalid");
    }
}

void ValidateQueueSolverDecision(
    const Epg::GraphConfig *sourceGraphConfig,
    const Epg::GraphConfig &graphConfig,
    const Epg::SolverReportDecision &decision)
{
    const auto *queue = Solver::FindQueueConfig(graphConfig, decision.name);
    if (!queue) {
        throw std::runtime_error(
            "solver report queue decision target missing: " + decision.name);
    }
    if (decision.depthAfter != static_cast<std::uint64_t>(queue->depth)) {
        throw std::runtime_error("solver report queue depth mismatch: " +
                                 decision.name);
    }
    if (decision.depthAfter == 0) {
        throw std::runtime_error("solver report queue constraint mismatch: " +
                                 decision.name);
    }
    if (sourceGraphConfig) {
        const auto *sourceQueue = Solver::FindQueueConfig(*sourceGraphConfig,
                                                          decision.name);
        if (!sourceQueue ||
            decision.depthBefore !=
                static_cast<std::uint64_t>(sourceQueue->depth)) {
            throw std::runtime_error("solver report queue source mismatch: " +
                                     decision.name);
        }
    }
    const std::string expectedReason =
        decision.depthAfter != decision.depthBefore ? "global_optimum_depth"
                                                    : "keep";
    if (decision.reason != expectedReason) {
        throw std::runtime_error("solver report queue reason mismatch: " +
                                 decision.name);
    }
}

} // namespace

void ValidateEpgSolverReportScore(const Epg::SolverReport &report)
{
    const auto expected = Solver::BuildSolverReportScore(report.decisions);
    if (report.score.queuePressure != expected.queuePressure ||
        report.score.periodicOverloadUs != expected.periodicOverloadUs ||
        report.score.resourceWaitUs != expected.resourceWaitUs ||
        report.score.schedulingErrors != expected.schedulingErrors ||
        report.score.budgetOverruns != expected.budgetOverruns ||
        report.score.deadlineMisses != expected.deadlineMisses ||
        report.score.utilizationOverPpm != expected.utilizationOverPpm ||
        report.score.topologyPenalty != expected.topologyPenalty ||
        report.score.totalPenalty != expected.totalPenalty) {
        throw std::runtime_error("solver report score mismatch");
    }
}

void ValidateEpgSolverReportDecisionCoverage(
    const Epg::GraphConfig &graphConfig,
    const Epg::SolverReport &report)
{
    std::set<std::string> expected;
    for (const auto &queue : graphConfig.queues) {
        expected.insert("queue:" + queue.name);
    }
    for (const auto &task : graphConfig.tasks) {
        expected.insert("task:" + task.name);
    }
    std::set<std::string> actual;
    for (const auto &decision : report.decisions) {
        const auto key = decision.kind + ":" + decision.name;
        if (!actual.insert(key).second) {
            throw std::runtime_error("solver report duplicates decision: " +
                                     key);
        }
    }
    if (actual != expected) {
        throw std::runtime_error("solver report decision coverage mismatch");
    }
}

void ValidateEpgSolverReportDecisionDetails(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig *sourceGraphConfig,
    const Epg::GraphConfig &graphConfig,
    const Epg::SolverReport &report)
{
    ValidateSolverReportConstraints(report);
    for (const auto &decision : report.decisions) {
        if (decision.kind == "queue") {
            ValidateQueueSolverDecision(
                sourceGraphConfig, graphConfig, decision);
            continue;
        }
        ValidateEpgTaskSolverReportDecision(
            manifest, sourceGraphConfig, graphConfig, report.constraints,
            decision);
    }
}

} // namespace SmartDrone::Core::Application
