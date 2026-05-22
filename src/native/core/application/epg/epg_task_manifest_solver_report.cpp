#include "core/application/epg/epg_task_manifest.h"
#include "core/application/epg/epg_task_manifest_internal.h"

#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace SmartDrone::Core::Application {
namespace {

using EpgTaskManifestInternal::GLOBAL_TOPOLOGY_OBJECTIVE;
using EpgTaskManifestInternal::RequireCatalogEntry;

std::uint64_t SolverReportTotalPenalty(
    const Epg::SolverReportScore &score)
{
    return score.queuePressure * 1000 + score.periodicOverloadUs +
           score.resourceWaitUs + score.schedulingErrors * 10000 +
           score.budgetOverruns * 2000 + score.deadlineMisses * 5000 +
           score.utilizationOverPpm;
}

bool SolverReportUsesGlobalObjective(const Epg::SolverReport &report)
{
    return report.objectiveName == GLOBAL_TOPOLOGY_OBJECTIVE;
}

std::uint64_t CeilDiv(std::uint64_t numerator, std::uint64_t denominator)
{
    if (denominator == 0) {
        return numerator;
    }
    return (numerator + denominator - 1) / denominator;
}

std::uint64_t TaskPeriodicOverloadUs(std::uint64_t intervalAfterMs,
                                     std::uint64_t effectiveLoopUs)
{
    const std::uint64_t intervalUs = intervalAfterMs * 1000;
    if (intervalUs == 0 || effectiveLoopUs <= intervalUs) {
        return 0;
    }
    return effectiveLoopUs - intervalUs;
}

std::uint64_t SolverTaskPeriodicOverloadUs(
    const Epg::SolverReportDecision &decision)
{
    return TaskPeriodicOverloadUs(decision.intervalAfterMs,
                                  decision.effectiveLoopUs);
}

std::uint64_t TaskUtilizationOverPpm(std::uint64_t intervalBeforeMs,
                                     std::uint64_t intervalAfterMs,
                                     std::uint64_t utilizationPpm,
                                     std::uint64_t targetUtilizationPpm)
{
    if (intervalAfterMs == 0 || utilizationPpm <= targetUtilizationPpm) {
        return 0;
    }
    const auto scaledUtilization =
        CeilDiv(utilizationPpm * intervalBeforeMs, intervalAfterMs);
    if (scaledUtilization <= targetUtilizationPpm) {
        return 0;
    }
    return scaledUtilization - targetUtilizationPpm;
}

std::uint64_t SolverTaskUtilizationOverPpm(
    const Epg::SolverReportDecision &decision)
{
    return TaskUtilizationOverPpm(decision.intervalBeforeMs,
                                  decision.intervalAfterMs,
                                  decision.utilizationPpm,
                                  decision.targetUtilizationPpm);
}

std::uint64_t QueuePressureAtDepth(
    std::uint64_t depth,
    const Epg::QueueProfileMetrics &stats)
{
    const auto depthPressure =
        stats.maxDepthObserved > depth ? stats.maxDepthObserved - depth : 0;
    return depthPressure + stats.droppedNewest + stats.overwrittenOldest;
}

std::uint64_t QueueCandidatePenalty(
    std::uint64_t depth,
    const Epg::QueueProfileMetrics &stats)
{
    return QueuePressureAtDepth(depth, stats) * 1000 + depth;
}

Epg::QueueProfileMetrics QueueMetricsFromDecision(
    const Epg::SolverReportDecision &decision)
{
    Epg::QueueProfileMetrics stats;
    stats.maxDepthObserved = decision.maxDepthObserved;
    stats.droppedNewest = decision.droppedNewest;
    stats.overwrittenOldest = decision.overwrittenOldest;
    stats.pushedPerSecond = decision.pushedPerSecond;
    stats.poppedPerSecond = decision.poppedPerSecond;
    stats.droppedPerSecond = decision.droppedPerSecond;
    if (stats.maxDepthObserved == 0 && decision.pressureBefore > 0) {
        stats.maxDepthObserved =
            decision.depthBefore + decision.pressureBefore;
    }
    return stats;
}

std::uint64_t TaskCandidatePenalty(
    const Epg::SolverReportDecision &decision,
    std::uint64_t intervalMs,
    const Epg::TaskProfileMetrics &stats,
    std::uint64_t effectiveLoopUs)
{
    return TaskPeriodicOverloadUs(intervalMs, effectiveLoopUs) +
           stats.totalResourceWaitUs +
           stats.schedulingErrorCount * 10000 +
           stats.budgetOverrunCount * 2000 +
           stats.deadlineMissCount * 5000 +
           TaskUtilizationOverPpm(decision.intervalBeforeMs, intervalMs,
                                  stats.utilizationPpm,
                                  decision.targetUtilizationPpm) +
           intervalMs;
}

std::uint64_t TaskFeasibleIntervalLimit(
    const Epg::SolverReportConstraints &constraints,
    const Epg::TaskProfileMetrics &stats,
    std::uint64_t effectiveLoopUs,
    const Epg::SolverReportDecision &decision)
{
    std::uint64_t limit = decision.intervalBeforeMs;
    if (effectiveLoopUs > decision.intervalBeforeMs * 1000) {
        limit = std::max(limit, CeilDiv(effectiveLoopUs, 1000));
    }
    if (stats.utilizationPpm > decision.targetUtilizationPpm) {
        limit = std::max(
            limit,
            CeilDiv(decision.intervalBeforeMs * stats.utilizationPpm,
                    decision.targetUtilizationPpm));
    }
    return std::min(std::max(limit, decision.intervalBeforeMs),
                    constraints.maxPeriodicIntervalMs);
}

std::uint64_t BestQueuePenalty(
    const Epg::SolverReportConstraints &constraints,
    const Epg::QueueProfileMetrics &stats,
    const Epg::SolverReportDecision &decision)
{
    const auto minDepth = std::max<std::uint64_t>(1, decision.depthBefore);
    const auto maxDepth = std::max(minDepth, constraints.maxQueueDepth);
    auto bestPenalty = QueueCandidatePenalty(minDepth, stats);
    for (std::uint64_t depth = minDepth + 1; depth <= maxDepth; ++depth) {
        bestPenalty =
            std::min(bestPenalty, QueueCandidatePenalty(depth, stats));
    }
    return bestPenalty;
}

std::uint64_t BestTaskPenalty(
    const Epg::SolverReportConstraints &constraints,
    const Epg::TaskProfileMetrics &stats,
    const Epg::SolverReportDecision &decision)
{
    if (!decision.replaceable || decision.intervalBeforeMs == 0) {
        return TaskCandidatePenalty(
            decision, decision.intervalBeforeMs, stats,
            decision.effectiveLoopUs);
    }
    const auto maxInterval =
        TaskFeasibleIntervalLimit(constraints, stats,
                                  decision.effectiveLoopUs, decision);
    auto bestPenalty =
        TaskCandidatePenalty(
            decision, decision.intervalBeforeMs, stats,
            decision.effectiveLoopUs);
    for (std::uint64_t interval = decision.intervalBeforeMs + 1;
         interval <= maxInterval; ++interval) {
        const auto penalty = TaskCandidatePenalty(
            decision, interval, stats, decision.effectiveLoopUs);
        bestPenalty = std::min(bestPenalty, penalty);
    }
    return bestPenalty;
}

void AddQueueDecisionScore(const Epg::SolverReportDecision &decision,
                           Epg::SolverReportScore &score)
{
    score.queuePressure += decision.pressureAfter;
}

void AddTaskDecisionScore(const Epg::SolverReportDecision &decision,
                          Epg::SolverReportScore &score)
{
    score.periodicOverloadUs += SolverTaskPeriodicOverloadUs(decision);
    score.resourceWaitUs += decision.totalResourceWaitUs;
    score.schedulingErrors += decision.schedulingErrorCount;
    score.budgetOverruns += decision.budgetOverrunCount;
    score.deadlineMisses += decision.deadlineMissCount;
    score.utilizationOverPpm += SolverTaskUtilizationOverPpm(decision);
}

Epg::SolverReportScore BuildSolverReportScore(
    const std::vector<Epg::SolverReportDecision> &decisions)
{
    Epg::SolverReportScore score;
    for (const auto &decision : decisions) {
        if (decision.kind == "queue") {
            AddQueueDecisionScore(decision, score);
            continue;
        }
        AddTaskDecisionScore(decision, score);
    }
    score.totalPenalty = SolverReportTotalPenalty(score);
    return score;
}

const Epg::SolverReportDecision &RequireDecision(
    const Epg::SolverReport &report,
    const std::string &kind,
    const std::string &name)
{
    for (const auto &decision : report.decisions) {
        if (decision.kind == kind && decision.name == name) {
            return decision;
        }
    }
    throw std::runtime_error("solver report decision missing: " + kind + ":" +
                             name);
}

Epg::TaskProfileMetrics TaskMetricsFromDecision(
    const Epg::SolverReportDecision &decision)
{
    Epg::TaskProfileMetrics metrics;
    metrics.maxLoopUs = decision.maxLoopUs;
    metrics.averageLoopUs = decision.averageLoopUs;
    metrics.p90LoopUs = decision.p90LoopUs;
    metrics.p99LoopUs = decision.p99LoopUs;
    metrics.resourceWaitCount = decision.resourceWaitCount;
    metrics.maxResourceWaitUs = decision.maxResourceWaitUs;
    metrics.averageResourceWaitUs = decision.averageResourceWaitUs;
    metrics.totalResourceWaitUs = decision.totalResourceWaitUs;
    metrics.utilizationPpm = decision.utilizationPpm;
    metrics.budgetOverrunCount = decision.budgetOverrunCount;
    metrics.deadlineMissCount = decision.deadlineMissCount;
    metrics.schedulingErrorCount = decision.schedulingErrorCount;
    return metrics;
}

bool TaskDecisionMetricsMatch(const Epg::SolverReportDecision &decision,
                              const Epg::TaskProfileMetrics &stats)
{
    return decision.maxLoopUs == stats.maxLoopUs &&
           decision.averageLoopUs == stats.averageLoopUs &&
           decision.p90LoopUs == stats.p90LoopUs &&
           decision.p99LoopUs == stats.p99LoopUs &&
           decision.resourceWaitCount == stats.resourceWaitCount &&
           decision.maxResourceWaitUs == stats.maxResourceWaitUs &&
           decision.averageResourceWaitUs == stats.averageResourceWaitUs &&
           decision.totalResourceWaitUs == stats.totalResourceWaitUs &&
           decision.utilizationPpm == stats.utilizationPpm &&
           decision.budgetOverrunCount == stats.budgetOverrunCount &&
           decision.deadlineMissCount == stats.deadlineMissCount &&
           decision.schedulingErrorCount == stats.schedulingErrorCount;
}

void ValidateQueueGlobalOptimum(
    const Epg::SolverReport &report,
    const Epg::QueueConfig &queue,
    const Epg::QueueProfileMetrics *profileStats)
{
    const auto &decision = RequireDecision(report, "queue", queue.name);
    const auto decisionStats = QueueMetricsFromDecision(decision);
    const auto &stats = profileStats ? *profileStats : decisionStats;
    if (profileStats && (decision.maxDepthObserved != stats.maxDepthObserved ||
                         decision.droppedNewest != stats.droppedNewest ||
                         decision.overwrittenOldest !=
                             stats.overwrittenOldest)) {
        throw std::runtime_error("solver report queue metrics mismatch: " +
                                 queue.name);
    }
    const auto expectedPressureBefore =
        QueuePressureAtDepth(decision.depthBefore, stats);
    const auto expectedPressureAfter =
        QueuePressureAtDepth(decision.depthAfter, stats);
    if (decision.pressureBefore != expectedPressureBefore ||
        decision.pressureAfter != expectedPressureAfter ||
        decision.pushedPerSecond != stats.pushedPerSecond ||
        decision.poppedPerSecond != stats.poppedPerSecond ||
        decision.droppedPerSecond != stats.droppedPerSecond) {
        throw std::runtime_error("solver report queue metrics mismatch: " +
                                 queue.name);
    }
    const auto actualPenalty =
        QueueCandidatePenalty(decision.depthAfter, stats);
    const auto bestPenalty =
        BestQueuePenalty(report.constraints, stats, decision);
    if (actualPenalty != bestPenalty) {
        throw std::runtime_error("solver report queue is not optimal: " +
                                 queue.name);
    }
}

void ValidateTaskGlobalOptimum(
    const Epg::SolverReport &report,
    const Epg::TaskConfig &task,
    const Epg::TaskProfileMetrics *profileStats)
{
    const auto &decision = RequireDecision(report, "task", task.name);
    const auto decisionStats = TaskMetricsFromDecision(decision);
    const auto &stats = profileStats ? *profileStats : decisionStats;
    const auto effectiveLoopUs =
        std::max({stats.p99LoopUs, stats.p90LoopUs, stats.maxLoopUs,
                  stats.averageLoopUs});
    if (profileStats && !TaskDecisionMetricsMatch(decision, stats)) {
        throw std::runtime_error("solver report task metrics mismatch: " +
                                 task.name);
    }
    if (decision.effectiveLoopUs != effectiveLoopUs) {
        throw std::runtime_error("solver report task metrics mismatch: " +
                                 task.name);
    }
    const auto actualPenalty = TaskCandidatePenalty(
        decision, decision.intervalAfterMs, stats, effectiveLoopUs);
    const auto bestPenalty =
        BestTaskPenalty(report.constraints, stats, decision);
    if (actualPenalty != bestPenalty) {
        throw std::runtime_error("solver report task is not optimal: " +
                                 task.name);
    }
}

void ValidateSolverReportGlobalOptimum(
    const Epg::OptimizedGraph &optimizedGraph,
    const Epg::SolverReport &report,
    const Epg::GraphProfile *sourceProfile)
{
    if (!SolverReportUsesGlobalObjective(report)) {
        throw std::runtime_error("solver report objective mismatch");
    }
    for (const auto &queue : optimizedGraph.config.queues) {
        const Epg::QueueProfileMetrics *stats = nullptr;
        if (sourceProfile) {
            stats = &sourceProfile->diagnostics.queues.at(queue.name);
        }
        ValidateQueueGlobalOptimum(report, queue, stats);
    }
    for (const auto &task : optimizedGraph.config.tasks) {
        const Epg::TaskProfileMetrics *stats = nullptr;
        if (sourceProfile) {
            stats = &sourceProfile->diagnostics.tasks.at(task.name);
        }
        ValidateTaskGlobalOptimum(report, task, stats);
    }
}

void ValidateSolverReportScore(const Epg::SolverReport &report)
{
    const auto expected = BuildSolverReportScore(report.decisions);
    if (report.score.queuePressure != expected.queuePressure ||
        report.score.periodicOverloadUs != expected.periodicOverloadUs ||
        report.score.resourceWaitUs != expected.resourceWaitUs ||
        report.score.schedulingErrors != expected.schedulingErrors ||
        report.score.budgetOverruns != expected.budgetOverruns ||
        report.score.deadlineMisses != expected.deadlineMisses ||
        report.score.utilizationOverPpm != expected.utilizationOverPpm ||
        report.score.totalPenalty != expected.totalPenalty) {
        throw std::runtime_error("solver report score mismatch");
    }
}

void ValidateSolverReportDecisionCoverage(
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

const Epg::QueueConfig *FindQueueConfig(
    const Epg::GraphConfig &graphConfig,
    const std::string &name)
{
    for (const auto &queue : graphConfig.queues) {
        if (queue.name == name) {
            return &queue;
        }
    }
    return nullptr;
}

const Epg::TaskConfig *FindTaskConfig(
    const Epg::GraphConfig &graphConfig,
    const std::string &name)
{
    for (const auto &task : graphConfig.tasks) {
        if (task.name == name) {
            return &task;
        }
    }
    return nullptr;
}

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
    const Epg::SolverReportConstraints &constraints,
    const Epg::SolverReportDecision &decision)
{
    const auto *queue = FindQueueConfig(graphConfig, decision.name);
    if (!queue) {
        throw std::runtime_error(
            "solver report queue decision target missing: " + decision.name);
    }
    if (decision.depthAfter != static_cast<std::uint64_t>(queue->depth)) {
        throw std::runtime_error("solver report queue depth mismatch: " +
                                 decision.name);
    }
    if (decision.depthAfter == 0 ||
        decision.depthAfter > constraints.maxQueueDepth) {
        throw std::runtime_error("solver report queue constraint mismatch: " +
                                 decision.name);
    }
    if (sourceGraphConfig) {
        const auto *sourceQueue = FindQueueConfig(*sourceGraphConfig,
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

std::vector<std::string> TaskDecisionReasons(
    const Epg::SolverReportDecision &decision)
{
    std::vector<std::string> reasons;
    if (decision.utilizationPpm > decision.targetUtilizationPpm) {
        reasons.push_back("utilization_over_target");
    }
    if (decision.budgetOverrunCount > 0 ||
        (decision.budgetUs > 0 &&
         decision.effectiveLoopUs > decision.budgetUs)) {
        reasons.push_back("budget_overrun");
    }
    if (decision.deadlineMissCount > 0 ||
        (decision.deadlineUs > 0 &&
         decision.effectiveLoopUs > decision.deadlineUs)) {
        reasons.push_back("deadline_miss");
    }
    if (decision.schedulingErrorCount > 0) {
        reasons.push_back("scheduling_error");
    }
    if (decision.maxResourceWaitUs > 1000 ||
        decision.averageResourceWaitUs > 1000 ||
        decision.totalResourceWaitUs > 1000) {
        reasons.push_back("resource_wait");
    }
    return reasons;
}

std::string JoinTaskDecisionReasons(const std::vector<std::string> &reasons)
{
    if (reasons.empty()) {
        return "keep";
    }
    std::string reason = reasons.front();
    for (std::size_t index = 1; index < reasons.size(); ++index) {
        reason += "+" + reasons[index];
    }
    return reason;
}

std::string ExpectedTaskDecisionReason(
    const Epg::SolverReportDecision &decision)
{
    if (decision.intervalAfterMs != decision.intervalBeforeMs) {
        return "global_optimum_interval";
    }
    const auto reason = JoinTaskDecisionReasons(TaskDecisionReasons(decision));
    if (decision.replaceable) {
        return reason;
    }
    return reason == "keep" ? "not_replaceable" : "not_replaceable+" + reason;
}

void ValidateTaskSolverDecision(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig *sourceGraphConfig,
    const Epg::GraphConfig &graphConfig,
    const Epg::SolverReportConstraints &constraints,
    const Epg::SolverReportDecision &decision)
{
    const auto *task = FindTaskConfig(graphConfig, decision.name);
    if (!task) {
        throw std::runtime_error(
            "solver report task decision target missing: " + decision.name);
    }
    const auto &catalog = RequireCatalogEntry(manifest, task->type);
    if (decision.intervalAfterMs > constraints.maxPeriodicIntervalMs) {
        throw std::runtime_error(
            "solver report task interval constraint mismatch: " +
            decision.name);
    }
    if (decision.intervalAfterMs !=
            static_cast<std::uint64_t>(task->trigger.interval.count()) ||
        decision.budgetUs != task->scheduling.budgetUs ||
        decision.deadlineUs != task->scheduling.deadlineUs ||
        decision.targetUtilizationPpm != constraints.targetUtilizationPpm) {
        throw std::runtime_error("solver report task decision mismatch: " +
                                 decision.name);
    }
    if (sourceGraphConfig) {
        const auto *sourceTask = FindTaskConfig(*sourceGraphConfig,
                                                decision.name);
        if (!sourceTask ||
            decision.intervalBeforeMs !=
                static_cast<std::uint64_t>(
                    sourceTask->trigger.interval.count())) {
            throw std::runtime_error("solver report task source mismatch: " +
                                     decision.name);
        }
    }
    if (decision.catalogRole != catalog.role ||
        decision.replaceable != catalog.replaceable) {
        throw std::runtime_error("solver report task catalog mismatch: " +
                                 decision.name);
    }
    if (decision.reason != ExpectedTaskDecisionReason(decision)) {
        throw std::runtime_error("solver report task reason mismatch: " +
                                 decision.name);
    }
}

void ValidateSolverReportDecisionDetails(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig *sourceGraphConfig,
    const Epg::GraphConfig &graphConfig,
    const Epg::SolverReport &report)
{
    ValidateSolverReportConstraints(report);
    for (const auto &decision : report.decisions) {
        if (decision.kind == "queue") {
            ValidateQueueSolverDecision(
                sourceGraphConfig, graphConfig, report.constraints, decision);
            continue;
        }
        ValidateTaskSolverDecision(
            manifest, sourceGraphConfig, graphConfig, report.constraints,
            decision);
    }
}

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

void ValidateEpgSolverReportProfile(
    const EpgTaskManifest &manifest,
    const Epg::GraphProfileMetadata &profileMetadata,
    const Epg::OptimizedGraphMetadata &optimizedMetadata,
    const Epg::SolverReportMetadata &reportMetadata)
{
    if (profileMetadata.schema != Epg::GRAPH_PROFILE_SCHEMA) {
        throw std::runtime_error("solver report profile schema mismatch");
    }
    if (profileMetadata.graph != manifest.subgraphName ||
        profileMetadata.graph != optimizedMetadata.sourceProfile ||
        profileMetadata.graph != reportMetadata.sourceProfile) {
        throw std::runtime_error("solver report profile graph mismatch");
    }
    if (profileMetadata.topologyVersion != manifest.topologyVersion ||
        profileMetadata.topologyVersion != optimizedMetadata.topologyVersion ||
        profileMetadata.topologyVersion != reportMetadata.topologyVersion) {
        throw std::runtime_error("solver report profile topology mismatch");
    }
    if (profileMetadata.timestampMs == 0 ||
        profileMetadata.timestampMs != optimizedMetadata.sourceTimestampMs ||
        profileMetadata.timestampMs != reportMetadata.sourceTimestampMs) {
        throw std::runtime_error("solver report profile timestamp mismatch");
    }
}

} // namespace

void ValidateEpgSolverReportManifest(
    const EpgTaskManifest &manifest,
    const Epg::OptimizedGraphMetadata &optimizedMetadata,
    const Epg::SolverReportMetadata &reportMetadata)
{
    if (reportMetadata.schema != Epg::SOLVER_REPORT_SCHEMA) {
        throw std::runtime_error("solver report schema mismatch");
    }
    if (reportMetadata.targetGraph != manifest.subgraphName ||
        reportMetadata.targetGraph != optimizedMetadata.targetGraph) {
        throw std::runtime_error("solver report target mismatch");
    }
    if (reportMetadata.topologyVersion != manifest.topologyVersion ||
        reportMetadata.topologyVersion != optimizedMetadata.topologyVersion) {
        throw std::runtime_error("solver report topology version mismatch");
    }
    if (reportMetadata.sourceProfile != optimizedMetadata.sourceProfile ||
        reportMetadata.sourceProfile != manifest.subgraphName) {
        throw std::runtime_error("solver report source profile mismatch");
    }
    if (reportMetadata.solverVersion != optimizedMetadata.solverVersion) {
        throw std::runtime_error("solver report version mismatch");
    }
    if (reportMetadata.sourceTimestampMs !=
            optimizedMetadata.sourceTimestampMs ||
        reportMetadata.generatedAtMs != optimizedMetadata.generatedAtMs) {
        throw std::runtime_error("solver report provenance mismatch");
    }
}

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
    ValidateSolverReportScore(report);
    ValidateSolverReportDecisionCoverage(optimizedGraph.config, report);
    ValidateSolverReportDecisionDetails(
        manifest, nullptr, optimizedGraph.config, report);
    if (SolverReportUsesGlobalObjective(report)) {
        ValidateSolverReportGlobalOptimum(optimizedGraph, report, nullptr);
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
    ValidateSolverReportScore(report);
    ValidateSolverReportDecisionCoverage(optimizedGraph.config, report);
    ValidateSolverReportDecisionDetails(
        manifest, &sourceProfile.topology, optimizedGraph.config, report);
    if (SolverReportUsesGlobalObjective(report)) {
        ValidateSolverReportGlobalOptimum(optimizedGraph, report,
                                          &sourceProfile);
    }
}

} // namespace SmartDrone::Core::Application
