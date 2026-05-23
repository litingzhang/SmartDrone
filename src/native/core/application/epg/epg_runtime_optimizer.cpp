#include "core/application/epg/epg_runtime_optimizer.h"

#include <algorithm>
#include <chrono>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "core/application/epg/epg_solver_primitives.h"
#include "core/application/epg/epg_runtime_optimizer_io.h"
#include "core/application/epg/epg_task_manifest.h"
#include "core/application/runtime/epg_dfx_snapshot.h"
namespace SmartDrone::Core::Application {
namespace {
namespace Solver = EpgSolverPrimitives;
constexpr std::uint64_t PROFILE_FRESHNESS_MS = 60000;
constexpr std::size_t MAX_QUEUE_DEPTH = 16;
constexpr std::uint64_t MAX_PERIODIC_INTERVAL_MS = 1000;
constexpr std::uint64_t TARGET_UTILIZATION_PPM = 800000;
constexpr std::uint64_t RESOURCE_WAIT_PRESSURE_US = 1000;
constexpr int RESOURCE_ISOLATION_CPU_AFFINITY = 2;
constexpr int RESOURCE_ISOLATION_PRIORITY = 20;
constexpr std::uint64_t MAX_EXACT_TOPOLOGY_CANDIDATES = 200000;

struct SolverDecision {
    std::string kind;
    std::string name;
    std::string reason;
    std::string catalogRole;
    bool replaceable{false};
    std::uint64_t depthBefore{0}, depthAfter{0};
    std::uint64_t pressureBefore{0}, pressureAfter{0};
    std::uint64_t maxDepthObserved{0}, droppedNewest{0};
    std::uint64_t overwrittenOldest{0}, pushedPerSecond{0};
    std::uint64_t poppedPerSecond{0}, droppedPerSecond{0};
    std::uint64_t intervalBeforeMs{0}, intervalAfterMs{0};
    std::uint64_t maxLoopUs{0}, averageLoopUs{0}, p90LoopUs{0}, p99LoopUs{0};
    std::uint64_t effectiveLoopUs{0}, resourceWaitCount{0};
    std::uint64_t maxResourceWaitUs{0}, averageResourceWaitUs{0};
    std::uint64_t totalResourceWaitUs{0}, utilizationPpm{0};
    std::uint64_t predictedResourceWaitUs{0};
    std::uint64_t budgetUs{0}, deadlineUs{0};
    std::uint64_t budgetOverrunCount{0}, deadlineMissCount{0};
    std::uint64_t schedulingErrorCount{0}, topologyPenalty{0};
    std::string resourceBefore;
    std::string resourceAfter;
    std::vector<Epg::PortId> backpressureBefore;
    std::vector<Epg::PortId> backpressureAfter;
};
struct SolverScore {
    std::uint64_t queuePressure{0}, periodicOverloadUs{0};
    std::uint64_t resourceWaitUs{0}, schedulingErrors{0};
    std::uint64_t budgetOverruns{0}, deadlineMisses{0};
    std::uint64_t utilizationOverPpm{0}, topologyPenalty{0};
};
struct QueueCandidate {
    std::uint64_t depth{0}, pressureAfter{0}, penalty{0};
};
struct TaskCandidate {
    std::uint64_t intervalMs{0};
    std::string resource;
    std::vector<Epg::PortId> backpressureOutputs;
    std::uint64_t predictedResourceWaitUs{0};
    std::uint64_t topologyPenalty{0};
    std::uint64_t penalty{0};
};

struct QueueSolverNode {
    std::size_t index{0};
    std::uint64_t depthBefore{0}, pressureBefore{0};
    Epg::QueueProfileMetrics stats;
    std::vector<QueueCandidate> candidates;
};

struct TaskSolverNode {
    std::size_t index{0};
    std::uint64_t intervalBeforeMs{0}, effectiveLoopUs{0};
    const EpgTaskCatalogEntry *catalog{nullptr};
    Epg::TaskProfileMetrics stats;
    std::vector<TaskCandidate> candidates;
};

struct TopologySolution {
    std::vector<std::size_t> queueCandidateIndexes;
    std::vector<std::size_t> taskCandidateIndexes;
};

std::uint64_t ProfileAgeMs(std::uint64_t nowMs,
                           std::uint64_t profileTimestampMs)
{
    if (profileTimestampMs == 0 || nowMs < profileTimestampMs) {
        return PROFILE_FRESHNESS_MS + 1;
    }
    return nowMs - profileTimestampMs;
}

const EpgTaskCatalogEntry *FindCatalogEntry(const EpgTaskManifest &manifest,
                                            const std::string &taskType)
{
    for (const auto &entry : manifest.catalog) {
        if (entry.taskType == taskType) {
            return &entry;
        }
    }
    return nullptr;
}

const Epg::GraphProfileTaskCatalogEntry *FindProfileCatalogEntry(
    const Epg::GraphProfile &profile,
    const std::string &taskType)
{
    for (const auto &entry : profile.taskCatalog) {
        if (entry.taskType == taskType) {
            return &entry;
        }
    }
    return nullptr;
}

void ValidateProfileCatalogEntry(const EpgTaskCatalogEntry &manifestEntry,
                                 const Epg::GraphProfileTaskCatalogEntry &entry)
{
    if (entry.role != manifestEntry.role ||
        entry.resource != manifestEntry.resource ||
        entry.budgetUs != manifestEntry.budgetUs ||
        entry.deadlineUs != manifestEntry.deadlineUs ||
        entry.replaceable != manifestEntry.replaceable ||
        entry.resourceAlternates != manifestEntry.resourceAlternates) {
        throw std::runtime_error("profile task catalog mismatch: " +
                                 manifestEntry.taskType);
    }
}

void ValidateProfileCatalog(const EpgTaskManifest &manifest,
                            const Epg::GraphProfile &profile)
{
    if (profile.taskCatalog.size() != manifest.catalog.size()) {
        throw std::runtime_error("profile task catalog size mismatch");
    }
    for (const auto &entry : manifest.catalog) {
        const auto *profileEntry =
            FindProfileCatalogEntry(profile, entry.taskType);
    if (!profileEntry) {
        throw std::runtime_error("profile task catalog missing: " +
                                 entry.taskType);
    }
    ValidateProfileCatalogEntry(entry, *profileEntry);
    }
}

void ValidateProfileQueueDiagnostics(
    const Epg::GraphConfig &topology,
    const Epg::GraphProfileDiagnostics &diagnostics)
{
    for (const auto &queue : topology.queues) {
        if (diagnostics.queues.find(queue.name) != diagnostics.queues.end()) {
            continue;
        }
        throw std::runtime_error("profile diagnostics missing queue: " +
                                 queue.name);
    }
}

void ValidateProfileTaskDiagnostics(
    const Epg::GraphConfig &topology,
    const Epg::GraphProfileDiagnostics &diagnostics)
{
    for (const auto &task : topology.tasks) {
        if (diagnostics.tasks.find(task.name) != diagnostics.tasks.end()) {
            continue;
        }
        throw std::runtime_error("profile diagnostics missing task: " +
                                 task.name);
    }
}

void ValidateProfileDiagnosticsCoverage(
    const Epg::GraphConfig &topology,
    const Epg::GraphProfileDiagnostics &diagnostics)
{
    ValidateProfileQueueDiagnostics(topology, diagnostics);
    ValidateProfileTaskDiagnostics(topology, diagnostics);
}

std::vector<std::string> TaskResourceCandidates(
    const Epg::TaskConfig &task,
    const Epg::TaskProfileMetrics &stats,
    const EpgTaskCatalogEntry *catalog)
{
    std::vector<std::string> candidates{task.scheduling.resource};
    if (!catalog || !catalog->replaceable ||
        !Solver::HasResourceSplitPressure(
            stats, RESOURCE_WAIT_PRESSURE_US, TARGET_UTILIZATION_PPM)) {
        return candidates;
    }
    for (const auto &resource : catalog->resourceAlternates) {
        if (resource != task.scheduling.resource) {
            candidates.push_back(resource);
        }
    }
    return candidates;
}

std::vector<QueueCandidate> BuildQueueCandidates(
    const Epg::QueueConfig &queue,
    const Epg::QueueProfileMetrics &stats)
{
    const auto minDepth = std::max<std::uint64_t>(1, queue.depth);
    const auto maxDepth = std::max<std::uint64_t>(minDepth, MAX_QUEUE_DEPTH);
    std::vector<QueueCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(maxDepth - minDepth + 1));
    for (std::uint64_t depth = minDepth; depth <= maxDepth; ++depth) {
        candidates.push_back({
            depth,
            Solver::QueuePressureAtDepth(depth, stats),
            Solver::QueueCandidatePenalty(depth, stats),
        });
    }
    return candidates;
}

std::vector<TaskCandidate> BuildTaskCandidates(
    const Epg::GraphConfig &config,
    const Epg::GraphProfileDiagnostics &diagnostics,
    const Epg::TaskConfig &task,
    const Epg::TaskProfileMetrics &stats,
    const EpgTaskCatalogEntry *catalog,
    std::uint64_t effectiveLoopUs)
{
    const auto intervalBeforeMs =
        static_cast<std::uint64_t>(task.trigger.interval.count());
    const bool replaceable = catalog && catalog->replaceable;
    const auto backpressureBefore =
        Solver::SortedUniquePorts(task.scheduling.backpressureOutputs);
    const auto backpressureOptimized =
        Solver::CandidateBackpressurePorts(config, &diagnostics, task,
                                           backpressureBefore,
                                           replaceable);
    const auto resourceCandidates =
        TaskResourceCandidates(task, stats, catalog);
    const auto appendCandidate =
        [&](std::vector<TaskCandidate> &candidates,
            std::uint64_t intervalMs,
            const std::string &resource,
            const std::vector<Epg::PortId> &backpressureOutputs) {
        const auto topologyPenalty =
            Solver::BackpressureTopologyPenalty(
                &config, &diagnostics, task, backpressureBefore,
                backpressureOutputs, stats.totalResourceWaitUs,
                replaceable) +
            Solver::ResourceTopologyPenalty(task.scheduling.resource,
                                            resource);
        const auto predictedResourceWaitUs =
            Solver::PredictedResourceWaitUs(stats.totalResourceWaitUs,
                                            task.scheduling.resource,
                                            resource);
        candidates.push_back({
            intervalMs,
            resource,
            backpressureOutputs,
            predictedResourceWaitUs,
            topologyPenalty,
            Solver::TaskCandidatePenalty(
                intervalBeforeMs, intervalMs, stats, effectiveLoopUs,
                predictedResourceWaitUs, topologyPenalty,
                TARGET_UTILIZATION_PPM),
        });
    };

    std::vector<TaskCandidate> candidates;
    if (!replaceable || intervalBeforeMs == 0) {
        for (const auto &resource : resourceCandidates) {
            appendCandidate(candidates, intervalBeforeMs, resource,
                            backpressureBefore);
            if (backpressureOptimized != backpressureBefore) {
                appendCandidate(candidates, intervalBeforeMs, resource,
                                backpressureOptimized);
            }
        }
        return candidates;
    }
    const auto maxInterval =
        Solver::TaskFeasibleIntervalLimit(
            intervalBeforeMs, stats, effectiveLoopUs, TARGET_UTILIZATION_PPM,
            MAX_PERIODIC_INTERVAL_MS);
    candidates.reserve(static_cast<std::size_t>(
                           (maxInterval - intervalBeforeMs + 1) *
                           resourceCandidates.size() * 2));
    for (std::uint64_t interval = intervalBeforeMs; interval <= maxInterval;
         ++interval) {
        for (const auto &resource : resourceCandidates) {
            appendCandidate(candidates, interval, resource,
                            backpressureBefore);
            if (backpressureOptimized != backpressureBefore) {
                appendCandidate(candidates, interval, resource,
                                backpressureOptimized);
            }
        }
    }
    return candidates;
}

std::size_t BestQueueCandidateIndex(const QueueSolverNode &node)
{
    std::size_t bestIndex = 0;
    for (std::size_t index = 1; index < node.candidates.size(); ++index) {
        if (node.candidates[index].penalty <
            node.candidates[bestIndex].penalty) {
            bestIndex = index;
        }
    }
    return bestIndex;
}

std::size_t BestTaskCandidateIndex(const TaskSolverNode &node)
{
    std::size_t bestIndex = 0;
    for (std::size_t index = 1; index < node.candidates.size(); ++index) {
        const auto &candidate = node.candidates[index];
        const auto &best = node.candidates[bestIndex];
        if (candidate.penalty < best.penalty ||
            (candidate.penalty == best.penalty &&
             candidate.predictedResourceWaitUs < best.predictedResourceWaitUs) ||
            (candidate.penalty == best.penalty &&
             candidate.predictedResourceWaitUs == best.predictedResourceWaitUs &&
             candidate.backpressureOutputs.size() >
                 best.backpressureOutputs.size())) {
            bestIndex = index;
        }
    }
    return bestIndex;
}

TopologySolution SolveIndependentTopology(
    const std::vector<QueueSolverNode> &queues,
    const std::vector<TaskSolverNode> &tasks)
{
    TopologySolution solution;
    solution.queueCandidateIndexes.reserve(queues.size());
    solution.taskCandidateIndexes.reserve(tasks.size());
    for (const auto &queue : queues) {
        solution.queueCandidateIndexes.push_back(BestQueueCandidateIndex(queue));
    }
    for (const auto &task : tasks) {
        solution.taskCandidateIndexes.push_back(BestTaskCandidateIndex(task));
    }
    return solution;
}

std::vector<Solver::DiscreteCandidateSet> BuildGlobalCandidateSets(
    const std::vector<QueueSolverNode> &queues,
    const std::vector<TaskSolverNode> &tasks)
{
    std::vector<Solver::DiscreteCandidateSet> sets;
    sets.reserve(queues.size() + tasks.size());
    for (std::size_t nodeIndex = 0; nodeIndex < queues.size();
         ++nodeIndex) {
        Solver::DiscreteCandidateSet set;
        for (const auto &candidate : queues[nodeIndex].candidates) {
            set.penalties.push_back(candidate.penalty);
            set.tieWeights.push_back(0);
        }
        sets.push_back(std::move(set));
    }
    for (std::size_t nodeIndex = 0; nodeIndex < tasks.size(); ++nodeIndex) {
        Solver::DiscreteCandidateSet set;
        for (const auto &candidate : tasks[nodeIndex].candidates) {
            set.penalties.push_back(candidate.penalty);
            std::uint64_t tieWeight = candidate.backpressureOutputs.size();
            if (candidate.predictedResourceWaitUs == 0) {
                ++tieWeight;
            }
            set.tieWeights.push_back(tieWeight);
        }
        sets.push_back(std::move(set));
    }
    return sets;
}

TopologySolution BuildTopologySolutionFromGlobalIndexes(
    const std::vector<std::size_t> &candidateIndexes,
    std::size_t queueCount,
    std::size_t taskCount)
{
    TopologySolution solution;
    solution.queueCandidateIndexes.assign(queueCount, 0);
    solution.taskCandidateIndexes.assign(taskCount, 0);
    for (std::size_t index = 0; index < candidateIndexes.size(); ++index) {
        if (index < queueCount) {
            solution.queueCandidateIndexes[index] = candidateIndexes[index];
            continue;
        }
        solution.taskCandidateIndexes[index - queueCount] =
            candidateIndexes[index];
    }
    return solution;
}

TopologySolution SolveGlobalTopology(
    const std::vector<QueueSolverNode> &queues,
    const std::vector<TaskSolverNode> &tasks)
{
    const auto candidateSets = BuildGlobalCandidateSets(queues, tasks);
    const auto globalSolution = Solver::SolveDiscreteGlobalTopology(
        candidateSets, MAX_EXACT_TOPOLOGY_CANDIDATES);
    if (!globalSolution.exact) {
        return SolveIndependentTopology(queues, tasks);
    }
    return BuildTopologySolutionFromGlobalIndexes(
        globalSolution.candidateIndexes, queues.size(), tasks.size());
}

std::string TaskDecisionReason(const Epg::TaskConfig &task,
                               const Epg::TaskProfileMetrics &stats,
                               std::uint64_t effectiveLoopUs)
{
    std::vector<std::string> reasons;
    if (stats.utilizationPpm > TARGET_UTILIZATION_PPM) {
        reasons.push_back("utilization_over_target");
    }
    if (stats.budgetOverrunCount > 0 ||
        (task.scheduling.budgetUs > 0 &&
         effectiveLoopUs > task.scheduling.budgetUs)) {
        reasons.push_back("budget_overrun");
    }
    if (stats.deadlineMissCount > 0 ||
        (task.scheduling.deadlineUs > 0 &&
         effectiveLoopUs > task.scheduling.deadlineUs)) {
        reasons.push_back("deadline_miss");
    }
    if (stats.schedulingErrorCount > 0) {
        reasons.push_back("scheduling_error");
    }
    if (Solver::HasResourceWaitPressure(stats, RESOURCE_WAIT_PRESSURE_US)) {
        reasons.push_back("resource_wait");
    }
    if (reasons.empty()) {
        return "keep";
    }

    std::string reason = reasons.front();
    for (std::size_t index = 1; index < reasons.size(); ++index) {
        reason += "+" + reasons[index];
    }
    return reason;
}

std::string NotReplaceableTaskReason(const Epg::TaskConfig &task,
                                     const Epg::TaskProfileMetrics &stats,
                                     std::uint64_t effectiveLoopUs)
{
    const std::string reason = TaskDecisionReason(task, stats, effectiveLoopUs);
    if (reason == "keep") {
        return "not_replaceable";
    }
    return "not_replaceable+" + reason;
}

std::string JoinReasons(const std::vector<std::string> &reasons)
{
    if (reasons.empty()) {
        return {};
    }
    std::string reason = reasons.front();
    for (std::size_t index = 1; index < reasons.size(); ++index) {
        reason += "+" + reasons[index];
    }
    return reason;
}

std::string OptimizedTaskReason(const Epg::TaskConfig &task,
                                const Epg::TaskProfileMetrics &stats,
                                std::uint64_t effectiveLoopUs,
                                std::uint64_t intervalBeforeMs,
                                std::uint64_t intervalAfterMs,
                                const std::string &resourceBefore,
                                const std::string &resourceAfter,
                                const std::vector<Epg::PortId> &backpressureBefore,
                                const std::vector<Epg::PortId> &backpressureAfter,
                                const EpgTaskCatalogEntry *catalog)
{
    std::vector<std::string> reasons;
    if (intervalAfterMs != intervalBeforeMs) {
        reasons.push_back("global_optimum_interval");
    }
    if (backpressureAfter != backpressureBefore) {
        reasons.push_back("global_optimum_backpressure");
    }
    if (resourceAfter != resourceBefore) {
        reasons.push_back("global_optimum_resource");
    }
    if (!reasons.empty()) {
        return JoinReasons(reasons);
    }
    if (catalog && !catalog->replaceable) {
        return NotReplaceableTaskReason(task, stats, effectiveLoopUs);
    }
    return TaskDecisionReason(task, stats, effectiveLoopUs);
}

void ApplyResourceIsolation(Epg::TaskConfig &task,
                            const Epg::TaskProfileMetrics &stats,
                            const EpgTaskCatalogEntry *catalog)
{
    if (!catalog || !catalog->replaceable ||
        !Solver::HasResourceWaitPressure(stats, RESOURCE_WAIT_PRESSURE_US)) {
        return;
    }
    if (task.scheduling.cpuAffinity < 0) {
        task.scheduling.cpuAffinity = RESOURCE_ISOLATION_CPU_AFFINITY;
    }
    if (!task.scheduling.realtime && task.trigger.interval.count() > 0) {
        task.scheduling.realtime = true;
        task.scheduling.priority = RESOURCE_ISOLATION_PRIORITY;
    }
}

std::map<std::string, std::uint64_t>
MakeOptimizerNumbers(const Epg::GraphProfileMetadata &metadata,
                     std::uint64_t nowMs)
{
    return {
        {"generatedAtMs", nowMs},
        {"sourceTimestampMs", metadata.timestampMs},
    };
}

std::map<std::string, std::string>
MakeOptimizerStrings(const EpgTaskManifest &manifest)
{
    return {
        {"schema", Epg::OPTIMIZED_GRAPH_SCHEMA},
        {"sourceProfile", manifest.subgraphName},
        {"targetGraph", manifest.subgraphName},
        {"topologyVersion", manifest.topologyVersion},
        {"solverVersion", Epg::NATIVE_EXACT_SOLVER_VERSION},
    };
}

QueueSolverNode BuildQueueSolverNode(
    std::size_t index,
    const Epg::QueueConfig &queue,
    const Epg::QueueProfileMetrics &stats)
{
    QueueSolverNode node;
    node.index = index;
    node.depthBefore = queue.depth;
    node.pressureBefore = Solver::QueuePressure(queue, stats);
    node.stats = stats;
    node.candidates = BuildQueueCandidates(queue, stats);
    return node;
}

TaskSolverNode BuildTaskSolverNode(std::size_t index,
                                   const Epg::GraphConfig &config,
                                   const Epg::GraphProfileDiagnostics &diagnostics,
                                   const Epg::TaskConfig &task,
                                   const Epg::TaskProfileMetrics &stats,
                                   const EpgTaskCatalogEntry *catalog)
{
    TaskSolverNode node;
    node.index = index;
    node.intervalBeforeMs =
        static_cast<std::uint64_t>(task.trigger.interval.count());
    node.effectiveLoopUs = Solver::EffectiveLoopUs(stats);
    node.catalog = catalog;
    node.stats = stats;
    node.candidates =
        BuildTaskCandidates(config, diagnostics, task, stats, catalog,
                            node.effectiveLoopUs);
    return node;
}

void ApplyQueueSolution(Epg::QueueConfig &queue,
                        const QueueSolverNode &node,
                        const QueueCandidate &candidate,
                        std::vector<SolverDecision> &decisions)
{
    queue.depth = static_cast<std::size_t>(candidate.depth);
    SolverDecision decision;
    decision.kind = "queue";
    decision.name = queue.name;
    decision.depthBefore = node.depthBefore;
    decision.depthAfter = candidate.depth;
    decision.pressureBefore = node.pressureBefore;
    decision.pressureAfter = candidate.pressureAfter;
    decision.maxDepthObserved = node.stats.maxDepthObserved;
    decision.droppedNewest = node.stats.droppedNewest;
    decision.overwrittenOldest = node.stats.overwrittenOldest;
    decision.pushedPerSecond = node.stats.pushedPerSecond;
    decision.poppedPerSecond = node.stats.poppedPerSecond;
    decision.droppedPerSecond = node.stats.droppedPerSecond;
    decision.reason =
        candidate.depth != node.depthBefore ? "global_optimum_depth" : "keep";
    decisions.push_back(std::move(decision));
}

void ApplyTaskSolution(Epg::TaskConfig &task,
                       const TaskSolverNode &node,
                       const TaskCandidate &candidate,
                       std::vector<SolverDecision> &decisions)
{
    const auto backpressureBefore =
        Solver::SortedUniquePorts(task.scheduling.backpressureOutputs);
    const auto resourceBefore = task.scheduling.resource;
    if (candidate.intervalMs != node.intervalBeforeMs) {
        task.trigger.interval =
            std::chrono::milliseconds(static_cast<int>(candidate.intervalMs));
    }
    task.scheduling.resource = candidate.resource;
    task.scheduling.backpressureOutputs =
        Solver::SortedUniquePorts(candidate.backpressureOutputs);
    ApplyResourceIsolation(task, node.stats, node.catalog);
    const auto backpressureAfter = task.scheduling.backpressureOutputs;
    const auto resourceAfter = task.scheduling.resource;

    SolverDecision decision;
    decision.kind = "task";
    decision.name = task.name;
    decision.reason = OptimizedTaskReason(
        task, node.stats, node.effectiveLoopUs, node.intervalBeforeMs,
        candidate.intervalMs, resourceBefore, resourceAfter,
        backpressureBefore, backpressureAfter, node.catalog);
    decision.catalogRole = node.catalog ? node.catalog->role : "";
    decision.replaceable = node.catalog ? node.catalog->replaceable : false;
    decision.intervalBeforeMs = node.intervalBeforeMs;
    decision.intervalAfterMs = candidate.intervalMs;
    decision.maxLoopUs = node.stats.maxLoopUs;
    decision.averageLoopUs = node.stats.averageLoopUs;
    decision.p90LoopUs = node.stats.p90LoopUs;
    decision.p99LoopUs = node.stats.p99LoopUs;
    decision.effectiveLoopUs = node.effectiveLoopUs;
    decision.resourceWaitCount = node.stats.resourceWaitCount;
    decision.maxResourceWaitUs = node.stats.maxResourceWaitUs;
    decision.averageResourceWaitUs = node.stats.averageResourceWaitUs;
    decision.totalResourceWaitUs = node.stats.totalResourceWaitUs;
    decision.predictedResourceWaitUs = candidate.predictedResourceWaitUs;
    decision.utilizationPpm = node.stats.utilizationPpm;
    decision.budgetUs = task.scheduling.budgetUs;
    decision.deadlineUs = task.scheduling.deadlineUs;
    decision.budgetOverrunCount = node.stats.budgetOverrunCount;
    decision.deadlineMissCount = node.stats.deadlineMissCount;
    decision.schedulingErrorCount = node.stats.schedulingErrorCount;
    decision.resourceBefore = resourceBefore;
    decision.resourceAfter = resourceAfter;
    decision.backpressureBefore = backpressureBefore;
    decision.backpressureAfter = backpressureAfter;
    decision.topologyPenalty = candidate.topologyPenalty;
    decisions.push_back(std::move(decision));
}

std::vector<QueueSolverNode> BuildQueueSolverNodes(
    const Epg::GraphConfig &config,
    const Epg::GraphProfileDiagnostics &diagnostics)
{
    std::vector<QueueSolverNode> nodes;
    nodes.reserve(config.queues.size());
    for (std::size_t index = 0; index < config.queues.size(); ++index) {
        const auto &queue = config.queues[index];
        nodes.push_back(
            BuildQueueSolverNode(index, queue,
                                 diagnostics.queues.at(queue.name)));
    }
    return nodes;
}

std::vector<TaskSolverNode> BuildTaskSolverNodes(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig &config,
    const Epg::GraphProfileDiagnostics &diagnostics)
{
    std::vector<TaskSolverNode> nodes;
    nodes.reserve(config.tasks.size());
    for (std::size_t index = 0; index < config.tasks.size(); ++index) {
        const auto &task = config.tasks[index];
        nodes.push_back(BuildTaskSolverNode(
            index, config, diagnostics, task, diagnostics.tasks.at(task.name),
            FindCatalogEntry(manifest, task.type)));
    }
    return nodes;
}

Epg::GraphConfig OptimizeGraphConfig(const EpgTaskManifest &manifest,
                                     const Epg::GraphConfig &profileTopology,
                                     const Epg::GraphProfileDiagnostics &diagnostics,
                                     std::vector<SolverDecision> &decisions)
{
    auto config = profileTopology;
    ApplyEpgTaskCatalogDefaults(manifest, config);
    const auto queueNodes = BuildQueueSolverNodes(config, diagnostics);
    const auto taskNodes =
        BuildTaskSolverNodes(manifest, config, diagnostics);
    const auto solution = SolveGlobalTopology(queueNodes, taskNodes);

    for (std::size_t index = 0; index < queueNodes.size(); ++index) {
        const auto &node = queueNodes[index];
        const auto &candidate =
            node.candidates[solution.queueCandidateIndexes[index]];
        ApplyQueueSolution(config.queues[node.index], node, candidate,
                           decisions);
    }
    for (std::size_t index = 0; index < taskNodes.size(); ++index) {
        const auto &node = taskNodes[index];
        const auto &candidate =
            node.candidates[solution.taskCandidateIndexes[index]];
        ApplyTaskSolution(config.tasks[node.index], node, candidate,
                          decisions);
    }
    return config;
}

SolverScore ScoreDecisions(const std::vector<SolverDecision> &decisions)
{
    SolverScore score;
    for (const auto &decision : decisions) {
        if (decision.kind == "queue") {
            score.queuePressure += decision.pressureAfter;
            continue;
        }
        if (decision.kind != "task") {
            continue;
        }
        score.periodicOverloadUs += Solver::TaskPeriodicOverloadUs(
            decision.intervalAfterMs, decision.effectiveLoopUs);
        score.resourceWaitUs += decision.predictedResourceWaitUs;
        score.schedulingErrors += decision.schedulingErrorCount;
        score.budgetOverruns += decision.budgetOverrunCount;
        score.deadlineMisses += decision.deadlineMissCount;
        score.utilizationOverPpm += Solver::TaskUtilizationOverPpm(
            decision.intervalBeforeMs, decision.intervalAfterMs,
            decision.utilizationPpm, TARGET_UTILIZATION_PPM);
        score.topologyPenalty += decision.topologyPenalty;
    }
    return score;
}

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

std::uint64_t TotalPenalty(const SolverScore &score)
{
    return score.queuePressure * 1000 + score.periodicOverloadUs +
           score.resourceWaitUs + score.schedulingErrors * 10000 +
           score.budgetOverruns * 2000 + score.deadlineMisses * 5000 +
           score.utilizationOverPpm + score.topologyPenalty;
}

void WriteQueueDecisionJson(std::ostringstream &out,
                            const SolverDecision &decision)
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

void WriteTaskDecisionJson(std::ostringstream &out,
                           const SolverDecision &decision)
{
    out << "\"intervalBeforeMs\": " << decision.intervalBeforeMs << ", ";
    out << "\"intervalAfterMs\": " << decision.intervalAfterMs << ", ";
    out << "\"maxLoopUs\": " << decision.maxLoopUs << ", ";
    out << "\"averageLoopUs\": " << decision.averageLoopUs << ", ";
    out << "\"p90LoopUs\": " << decision.p90LoopUs << ", ";
    out << "\"p99LoopUs\": " << decision.p99LoopUs << ", ";
    out << "\"effectiveLoopUs\": " << decision.effectiveLoopUs << ", ";
    out << "\"resourceWaitCount\": " << decision.resourceWaitCount << ", ";
    out << "\"maxResourceWaitUs\": " << decision.maxResourceWaitUs << ", ";
    out << "\"averageResourceWaitUs\": "
        << decision.averageResourceWaitUs << ", ";
    out << "\"totalResourceWaitUs\": " << decision.totalResourceWaitUs << ", ";
    out << "\"predictedResourceWaitUs\": "
        << decision.predictedResourceWaitUs << ", ";
    out << "\"utilizationPpm\": " << decision.utilizationPpm << ", ";
    out << "\"targetUtilizationPpm\": " << TARGET_UTILIZATION_PPM << ", ";
    out << "\"budgetUs\": " << decision.budgetUs << ", ";
    out << "\"deadlineUs\": " << decision.deadlineUs << ", ";
    out << "\"catalogRole\": \"" << Solver::JsonEscape(decision.catalogRole)
        << "\", ";
    out << "\"replaceable\": " << (decision.replaceable ? "true" : "false")
        << ", ";
    out << "\"resourceBefore\": \""
        << Solver::JsonEscape(decision.resourceBefore) << "\", ";
    out << "\"resourceAfter\": \""
        << Solver::JsonEscape(decision.resourceAfter) << "\", ";
    out << "\"budgetOverrunCount\": " << decision.budgetOverrunCount << ", ";
    out << "\"deadlineMissCount\": " << decision.deadlineMissCount << ", ";
    out << "\"schedulingErrorCount\": " << decision.schedulingErrorCount
        << ", ";
    out << "\"topologyPenalty\": " << decision.topologyPenalty << ", ";
    out << "\"backpressureBefore\": ";
    WritePortArray(out, decision.backpressureBefore);
    out << ", ";
    out << "\"backpressureAfter\": ";
    WritePortArray(out, decision.backpressureAfter);
    out << ", ";
}

void WriteDecisionJson(std::ostringstream &out, const SolverDecision &decision)
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

std::string BuildSolverReport(const EpgTaskManifest &manifest,
                              const Epg::GraphProfileMetadata &metadata,
                              std::uint64_t nowMs,
                              const std::vector<SolverDecision> &decisions)
{
    const auto score = ScoreDecisions(decisions);
    std::ostringstream out;
    out << "{\n";
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
    out << "  \"objective\": {\n";
    out << "    \"name\": \"" << EPG_EXACT_SOLVER_OBJECTIVE << "\",\n";
    out << "    \"score\": {";
    out << "\"queuePressure\": " << score.queuePressure << ", ";
    out << "\"periodicOverloadUs\": " << score.periodicOverloadUs << ", ";
    out << "\"resourceWaitUs\": " << score.resourceWaitUs << ", ";
    out << "\"schedulingErrors\": " << score.schedulingErrors << ", ";
    out << "\"budgetOverruns\": " << score.budgetOverruns << ", ";
    out << "\"deadlineMisses\": " << score.deadlineMisses << ", ";
    out << "\"utilizationOverPpm\": " << score.utilizationOverPpm << ", ";
    out << "\"topologyPenalty\": " << score.topologyPenalty << ", ";
    out << "\"totalPenalty\": " << TotalPenalty(score) << "}\n";
    out << "  },\n";
    out << "  \"constraints\": {";
    out << "\"maxQueueDepth\": " << MAX_QUEUE_DEPTH << ", ";
    out << "\"maxPeriodicIntervalMs\": " << MAX_PERIODIC_INTERVAL_MS << ", ";
    out << "\"targetUtilizationPpm\": " << TARGET_UTILIZATION_PPM << "},\n";
    out << "  \"decisions\": [\n";
    for (std::size_t index = 0; index < decisions.size(); ++index) {
        if (index != 0) {
            out << ",\n";
        }
        WriteDecisionJson(out, decisions[index]);
    }
    out << "\n  ]\n";
    out << "}\n";
    return out.str();
}

Epg::OptimizedGraph ValidateGeneratedArtifacts(
    const EpgTaskManifest &manifest,
    const Epg::GraphProfile &sourceProfile,
    const std::string &optimizedJson,
    const std::string &reportJson)
{
    auto optimized = Epg::ParseOptimizedGraphJson(optimizedJson);
    ValidateEpgOptimizedGraphManifest(manifest, optimized);
    const auto report = Epg::ParseSolverReportJson(reportJson);
    ValidateEpgSolverReport(manifest, sourceProfile, optimized, report);
    return optimized;
}

EpgRuntimeOptimizerResult WriteOptimizedConfig(const EpgTaskManifest &manifest,
                                               const Epg::GraphProfile &profile,
                                               std::uint64_t nowMs)
{
    const auto &paths = manifest.artifactPaths;
    std::vector<SolverDecision> decisions;
    const auto config =
        OptimizeGraphConfig(manifest, profile.topology, profile.diagnostics,
                            decisions);
    const std::string json =
        Epg::GraphConfigToJson(config, MakeOptimizerStrings(manifest),
                               MakeOptimizerNumbers(profile.metadata, nowMs));
    const std::string report =
        BuildSolverReport(manifest, profile.metadata, nowMs, decisions);
    (void)ValidateGeneratedArtifacts(manifest, profile, json, report);
    const bool changed =
        EpgOptimizedConfigChanged(
            ReadEpgOptimizerFile(paths.optimizedConfigPath), json);
    WriteRequiredEpgOptimizerArtifactFile(paths.solverReportPath, report);
    WriteRequiredEpgOptimizerArtifactFile(paths.optimizedConfigPath, json);
    EpgRuntimeOptimizerResult result;
    result.optimized = true;
    result.configChanged = changed;
    result.message = changed ? "optimized config changed"
                             : "optimized config refreshed";
    result.targetGraph = manifest.subgraphName;
    result.topologyVersion = manifest.topologyVersion;
    result.sourceProfile = manifest.subgraphName;
    result.sourceProfilePath = paths.profilePath;
    result.sourceTimestampMs = profile.metadata.timestampMs;
    result.generatedAtMs = nowMs;
    result.solverVersion = Epg::NATIVE_EXACT_SOLVER_VERSION;
    result.optimizedConfigPath = paths.optimizedConfigPath;
    result.solverReportPath = paths.solverReportPath;
    return result;
}

} // namespace

EpgRuntimeOptimizerResult
OptimizeEpgProfileForManifest(const EpgTaskManifest &manifest,
                              std::uint64_t nowMs)
{
    try {
        ValidateEpgTaskManifest(manifest);
    } catch (const std::exception &error) {
        return {false, false, error.what()};
    }
    const std::string profileText =
        ReadEpgOptimizerFile(manifest.artifactPaths.profilePath);
    if (profileText.empty()) {
        return {false, false, "profile missing"};
    }
    Epg::GraphProfile profile;
    try {
        profile = Epg::ParseGraphProfileJson(profileText);
    } catch (const std::exception &error) {
        return {false, false, error.what()};
    }
    const auto &metadata = profile.metadata;
    if (metadata.schema != Epg::GRAPH_PROFILE_SCHEMA) {
        return {false, false, "profile schema mismatch"};
    }
    if (metadata.graph != manifest.subgraphName) {
        return {false, false, "profile graph mismatch"};
    }
    if (metadata.topologyVersion != manifest.topologyVersion) {
        return {false, false, "profile topology version mismatch"};
    }
    if (ProfileAgeMs(nowMs, metadata.timestampMs) > PROFILE_FRESHNESS_MS) {
        return {false, false, "profile stale"};
    }
    try {
        ValidateProfileCatalog(manifest, profile);
        ApplyEpgTaskCatalogDefaults(manifest, profile.topology);
        ValidateEpgTaskGraphManifest(manifest, profile.topology);
        ValidateProfileDiagnosticsCoverage(profile.topology,
                                           profile.diagnostics);
        return WriteOptimizedConfig(manifest, profile, nowMs);
    } catch (const std::exception &error) {
        return {false, false, error.what()};
    }
}

} // namespace SmartDrone::Core::Application
