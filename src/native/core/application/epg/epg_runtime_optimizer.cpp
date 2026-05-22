#include "core/application/epg/epg_runtime_optimizer.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "core/application/epg/epg_task_manifest.h"
#include "core/application/runtime/epg_dfx_snapshot.h"

namespace SmartDrone::Core::Application {
namespace {

constexpr std::uint64_t PROFILE_FRESHNESS_MS = 60000;
constexpr std::size_t MAX_QUEUE_DEPTH = 16;
constexpr std::uint64_t MAX_PERIODIC_INTERVAL_MS = 1000;
constexpr std::uint64_t TARGET_UTILIZATION_PPM = 800000;
constexpr std::uint64_t RESOURCE_WAIT_PRESSURE_US = 1000;
constexpr int RESOURCE_ISOLATION_CPU_AFFINITY = 2;
constexpr int RESOURCE_ISOLATION_PRIORITY = 20;

struct SolverDecision {
    std::string kind;
    std::string name;
    std::string reason;
    std::string catalogRole;
    bool replaceable{false};
    std::uint64_t depthBefore{0};
    std::uint64_t depthAfter{0};
    std::uint64_t pressureBefore{0};
    std::uint64_t pressureAfter{0};
    std::uint64_t maxDepthObserved{0};
    std::uint64_t droppedNewest{0};
    std::uint64_t overwrittenOldest{0};
    std::uint64_t pushedPerSecond{0};
    std::uint64_t poppedPerSecond{0};
    std::uint64_t droppedPerSecond{0};
    std::uint64_t intervalBeforeMs{0};
    std::uint64_t intervalAfterMs{0};
    std::uint64_t maxLoopUs{0};
    std::uint64_t averageLoopUs{0};
    std::uint64_t p90LoopUs{0};
    std::uint64_t p99LoopUs{0};
    std::uint64_t effectiveLoopUs{0};
    std::uint64_t resourceWaitCount{0};
    std::uint64_t maxResourceWaitUs{0};
    std::uint64_t averageResourceWaitUs{0};
    std::uint64_t totalResourceWaitUs{0};
    std::uint64_t utilizationPpm{0};
    std::uint64_t budgetUs{0};
    std::uint64_t deadlineUs{0};
    std::uint64_t budgetOverrunCount{0};
    std::uint64_t deadlineMissCount{0};
    std::uint64_t schedulingErrorCount{0};
};

struct SolverScore {
    std::uint64_t queuePressure{0};
    std::uint64_t periodicOverloadUs{0};
    std::uint64_t resourceWaitUs{0};
    std::uint64_t schedulingErrors{0};
    std::uint64_t budgetOverruns{0};
    std::uint64_t deadlineMisses{0};
    std::uint64_t utilizationOverPpm{0};
};

struct QueueCandidate {
    std::uint64_t depth{0};
    std::uint64_t pressureAfter{0};
    std::uint64_t penalty{0};
};

struct TaskCandidate {
    std::uint64_t intervalMs{0};
    std::uint64_t penalty{0};
};

struct QueueSolverNode {
    std::size_t index{0};
    std::uint64_t depthBefore{0};
    std::uint64_t pressureBefore{0};
    Epg::QueueProfileMetrics stats;
    std::vector<QueueCandidate> candidates;
};

struct TaskSolverNode {
    std::size_t index{0};
    std::uint64_t intervalBeforeMs{0};
    std::uint64_t effectiveLoopUs{0};
    const EpgTaskCatalogEntry *catalog{nullptr};
    Epg::TaskProfileMetrics stats;
    std::vector<TaskCandidate> candidates;
};

struct TopologySolution {
    std::vector<std::size_t> queueCandidateIndexes;
    std::vector<std::size_t> taskCandidateIndexes;
    std::uint64_t totalPenalty{0};
};

std::string ReadFile(const std::string &path)
{
    std::ifstream input(path);
    if (!input) {
        return {};
    }
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
}

std::string StripGeneratedMetadata(std::string text)
{
    const std::vector<std::string> fields = {
        "generatedAtMs",
        "sourceTimestampMs",
    };
    for (const auto &field : fields) {
        const auto key = "\"" + field + "\":";
        auto pos = text.find(key);
        if (pos == std::string::npos) {
            continue;
        }
        auto end = text.find('\n', pos);
        if (end == std::string::npos) {
            text.erase(pos);
            continue;
        }
        text.erase(pos, end - pos + 1);
    }
    return text;
}

bool OptimizedConfigChanged(const std::string &oldJson,
                            const std::string &newJson)
{
    return StripGeneratedMetadata(oldJson) != StripGeneratedMetadata(newJson);
}

void EnsureArtifactDirectory(const std::string &path)
{
    const std::filesystem::path artifactPath(path);
    const auto parent = artifactPath.parent_path();
    if (parent.empty()) {
        return;
    }
    std::error_code error;
    if (std::filesystem::exists(parent, error) &&
        !std::filesystem::is_directory(parent, error)) {
        throw std::runtime_error("EPG artifact directory is not a directory: " +
                                 parent.string());
    }
    std::filesystem::create_directories(parent, error);
    if (error) {
        throw std::runtime_error("failed to create EPG artifact directory: " +
                                 parent.string());
    }
}

void WriteRequiredArtifactFile(const std::string &path,
                               const std::string &text)
{
    EnsureArtifactDirectory(path);
    const std::string tempPath = path + ".tmp";
    {
        std::ofstream output(tempPath, std::ios::out | std::ios::trunc);
        if (!output) {
            throw std::runtime_error("failed to open EPG artifact: " +
                                     tempPath);
        }
        output << text;
        if (!output) {
            throw std::runtime_error("failed to write EPG artifact: " +
                                     tempPath);
        }
    }
    if (std::rename(tempPath.c_str(), path.c_str()) != 0) {
        (void)std::remove(tempPath.c_str());
        throw std::runtime_error("failed to publish EPG artifact: " + path);
    }
}

std::string JsonEscape(const std::string &value)
{
    std::string result;
    for (const char ch : value) {
        switch (ch) {
        case '\\':
            result += "\\\\";
            break;
        case '"':
            result += "\\\"";
            break;
        case '\n':
            result += "\\n";
            break;
        case '\r':
            result += "\\r";
            break;
        case '\t':
            result += "\\t";
            break;
        default:
            result.push_back(ch);
            break;
        }
    }
    return result;
}

std::uint64_t ProfileAgeMs(std::uint64_t nowMs,
                           std::uint64_t profileTimestampMs)
{
    if (profileTimestampMs == 0 || nowMs < profileTimestampMs) {
        return PROFILE_FRESHNESS_MS + 1;
    }
    return nowMs - profileTimestampMs;
}

std::uint64_t CeilDiv(std::uint64_t numerator, std::uint64_t denominator)
{
    if (denominator == 0) {
        return numerator;
    }
    return (numerator + denominator - 1) / denominator;
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
        entry.replaceable != manifestEntry.replaceable) {
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

std::uint64_t EffectiveLoopUs(const Epg::TaskProfileMetrics &stats)
{
    return std::max({stats.p99LoopUs, stats.p90LoopUs, stats.maxLoopUs,
                     stats.averageLoopUs});
}

bool HasResourceWaitPressure(const Epg::TaskProfileMetrics &stats)
{
    return stats.maxResourceWaitUs > RESOURCE_WAIT_PRESSURE_US ||
           stats.averageResourceWaitUs > RESOURCE_WAIT_PRESSURE_US ||
           stats.totalResourceWaitUs > RESOURCE_WAIT_PRESSURE_US;
}

std::uint64_t QueuePressure(const Epg::QueueConfig &queue,
                            const Epg::QueueProfileMetrics &stats)
{
    const auto depthPressure =
        stats.maxDepthObserved > queue.depth
            ? stats.maxDepthObserved - static_cast<std::uint64_t>(queue.depth)
            : 0;
    return depthPressure + stats.droppedNewest + stats.overwrittenOldest;
}

std::uint64_t QueuePressureAtDepth(std::uint64_t depth,
                                   const Epg::QueueProfileMetrics &stats)
{
    const auto depthPressure =
        stats.maxDepthObserved > depth ? stats.maxDepthObserved - depth : 0;
    return depthPressure + stats.droppedNewest + stats.overwrittenOldest;
}

std::uint64_t TaskPeriodicOverloadUs(std::uint64_t intervalMs,
                                     std::uint64_t effectiveLoopUs)
{
    const std::uint64_t intervalUs = intervalMs * 1000;
    if (intervalUs == 0 || effectiveLoopUs <= intervalUs) {
        return 0;
    }
    return effectiveLoopUs - intervalUs;
}

std::uint64_t TaskUtilizationOverPpm(std::uint64_t intervalBeforeMs,
                                     std::uint64_t intervalAfterMs,
                                     std::uint64_t utilizationPpm)
{
    if (intervalAfterMs == 0 || utilizationPpm <= TARGET_UTILIZATION_PPM) {
        return 0;
    }
    const auto scaledUtilization =
        CeilDiv(utilizationPpm * intervalBeforeMs, intervalAfterMs);
    if (scaledUtilization <= TARGET_UTILIZATION_PPM) {
        return 0;
    }
    return scaledUtilization - TARGET_UTILIZATION_PPM;
}

std::uint64_t TaskFeasibleIntervalLimit(std::uint64_t intervalBeforeMs,
                                        const Epg::TaskProfileMetrics &stats,
                                        std::uint64_t effectiveLoopUs)
{
    std::uint64_t limit = intervalBeforeMs;
    if (intervalBeforeMs > 0 && effectiveLoopUs > intervalBeforeMs * 1000) {
        limit = std::max(limit, CeilDiv(effectiveLoopUs, 1000));
    }
    if (intervalBeforeMs > 0 && stats.utilizationPpm > TARGET_UTILIZATION_PPM) {
        limit = std::max(
            limit,
            CeilDiv(intervalBeforeMs * stats.utilizationPpm,
                    TARGET_UTILIZATION_PPM));
    }
    return std::min(std::max(limit, intervalBeforeMs),
                    MAX_PERIODIC_INTERVAL_MS);
}

std::uint64_t QueueCandidatePenalty(std::uint64_t depth,
                                    const Epg::QueueProfileMetrics &stats)
{
    return QueuePressureAtDepth(depth, stats) * 1000 + depth;
}

std::uint64_t TaskCandidatePenalty(std::uint64_t intervalBeforeMs,
                                   std::uint64_t intervalAfterMs,
                                   const Epg::TaskProfileMetrics &stats,
                                   std::uint64_t effectiveLoopUs)
{
    return TaskPeriodicOverloadUs(intervalAfterMs, effectiveLoopUs) +
           stats.totalResourceWaitUs + stats.schedulingErrorCount * 10000 +
           stats.budgetOverrunCount * 2000 +
           stats.deadlineMissCount * 5000 +
           TaskUtilizationOverPpm(intervalBeforeMs, intervalAfterMs,
                                  stats.utilizationPpm) +
           intervalAfterMs;
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
            QueuePressureAtDepth(depth, stats),
            QueueCandidatePenalty(depth, stats),
        });
    }
    return candidates;
}

std::vector<TaskCandidate> BuildTaskCandidates(
    const Epg::TaskConfig &task,
    const Epg::TaskProfileMetrics &stats,
    const EpgTaskCatalogEntry *catalog,
    std::uint64_t effectiveLoopUs)
{
    const auto intervalBeforeMs =
        static_cast<std::uint64_t>(task.trigger.interval.count());
    if (!catalog || !catalog->replaceable || intervalBeforeMs == 0) {
        return {{
            intervalBeforeMs,
            TaskCandidatePenalty(intervalBeforeMs, intervalBeforeMs, stats,
                                 effectiveLoopUs),
        }};
    }
    const auto maxInterval =
        TaskFeasibleIntervalLimit(intervalBeforeMs, stats, effectiveLoopUs);
    std::vector<TaskCandidate> candidates;
    candidates.reserve(static_cast<std::size_t>(maxInterval -
                                                intervalBeforeMs + 1));
    for (std::uint64_t interval = intervalBeforeMs; interval <= maxInterval;
         ++interval) {
        candidates.push_back({
            interval,
            TaskCandidatePenalty(intervalBeforeMs, interval, stats,
                                 effectiveLoopUs),
        });
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
        if (node.candidates[index].penalty <
            node.candidates[bestIndex].penalty) {
            bestIndex = index;
        }
    }
    return bestIndex;
}

TopologySolution SolveGlobalTopology(
    const std::vector<QueueSolverNode> &queues,
    const std::vector<TaskSolverNode> &tasks)
{
    TopologySolution solution;
    solution.queueCandidateIndexes.reserve(queues.size());
    solution.taskCandidateIndexes.reserve(tasks.size());
    for (const auto &queue : queues) {
        const auto index = BestQueueCandidateIndex(queue);
        solution.queueCandidateIndexes.push_back(index);
        solution.totalPenalty += queue.candidates[index].penalty;
    }
    for (const auto &task : tasks) {
        const auto index = BestTaskCandidateIndex(task);
        solution.taskCandidateIndexes.push_back(index);
        solution.totalPenalty += task.candidates[index].penalty;
    }
    return solution;
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
    if (HasResourceWaitPressure(stats)) {
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

void ApplyResourceIsolation(Epg::TaskConfig &task,
                            const Epg::TaskProfileMetrics &stats,
                            const EpgTaskCatalogEntry *catalog)
{
    if (!catalog || !catalog->replaceable || !HasResourceWaitPressure(stats)) {
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
    node.pressureBefore = QueuePressure(queue, stats);
    node.stats = stats;
    node.candidates = BuildQueueCandidates(queue, stats);
    return node;
}

TaskSolverNode BuildTaskSolverNode(std::size_t index,
                                   const Epg::TaskConfig &task,
                                   const Epg::TaskProfileMetrics &stats,
                                   const EpgTaskCatalogEntry *catalog)
{
    TaskSolverNode node;
    node.index = index;
    node.intervalBeforeMs =
        static_cast<std::uint64_t>(task.trigger.interval.count());
    node.effectiveLoopUs = EffectiveLoopUs(stats);
    node.catalog = catalog;
    node.stats = stats;
    node.candidates =
        BuildTaskCandidates(task, stats, catalog, node.effectiveLoopUs);
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
    if (candidate.intervalMs != node.intervalBeforeMs) {
        task.trigger.interval =
            std::chrono::milliseconds(static_cast<int>(candidate.intervalMs));
    }
    ApplyResourceIsolation(task, node.stats, node.catalog);

    SolverDecision decision;
    decision.kind = "task";
    decision.name = task.name;
    decision.reason = candidate.intervalMs != node.intervalBeforeMs
                          ? "global_optimum_interval"
                      : node.catalog && !node.catalog->replaceable
                          ? NotReplaceableTaskReason(task, node.stats,
                                                     node.effectiveLoopUs)
                          : TaskDecisionReason(task, node.stats,
                                               node.effectiveLoopUs);
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
    decision.utilizationPpm = node.stats.utilizationPpm;
    decision.budgetUs = task.scheduling.budgetUs;
    decision.deadlineUs = task.scheduling.deadlineUs;
    decision.budgetOverrunCount = node.stats.budgetOverrunCount;
    decision.deadlineMissCount = node.stats.deadlineMissCount;
    decision.schedulingErrorCount = node.stats.schedulingErrorCount;
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
            index, task, diagnostics.tasks.at(task.name),
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
        score.periodicOverloadUs += TaskPeriodicOverloadUs(
            decision.intervalAfterMs, decision.effectiveLoopUs);
        score.resourceWaitUs += decision.totalResourceWaitUs;
        score.schedulingErrors += decision.schedulingErrorCount;
        score.budgetOverruns += decision.budgetOverrunCount;
        score.deadlineMisses += decision.deadlineMissCount;
        score.utilizationOverPpm += TaskUtilizationOverPpm(
            decision.intervalBeforeMs, decision.intervalAfterMs,
            decision.utilizationPpm);
    }
    return score;
}

std::uint64_t TotalPenalty(const SolverScore &score)
{
    return score.queuePressure * 1000 + score.periodicOverloadUs +
           score.resourceWaitUs + score.schedulingErrors * 10000 +
           score.budgetOverruns * 2000 + score.deadlineMisses * 5000 +
           score.utilizationOverPpm;
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
    out << "\"utilizationPpm\": " << decision.utilizationPpm << ", ";
    out << "\"targetUtilizationPpm\": " << TARGET_UTILIZATION_PPM << ", ";
    out << "\"budgetUs\": " << decision.budgetUs << ", ";
    out << "\"deadlineUs\": " << decision.deadlineUs << ", ";
    out << "\"catalogRole\": \"" << JsonEscape(decision.catalogRole) << "\", ";
    out << "\"replaceable\": " << (decision.replaceable ? "true" : "false")
        << ", ";
    out << "\"budgetOverrunCount\": " << decision.budgetOverrunCount << ", ";
    out << "\"deadlineMissCount\": " << decision.deadlineMissCount << ", ";
    out << "\"schedulingErrorCount\": " << decision.schedulingErrorCount
        << ", ";
}

void WriteDecisionJson(std::ostringstream &out, const SolverDecision &decision)
{
    out << "    {";
    out << "\"kind\": \"" << JsonEscape(decision.kind) << "\", ";
    out << "\"name\": \"" << JsonEscape(decision.name) << "\", ";
    if (decision.kind == "queue") {
        WriteQueueDecisionJson(out, decision);
    } else {
        WriteTaskDecisionJson(out, decision);
    }
    out << "\"reason\": \"" << JsonEscape(decision.reason) << "\"";
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
    out << "  \"targetGraph\": \"" << JsonEscape(manifest.subgraphName)
        << "\",\n";
    out << "  \"topologyVersion\": \""
        << JsonEscape(manifest.topologyVersion) << "\",\n";
    out << "  \"sourceProfile\": \"" << JsonEscape(manifest.subgraphName)
        << "\",\n";
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
        OptimizedConfigChanged(ReadFile(paths.optimizedConfigPath), json);
    WriteRequiredArtifactFile(paths.solverReportPath, report);
    WriteRequiredArtifactFile(paths.optimizedConfigPath, json);
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
        ReadFile(manifest.artifactPaths.profilePath);
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
