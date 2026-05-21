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

#include "common/epg/epg.h"
#include "core/application/runtime/epg_dfx_snapshot.h"

namespace smartdrone::core::application {
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

const epg::GraphProfileTaskCatalogEntry *FindProfileCatalogEntry(
    const epg::GraphProfile &profile,
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
                                 const epg::GraphProfileTaskCatalogEntry &entry)
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
                            const epg::GraphProfile &profile)
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
    const epg::GraphConfig &topology,
    const epg::GraphProfileDiagnostics &diagnostics)
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
    const epg::GraphConfig &topology,
    const epg::GraphProfileDiagnostics &diagnostics)
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
    const epg::GraphConfig &topology,
    const epg::GraphProfileDiagnostics &diagnostics)
{
    ValidateProfileQueueDiagnostics(topology, diagnostics);
    ValidateProfileTaskDiagnostics(topology, diagnostics);
}

std::uint64_t EffectiveLoopUs(const epg::TaskProfileMetrics &stats)
{
    return std::max({stats.p99LoopUs, stats.p90LoopUs, stats.maxLoopUs,
                     stats.averageLoopUs});
}

bool HasResourceWaitPressure(const epg::TaskProfileMetrics &stats)
{
    return stats.maxResourceWaitUs > RESOURCE_WAIT_PRESSURE_US ||
           stats.averageResourceWaitUs > RESOURCE_WAIT_PRESSURE_US ||
           stats.totalResourceWaitUs > RESOURCE_WAIT_PRESSURE_US;
}

std::uint64_t QueuePressure(const epg::QueueConfig &queue,
                            const epg::QueueProfileMetrics &stats)
{
    const auto depthPressure =
        stats.maxDepthObserved > queue.depth
            ? stats.maxDepthObserved - static_cast<std::uint64_t>(queue.depth)
            : 0;
    return depthPressure + stats.droppedNewest + stats.overwrittenOldest;
}

std::string TaskDecisionReason(const epg::TaskConfig &task,
                               const epg::TaskProfileMetrics &stats,
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

std::string NotReplaceableTaskReason(const epg::TaskConfig &task,
                                     const epg::TaskProfileMetrics &stats,
                                     std::uint64_t effectiveLoopUs)
{
    const std::string reason = TaskDecisionReason(task, stats, effectiveLoopUs);
    if (reason == "keep") {
        return "not_replaceable";
    }
    return "not_replaceable+" + reason;
}

std::uint64_t TargetIntervalMs(const epg::TaskConfig &task,
                               const epg::TaskProfileMetrics &stats,
                               std::uint64_t effectiveLoopUs)
{
    const auto intervalMs =
        static_cast<std::uint64_t>(task.trigger.interval.count());
    std::uint64_t target = intervalMs;
    if (intervalMs > 0 && effectiveLoopUs > intervalMs * 1000) {
        target = std::max(target, CeilDiv(effectiveLoopUs, 1000));
    }
    if (intervalMs > 0 && stats.utilizationPpm > TARGET_UTILIZATION_PPM) {
        target = std::max(target, CeilDiv(intervalMs * stats.utilizationPpm,
                                          TARGET_UTILIZATION_PPM));
    }
    target = std::max(target, intervalMs);
    return std::min(target, MAX_PERIODIC_INTERVAL_MS);
}

std::uint64_t OptimizedTaskIntervalMs(const epg::TaskConfig &task,
                                      const epg::TaskProfileMetrics &stats,
                                      const EpgTaskCatalogEntry *catalog,
                                      std::uint64_t effectiveLoopUs)
{
    const auto intervalMs =
        static_cast<std::uint64_t>(task.trigger.interval.count());
    if (!catalog || !catalog->replaceable) {
        return intervalMs;
    }
    return TargetIntervalMs(task, stats, effectiveLoopUs);
}

void ApplyResourceIsolation(epg::TaskConfig &task,
                            const epg::TaskProfileMetrics &stats,
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
MakeOptimizerNumbers(const epg::GraphProfileMetadata &metadata,
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
        {"schema", epg::OPTIMIZED_GRAPH_SCHEMA},
        {"sourceProfile", manifest.subgraphName},
        {"targetGraph", manifest.subgraphName},
        {"topologyVersion", manifest.topologyVersion},
        {"solverVersion", epg::NATIVE_HEURISTIC_SOLVER_VERSION},
    };
}

void OptimizeQueue(epg::QueueConfig &queue,
                   const epg::QueueProfileMetrics &stats,
                   std::vector<SolverDecision> &decisions)
{
    const auto depthBefore = queue.depth;
    const auto pressure = QueuePressure(queue, stats);
    if (pressure > 0) {
        const auto pressureDepth = std::max(
            {static_cast<std::size_t>(depthBefore + 1),
             static_cast<std::size_t>(stats.maxDepthObserved * 2), 2UL});
        queue.depth = std::min(pressureDepth, MAX_QUEUE_DEPTH);
    }

    SolverDecision decision;
    decision.kind = "queue";
    decision.name = queue.name;
    decision.depthBefore = depthBefore;
    decision.depthAfter = queue.depth;
    decision.pressureBefore = pressure;
    decision.pushedPerSecond = stats.pushedPerSecond;
    decision.poppedPerSecond = stats.poppedPerSecond;
    decision.droppedPerSecond = stats.droppedPerSecond;
    decision.reason = queue.depth != depthBefore ? "increase_depth" : "keep";
    decisions.push_back(std::move(decision));
}

void OptimizeTask(epg::TaskConfig &task,
                  const epg::TaskProfileMetrics &stats,
                  const EpgTaskCatalogEntry *catalog,
                  std::vector<SolverDecision> &decisions)
{
    const auto intervalBefore =
        static_cast<std::uint64_t>(task.trigger.interval.count());
    const auto effectiveLoopUs = EffectiveLoopUs(stats);
    const auto targetInterval =
        OptimizedTaskIntervalMs(task, stats, catalog, effectiveLoopUs);
    if (targetInterval != intervalBefore) {
        task.trigger.interval =
            std::chrono::milliseconds(static_cast<int>(targetInterval));
    }
    ApplyResourceIsolation(task, stats, catalog);

    SolverDecision decision;
    decision.kind = "task";
    decision.name = task.name;
    decision.reason = targetInterval != intervalBefore
                          ? "increase_interval"
                      : catalog && !catalog->replaceable
                          ? NotReplaceableTaskReason(task, stats,
                                                     effectiveLoopUs)
                          : TaskDecisionReason(task, stats, effectiveLoopUs);
    decision.catalogRole = catalog ? catalog->role : "";
    decision.replaceable = catalog ? catalog->replaceable : false;
    decision.intervalBeforeMs = intervalBefore;
    decision.intervalAfterMs = targetInterval;
    decision.maxLoopUs = stats.maxLoopUs;
    decision.averageLoopUs = stats.averageLoopUs;
    decision.p90LoopUs = stats.p90LoopUs;
    decision.p99LoopUs = stats.p99LoopUs;
    decision.effectiveLoopUs = effectiveLoopUs;
    decision.resourceWaitCount = stats.resourceWaitCount;
    decision.maxResourceWaitUs = stats.maxResourceWaitUs;
    decision.averageResourceWaitUs = stats.averageResourceWaitUs;
    decision.totalResourceWaitUs = stats.totalResourceWaitUs;
    decision.utilizationPpm = stats.utilizationPpm;
    decision.budgetUs = task.scheduling.budgetUs;
    decision.deadlineUs = task.scheduling.deadlineUs;
    decision.budgetOverrunCount = stats.budgetOverrunCount;
    decision.deadlineMissCount = stats.deadlineMissCount;
    decision.schedulingErrorCount = stats.schedulingErrorCount;
    decisions.push_back(std::move(decision));
}

epg::GraphConfig OptimizeGraphConfig(const EpgTaskManifest &manifest,
                                     const epg::GraphConfig &profileTopology,
                                     const epg::GraphProfileDiagnostics &diagnostics,
                                     std::vector<SolverDecision> &decisions)
{
    auto config = profileTopology;
    ApplyEpgTaskCatalogDefaults(manifest, config);

    for (auto &queue : config.queues) {
        OptimizeQueue(queue, diagnostics.queues.at(queue.name), decisions);
    }
    for (auto &task : config.tasks) {
        OptimizeTask(task, diagnostics.tasks.at(task.name),
                     FindCatalogEntry(manifest, task.type), decisions);
    }
    return config;
}

SolverScore ScoreDecisions(const std::vector<SolverDecision> &decisions)
{
    SolverScore score;
    for (const auto &decision : decisions) {
        if (decision.kind == "queue") {
            score.queuePressure += decision.pressureBefore;
            continue;
        }
        if (decision.kind != "task") {
            continue;
        }
        if (decision.effectiveLoopUs > decision.intervalBeforeMs * 1000) {
            score.periodicOverloadUs +=
                decision.effectiveLoopUs - decision.intervalBeforeMs * 1000;
        }
        score.resourceWaitUs += decision.totalResourceWaitUs;
        score.schedulingErrors += decision.schedulingErrorCount;
        score.budgetOverruns += decision.budgetOverrunCount;
        score.deadlineMisses += decision.deadlineMissCount;
        if (decision.utilizationPpm > TARGET_UTILIZATION_PPM) {
            score.utilizationOverPpm +=
                decision.utilizationPpm - TARGET_UTILIZATION_PPM;
        }
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

void WriteDecisionJson(std::ostringstream &out, const SolverDecision &decision)
{
    out << "    {";
    out << "\"kind\": \"" << JsonEscape(decision.kind) << "\", ";
    out << "\"name\": \"" << JsonEscape(decision.name) << "\", ";
    if (decision.kind == "queue") {
        out << "\"depthBefore\": " << decision.depthBefore << ", ";
        out << "\"depthAfter\": " << decision.depthAfter << ", ";
        out << "\"pressureBefore\": " << decision.pressureBefore << ", ";
        out << "\"pushedPerSecond\": " << decision.pushedPerSecond << ", ";
        out << "\"poppedPerSecond\": " << decision.poppedPerSecond << ", ";
        out << "\"droppedPerSecond\": " << decision.droppedPerSecond << ", ";
    } else {
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
        out << "\"totalResourceWaitUs\": " << decision.totalResourceWaitUs
            << ", ";
        out << "\"utilizationPpm\": " << decision.utilizationPpm << ", ";
        out << "\"targetUtilizationPpm\": " << TARGET_UTILIZATION_PPM << ", ";
        out << "\"budgetUs\": " << decision.budgetUs << ", ";
        out << "\"deadlineUs\": " << decision.deadlineUs << ", ";
        out << "\"catalogRole\": \"" << JsonEscape(decision.catalogRole)
            << "\", ";
        out << "\"replaceable\": " << (decision.replaceable ? "true" : "false")
            << ", ";
        out << "\"budgetOverrunCount\": " << decision.budgetOverrunCount
            << ", ";
        out << "\"deadlineMissCount\": " << decision.deadlineMissCount << ", ";
        out << "\"schedulingErrorCount\": " << decision.schedulingErrorCount
            << ", ";
    }
    out << "\"reason\": \"" << JsonEscape(decision.reason) << "\"";
    out << "}";
}

std::string BuildSolverReport(const EpgTaskManifest &manifest,
                              const epg::GraphProfileMetadata &metadata,
                              std::uint64_t nowMs,
                              const std::vector<SolverDecision> &decisions)
{
    const auto score = ScoreDecisions(decisions);
    std::ostringstream out;
    out << "{\n";
    out << "  \"schema\": \"" << epg::SOLVER_REPORT_SCHEMA << "\",\n";
    out << "  \"targetGraph\": \"" << JsonEscape(manifest.subgraphName)
        << "\",\n";
    out << "  \"topologyVersion\": \""
        << JsonEscape(manifest.topologyVersion) << "\",\n";
    out << "  \"sourceProfile\": \"" << JsonEscape(manifest.subgraphName)
        << "\",\n";
    out << "  \"sourceTimestampMs\": " << metadata.timestampMs << ",\n";
    out << "  \"generatedAtMs\": " << nowMs << ",\n";
    out << "  \"solverVersion\": \"" << epg::NATIVE_HEURISTIC_SOLVER_VERSION
        << "\",\n";
    out << "  \"objective\": {\n";
    out << "    \"name\": "
           "\"minimize_epg_pressure_overload_deadline_and_scheduling_penalty\","
        << "\n";
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

epg::OptimizedGraph ValidateGeneratedArtifacts(
    const EpgTaskManifest &manifest,
    const epg::GraphProfile &sourceProfile,
    const std::string &optimizedJson,
    const std::string &reportJson)
{
    auto optimized = epg::ParseOptimizedGraphJson(optimizedJson);
    ValidateEpgOptimizedGraphManifest(manifest, optimized);
    const auto report = epg::ParseSolverReportJson(reportJson);
    ValidateEpgSolverReport(manifest, sourceProfile, optimized, report);
    return optimized;
}

EpgRuntimeOptimizerResult WriteOptimizedConfig(const EpgTaskManifest &manifest,
                                               const epg::GraphProfile &profile,
                                               std::uint64_t nowMs)
{
    const auto &paths = manifest.artifactPaths;
    std::vector<SolverDecision> decisions;
    const auto config =
        OptimizeGraphConfig(manifest, profile.topology, profile.diagnostics,
                            decisions);
    const std::string json =
        epg::GraphConfigToJson(config, MakeOptimizerStrings(manifest),
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
    result.solverVersion = epg::NATIVE_HEURISTIC_SOLVER_VERSION;
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
    epg::GraphProfile profile;
    try {
        profile = epg::ParseGraphProfileJson(profileText);
    } catch (const std::exception &error) {
        return {false, false, error.what()};
    }
    const auto &metadata = profile.metadata;
    if (metadata.schema != epg::GRAPH_PROFILE_SCHEMA) {
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

} // namespace smartdrone::core::application
