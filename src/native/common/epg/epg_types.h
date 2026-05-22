#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace Epg {

using PortId = std::uint32_t;

inline constexpr const char *GRAPH_PROFILE_SCHEMA =
    "smartdrone.epg.profile.v1";
inline constexpr const char *OPTIMIZED_GRAPH_SCHEMA =
    "smartdrone.epg.optimized_config.v1";
inline constexpr const char *SOLVER_REPORT_SCHEMA =
    "smartdrone.epg.solver_report.v1";
inline constexpr const char *EXACT_SOLVER_OBJECTIVE =
    "global_minimize_predicted_epg_penalty_discrete_topology";
inline constexpr const char *NATIVE_EXACT_SOLVER_VERSION =
    "native-exact-v1";
inline constexpr const char *NATIVE_HEURISTIC_SOLVER_VERSION =
    NATIVE_EXACT_SOLVER_VERSION;

enum class OverflowPolicy {
    DropNewest,
    OverwriteOldest
};

enum class TriggerMode {
    Periodic,
    AnyQueueReady,
    AllQueueReady,
    PeriodicOrAnyQueueReady
};

struct QueueDiagnostics {
    std::atomic<std::uint64_t> pushed{0};
    std::atomic<std::uint64_t> popped{0};
    std::atomic<std::uint64_t> droppedNewest{0};
    std::atomic<std::uint64_t> overwrittenOldest{0};
    std::atomic<std::uint64_t> wakeups{0};
    std::atomic<std::uint64_t> maxDepthObserved{0};
    std::atomic<std::uint64_t> firstActivityMs{0};
    std::atomic<std::uint64_t> lastActivityMs{0};
};

struct QueueDiagnosticsSnapshot {
    std::uint64_t pushed{};
    std::uint64_t popped{};
    std::uint64_t droppedNewest{};
    std::uint64_t overwrittenOldest{};
    std::uint64_t wakeups{};
    std::uint64_t maxDepthObserved{};
    std::uint64_t firstActivityMs{};
    std::uint64_t lastActivityMs{};
};

struct TaskDiagnostics {
    std::atomic<std::uint64_t> loopCount{0};
    std::atomic<std::uint64_t> errorCount{0};
    std::atomic<std::uint64_t> idleWakeups{0};
    std::atomic<std::uint64_t> lastLoopUs{0};
    std::atomic<std::uint64_t> maxLoopUs{0};
    std::atomic<std::uint64_t> totalLoopUs{0};
    std::atomic<std::uint64_t> resourceWaitCount{0};
    std::atomic<std::uint64_t> lastResourceWaitUs{0};
    std::atomic<std::uint64_t> maxResourceWaitUs{0};
    std::atomic<std::uint64_t> totalResourceWaitUs{0};
    std::atomic<std::uint64_t> firstLoopMs{0};
    std::atomic<std::uint64_t> lastLoopMs{0};
    std::atomic<std::uint64_t> budgetOverrunCount{0};
    std::atomic<std::uint64_t> deadlineMissCount{0};
    std::atomic<std::uint64_t> schedulingErrorCount{0};
    std::atomic<int> lastSchedulingError{0};
};

struct TaskDiagnosticsSnapshot {
    std::uint64_t loopCount{};
    std::uint64_t errorCount{};
    std::uint64_t idleWakeups{};
    std::uint64_t lastLoopUs{};
    std::uint64_t maxLoopUs{};
    std::uint64_t p50LoopUs{};
    std::uint64_t p90LoopUs{};
    std::uint64_t p99LoopUs{};
    std::uint64_t totalLoopUs{};
    std::uint64_t resourceWaitCount{};
    std::uint64_t lastResourceWaitUs{};
    std::uint64_t maxResourceWaitUs{};
    std::uint64_t totalResourceWaitUs{};
    std::uint64_t firstLoopMs{};
    std::uint64_t lastLoopMs{};
    std::uint64_t budgetOverrunCount{};
    std::uint64_t deadlineMissCount{};
    std::uint64_t schedulingErrorCount{};
    int lastSchedulingError{};
};

struct QueueConfig {
    std::string name;
    std::string type;
    std::size_t depth{};
    OverflowPolicy overflow{OverflowPolicy::DropNewest};
};

struct TriggerConfig {
    TriggerMode mode{TriggerMode::Periodic};
    std::chrono::milliseconds interval{0};
    std::vector<std::string> queues;
};

struct TaskSchedulingConfig {
    std::string resource{"cpu"};
    int cpuAffinity{-1};
    std::uint64_t budgetUs{0};
    std::uint64_t deadlineUs{0};
    std::vector<PortId> backpressureOutputs;
    bool realtime{false};
    int priority{0};
};

struct TaskConfig {
    std::string name;
    std::string type;
    TriggerConfig trigger;
    TaskSchedulingConfig scheduling;
    std::map<PortId, std::string> inputs;
    std::map<PortId, std::string> outputs;
};

struct GraphConfig {
    std::vector<QueueConfig> queues;
    std::vector<TaskConfig> tasks;
};

struct GraphProfileMetadata {
    std::string schema;
    std::string graph;
    std::string topologyVersion;
    std::uint64_t timestampMs{};
};

struct GraphProfileTaskCatalogEntry {
    std::string taskType;
    std::string role;
    std::string resource;
    std::uint64_t budgetUs{};
    std::uint64_t deadlineUs{};
    bool replaceable{false};
};

struct OptimizedGraphMetadata {
    std::string schema;
    std::string targetGraph;
    std::string topologyVersion;
    std::string solverVersion;
    std::string sourceProfile;
    std::uint64_t sourceTimestampMs{};
    std::uint64_t generatedAtMs{};
};

struct OptimizedGraph {
    OptimizedGraphMetadata metadata;
    GraphConfig config;
};

struct SolverReportMetadata {
    std::string schema;
    std::string targetGraph;
    std::string topologyVersion;
    std::string sourceProfile;
    std::string solverVersion;
    std::uint64_t sourceTimestampMs{};
    std::uint64_t generatedAtMs{};
};

struct SolverReportScore {
    std::uint64_t queuePressure{};
    std::uint64_t periodicOverloadUs{};
    std::uint64_t resourceWaitUs{};
    std::uint64_t schedulingErrors{};
    std::uint64_t budgetOverruns{};
    std::uint64_t deadlineMisses{};
    std::uint64_t utilizationOverPpm{};
    std::uint64_t topologyPenalty{};
    std::uint64_t totalPenalty{};
};

struct SolverReportConstraints {
    std::uint64_t maxQueueDepth{};
    std::uint64_t maxPeriodicIntervalMs{};
    std::uint64_t targetUtilizationPpm{};
};

struct SolverReportDecision {
    std::string kind;
    std::string name;
    std::string reason;
    std::string catalogRole;
    bool replaceable{false};
    std::uint64_t depthBefore{};
    std::uint64_t depthAfter{};
    std::uint64_t pressureBefore{};
    std::uint64_t pressureAfter{};
    std::uint64_t maxDepthObserved{};
    std::uint64_t droppedNewest{};
    std::uint64_t overwrittenOldest{};
    std::uint64_t pushedPerSecond{};
    std::uint64_t poppedPerSecond{};
    std::uint64_t droppedPerSecond{};
    std::uint64_t intervalBeforeMs{};
    std::uint64_t intervalAfterMs{};
    std::uint64_t maxLoopUs{};
    std::uint64_t averageLoopUs{};
    std::uint64_t p90LoopUs{};
    std::uint64_t p99LoopUs{};
    std::uint64_t effectiveLoopUs{};
    std::uint64_t resourceWaitCount{};
    std::uint64_t maxResourceWaitUs{};
    std::uint64_t averageResourceWaitUs{};
    std::uint64_t totalResourceWaitUs{};
    std::uint64_t utilizationPpm{};
    std::uint64_t targetUtilizationPpm{};
    std::uint64_t budgetUs{};
    std::uint64_t deadlineUs{};
    std::uint64_t budgetOverrunCount{};
    std::uint64_t deadlineMissCount{};
    std::uint64_t schedulingErrorCount{};
    std::uint64_t topologyPenalty{};
    std::vector<PortId> backpressureBefore;
    std::vector<PortId> backpressureAfter;
};

struct SolverReport {
    SolverReportMetadata metadata;
    std::string objectiveName;
    SolverReportScore score;
    SolverReportConstraints constraints;
    std::vector<SolverReportDecision> decisions;
};

struct QueueProfileMetrics {
    std::uint64_t maxDepthObserved{};
    std::uint64_t droppedNewest{};
    std::uint64_t overwrittenOldest{};
    std::uint64_t pushedPerSecond{};
    std::uint64_t poppedPerSecond{};
    std::uint64_t droppedPerSecond{};
};

struct TaskProfileMetrics {
    std::uint64_t maxLoopUs{};
    std::uint64_t averageLoopUs{};
    std::uint64_t p90LoopUs{};
    std::uint64_t p99LoopUs{};
    std::uint64_t resourceWaitCount{};
    std::uint64_t maxResourceWaitUs{};
    std::uint64_t averageResourceWaitUs{};
    std::uint64_t totalResourceWaitUs{};
    std::uint64_t utilizationPpm{};
    std::uint64_t budgetOverrunCount{};
    std::uint64_t deadlineMissCount{};
    std::uint64_t schedulingErrorCount{};
};

struct GraphProfileDiagnostics {
    std::map<std::string, QueueProfileMetrics> queues;
    std::map<std::string, TaskProfileMetrics> tasks;
};

struct GraphProfile {
    GraphProfileMetadata metadata;
    std::vector<GraphProfileTaskCatalogEntry> taskCatalog;
    GraphConfig topology;
    GraphProfileDiagnostics diagnostics;
};

class Registry;

GraphProfile ParseGraphProfileJson(const std::string &jsonText);
GraphProfileMetadata ParseGraphProfileMetadataJson(
    const std::string &jsonText);
GraphProfileDiagnostics ParseGraphProfileDiagnosticsJson(
    const std::string &jsonText);
OptimizedGraphMetadata ParseOptimizedGraphMetadataJson(
    const std::string &jsonText);
OptimizedGraph ParseOptimizedGraphJson(const std::string &jsonText);
SolverReportMetadata ParseSolverReportMetadataJson(
    const std::string &jsonText);
SolverReport ParseSolverReportJson(const std::string &jsonText);
GraphConfig ParseGraphConfigJson(const std::string &jsonText);
GraphConfig ParseGraphConfigJsonField(const std::string &jsonText,
                                      const std::string &field);
GraphConfig ParseGraphConfigJsonFile(const std::string &path);
GraphConfig ParseGraphConfigMermaid(const std::string &mermaidText);
GraphConfig ParseGraphConfigMermaidFile(const std::string &path);
GraphConfig ParseGraphConfigMermaid(const std::string &mermaidText,
                                    Registry &registry);
GraphConfig ParseGraphConfigMermaidFile(const std::string &path,
                                        Registry &registry);
GraphConfig ParseGraphConfigMermaidSubgraph(const std::string &mermaidText,
                                            const std::string &subgraphName,
                                            Registry &registry);
GraphConfig ParseGraphConfigMermaidSubgraphFile(
    const std::string &path,
    const std::string &subgraphName,
    Registry &registry);
GraphConfig ParseGraphConfigDot(const std::string &dotText,
                                const std::string &subgraphName,
                                Registry &registry);
GraphConfig ParseGraphConfigDotFile(const std::string &path,
                                    const std::string &subgraphName,
                                    Registry &registry);
std::string GraphConfigToJson(
    const GraphConfig &config,
    const std::map<std::string, std::string> &stringMetadata = {},
    const std::map<std::string, std::uint64_t> &numericMetadata = {});

} // namespace Epg
