#include "core/application/epg/epg_task_manifest.h"

#include <sstream>
#include <set>
#include <stdexcept>
#include <string>

namespace smartdrone::core::application {
namespace {

constexpr const char *SYSTEM_RUNTIME_GRAPH_NAME =
    "cluster_system_runtime_graph";
constexpr const char *SLAM_SESSION_GRAPH_NAME = "cluster_slam_session_graph";
constexpr const char *CALIB_SESSION_GRAPH_NAME = "cluster_calib_session_graph";
constexpr const char *VEHICLE_TELEMETRY_RX_TASK_TYPE =
    "VehicleTelemetryRxTask";
constexpr const char *LEGACY_MAVLINK_RX_TASK_TYPE = "MavlinkRxTask";
constexpr const char *EPG_TOPOLOGY_PATH = "config/epg/epg_topology.dot";
constexpr const char *EPG_TOPOLOGY_REVISION = "v2";
constexpr const char *EPG_SNAPSHOT_DIR = "/tmp";
constexpr const char *EPG_OPTIMIZED_DIR = "output/epg";
const EpgTaskTopologySpec EPG_TOPOLOGY_SPEC{
    EPG_TOPOLOGY_PATH,
    EPG_TOPOLOGY_REVISION,
};

const EpgTaskManifest SYSTEM_RUNTIME_MANIFEST{
    EpgDomain::SystemRuntime,
    SYSTEM_RUNTIME_GRAPH_NAME,
    EPG_TOPOLOGY_SPEC.path,
    BuildEpgTaskTopologyVersion(EPG_TOPOLOGY_SPEC),
    BuildEpgTaskArtifactPaths({"smartdrone_epg_system",
                               "optimized_system_runtime_graph"}),
    {
        {LEGACY_MAVLINK_RX_TASK_TYPE, VEHICLE_TELEMETRY_RX_TASK_TYPE},
    },
    {},
    {
        {VEHICLE_TELEMETRY_RX_TASK_TYPE, "telemetry_rx", "cpu", 1000, 2000, false},
        {"SetpointStreamTask", "flight_setpoint", "cpu", 1000, 2000, false},
        {"UdpCommandTask", "command_rx", "cpu", 1000, 2000, false},
        {"ManualControlTask", "manual_control", "cpu", 1000, 5000, false},
        {"ForceRestartTask", "restart_guard", "cpu", 1000, 5000, false},
        {"RuntimeSupervisorTask", "session_supervisor", "cpu", 1000, 10000, false},
        {"EpgRedeployTask", "epg_redeploy", "cpu", 1000, 5000, false},
        {"DiscoveryBeaconTask", "discovery", "cpu", 1000, 10000, false},
        {"EpgDfxSnapshotTask", "dfx_snapshot", "cpu", 2000, 10000, false},
        {"EpgOptimizeTask", "epg_optimizer", "cpu", 5000, 50000, false},
    },
};

const EpgTaskManifest SLAM_SESSION_MANIFEST{
    EpgDomain::SlamSession,
    SLAM_SESSION_GRAPH_NAME,
    EPG_TOPOLOGY_SPEC.path,
    BuildEpgTaskTopologyVersion(EPG_TOPOLOGY_SPEC),
    BuildEpgTaskArtifactPaths({"smartdrone_epg_slam",
                               "optimized_slam_session_graph"}),
    {},
    {
        {"SlamResourceTask", true, false, false},
        {"SlamClockTask", true, false, false},
        {"SlamImuPollTask", false, true, true},
    },
    {
        {"SlamResourceTask", "resource_open", "cpu", 5000, 50000, false},
        {"SlamClockTask", "frame_clock", "cpu", 500, 2000, false},
        {"SlamImuPollTask", "imu_poll", "cpu", 1000, 2000, false},
        {"SlamBackendTickTask", "backend_maintenance", "cpu", 2000, 5000, true},
        {"SlamImuGateTask", "sensor_gate", "cpu", 1000, 2000, false},
        {"SlamAcquireTask", "frame_acquire", "cpu", 12000, 16000, true},
        {"SlamTrackingTask", "visual_tracking", "cpu", 24000, 33000, true},
        {"SlamPosePostprocessTask", "pose_postprocess", "cpu", 3000, 5000, true},
        {"SlamPointCloudTask", "point_cloud", "cpu", 5000, 10000, true},
        {"SlamLivePoseTask", "live_pose", "cpu", 2000, 5000, false},
        {"SlamMavlinkTask", "pose_publish", "cpu", 2000, 5000, false},
        {"SlamUdpTask", "preview_stream", "cpu", 10000, 16000, true},
        {"SlamDfxTask", "slam_dfx", "cpu", 5000, 10000, false},
        {"SlamMonitorTask", "session_monitor", "cpu", 2000, 10000, false},
        {"EpgDfxSnapshotTask", "dfx_snapshot", "cpu", 2000, 10000, false},
    },
};

const EpgTaskManifest CALIB_SESSION_MANIFEST{
    EpgDomain::CalibSession,
    CALIB_SESSION_GRAPH_NAME,
    EPG_TOPOLOGY_SPEC.path,
    BuildEpgTaskTopologyVersion(EPG_TOPOLOGY_SPEC),
    BuildEpgTaskArtifactPaths({"smartdrone_epg_calib",
                               "optimized_calib_session_graph"}),
    {},
    {},
    {
        {"CalibResourceTask", "resource_open", "cpu", 5000, 50000, false},
        {"CalibClockTask", "capture_clock", "cpu", 500, 2000, false},
        {"CalibCameraAcquireTask", "frame_acquire", "cpu", 12000, 20000, true},
        {"CalibPacingFilterTask", "save_pacing", "cpu", 1000, 5000, false},
        {"CalibStorageWriteTask", "storage_write", "cpu", 20000, 50000, true},
        {"CalibImuWriterTask", "imu_write", "cpu", 2000, 5000, false},
        {"CalibUdpPreviewTask", "preview_stream", "cpu", 10000, 16000, true},
        {"CalibCompletionTask", "completion", "cpu", 1000, 5000, false},
        {"CalibFlushSyncTask", "flush_sync", "cpu", 50000, 200000, false},
        {"CalibMonitorTask", "session_monitor", "cpu", 1000, 5000, false},
        {"EpgDfxSnapshotTask", "dfx_snapshot", "cpu", 2000, 10000, false},
    },
};

std::string JsonEscape(const std::string &value)
{
    std::ostringstream out;
    for (const char ch : value) {
        if (ch == '\\' || ch == '"') {
            out << '\\';
        }
        out << ch;
    }
    return out.str();
}

void ValidateTaskFactory(const EpgTaskFactoryResolver &resolver,
                         const std::string &taskType)
{
    if (resolver(taskType)) {
        return;
    }
    throw std::runtime_error("missing EventPipelineGraph task factory: " +
                             taskType);
}

std::string TaskGraphLabel(const EpgTaskManifest &manifest)
{
    return "EventPipelineGraph subgraph '" + manifest.subgraphName + "'";
}

bool ArtifactPathsComplete(const EpgTaskArtifactPaths &paths)
{
    return !paths.dfxSnapshotPath.empty() &&
           !paths.profilePath.empty() &&
           !paths.optimizedConfigPath.empty() &&
           !paths.solverReportPath.empty();
}

void ValidateManifestMetadata(const EpgTaskManifest &manifest)
{
    if (manifest.subgraphName.empty()) {
        throw std::runtime_error("EventPipelineGraph manifest subgraph is empty");
    }
    if (manifest.topologyPath.empty() || manifest.topologyVersion.empty()) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " topology metadata is incomplete");
    }
    if (!ArtifactPathsComplete(manifest.artifactPaths)) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " artifact paths are incomplete");
    }
}

void ValidateGraphTaskTypeAllowed(const EpgTaskManifest &manifest,
                                  const epg::TaskConfig &task,
                                  const std::set<std::string> &allowedTypes)
{
    if (allowedTypes.find(task.type) != allowedTypes.end()) {
        return;
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " uses task type outside manifest: " +
                             task.name + " type=" + task.type);
}

const EpgTaskCatalogEntry &RequireCatalogEntry(
    const EpgTaskManifest &manifest,
    const std::string &taskType)
{
    for (const auto &entry : manifest.catalog) {
        if (entry.taskType == taskType) {
            return entry;
        }
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " missing catalog task type: " + taskType);
}

const EpgTaskCatalogEntry *FindCatalogEntry(
    const EpgTaskManifest &manifest,
    const std::string &taskType)
{
    for (const auto &entry : manifest.catalog) {
        if (entry.taskType == taskType) {
            return &entry;
        }
    }
    return nullptr;
}

void ApplyCatalogDefaultsToTask(const EpgTaskCatalogEntry &entry,
                                epg::TaskConfig &task)
{
    task.scheduling.resource = entry.resource;
    task.scheduling.budgetUs = entry.budgetUs;
    task.scheduling.deadlineUs = entry.deadlineUs;
}

void ValidateTaskSchedulingCatalogMatch(const EpgTaskManifest &manifest,
                                        const epg::TaskConfig &task)
{
    const auto &entry = RequireCatalogEntry(manifest, task.type);
    const auto &scheduling = task.scheduling;
    if (scheduling.resource != entry.resource ||
        scheduling.budgetUs != entry.budgetUs ||
        scheduling.deadlineUs != entry.deadlineUs) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " task scheduling catalog mismatch: " +
                                 task.name + " type=" + task.type);
    }
}

void ValidateManifestTaskTypeUsed(const EpgTaskManifest &manifest,
                                  const std::string &taskType,
                                  const std::set<std::string> &usedTypes)
{
    if (usedTypes.find(taskType) != usedTypes.end()) {
        return;
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " does not use manifest task type: " +
                             taskType);
}

void ValidateCatalogTaskTypeUnique(const EpgTaskManifest &manifest,
                                   std::set<std::string> &catalogTypes,
                                   const std::string &taskType)
{
    if (catalogTypes.insert(taskType).second) {
        return;
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " duplicates catalog task type: " +
                             taskType);
}

void ValidateCatalogEntrySemantics(const EpgTaskManifest &manifest,
                                   const EpgTaskCatalogEntry &entry)
{
    if (entry.taskType.empty()) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " catalog task type is empty");
    }
    if (entry.role.empty() || entry.resource.empty()) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " catalog metadata is incomplete: " +
                                 entry.taskType);
    }
    if (entry.budgetUs == 0 || entry.deadlineUs == 0 ||
        entry.deadlineUs < entry.budgetUs) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " catalog timing is invalid: " +
                                 entry.taskType);
    }
}

std::set<std::string> ValidateManifestCatalog(
    const EpgTaskManifest &manifest)
{
    if (manifest.catalog.empty()) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " catalog is empty");
    }
    std::set<std::string> catalogTypes;
    for (const auto &entry : manifest.catalog) {
        ValidateCatalogEntrySemantics(manifest, entry);
        ValidateCatalogTaskTypeUnique(manifest, catalogTypes, entry.taskType);
    }
    return catalogTypes;
}

void ValidateAliasTargetDeclared(const EpgTaskManifest &manifest,
                                 const EpgTaskAliasManifestEntry &alias,
                                 const std::set<std::string> &catalogTypes)
{
    if (catalogTypes.find(alias.targetType) != catalogTypes.end()) {
        return;
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " alias target outside catalog: " +
                             alias.alias + " -> " + alias.targetType);
}

void ValidateAliasNameAvailable(const EpgTaskManifest &manifest,
                                std::set<std::string> &aliasTypes,
                                const EpgTaskAliasManifestEntry &alias,
                                const std::set<std::string> &catalogTypes)
{
    if (alias.alias.empty() || alias.targetType.empty()) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " alias metadata is incomplete");
    }
    if (catalogTypes.find(alias.alias) != catalogTypes.end()) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " alias shadows catalog task type: " +
                                 alias.alias);
    }
    if (aliasTypes.insert(alias.alias).second) {
        return;
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " duplicates task alias: " + alias.alias);
}

void ValidateManifestAliases(const EpgTaskManifest &manifest,
                             const std::set<std::string> &catalogTypes)
{
    std::set<std::string> aliasTypes;
    for (const auto &alias : manifest.aliases) {
        ValidateAliasNameAvailable(manifest, aliasTypes, alias, catalogTypes);
        ValidateAliasTargetDeclared(manifest, alias, catalogTypes);
    }
}

bool RuntimeTuningEnabled(const EpgTaskRuntimeTuningEntry &entry)
{
    return entry.interval || entry.realtime || entry.priority;
}

const EpgTaskRuntimeTuningEntry *FindRuntimeTuning(
    const std::vector<EpgTaskRuntimeTuningEntry> &entries,
    const std::string &taskName)
{
    for (const auto &entry : entries) {
        if (entry.taskName == taskName) {
            return &entry;
        }
    }
    return nullptr;
}

bool GraphUsesTaskName(const epg::GraphConfig &graphConfig,
                       const std::string &taskName)
{
    for (const auto &task : graphConfig.tasks) {
        if (task.name == taskName) {
            return true;
        }
    }
    return false;
}

void ValidateRuntimeTuningEntry(const EpgTaskManifest &manifest,
                                const EpgTaskRuntimeTuningEntry &entry)
{
    if (entry.taskName.empty() || !RuntimeTuningEnabled(entry)) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " runtime tuning metadata is incomplete");
    }
}

void ValidateRuntimeTuningRequestAllowed(
    const EpgTaskManifest &manifest,
    const EpgTaskRuntimeTuningEntry &allowed,
    const EpgTaskRuntimeTuningEntry &requested)
{
    if ((requested.interval && !allowed.interval) ||
        (requested.realtime && !allowed.realtime) ||
        (requested.priority && !allowed.priority)) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " runtime tuning is not allowed: " +
                                 requested.taskName);
    }
}

void ValidateRuntimeTuningTaskUnique(const EpgTaskManifest &manifest,
                                     std::set<std::string> &taskNames,
                                     const std::string &taskName)
{
    if (taskNames.insert(taskName).second) {
        return;
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " duplicates runtime tuning task: " + taskName);
}

void ValidateManifestRuntimeTuning(const EpgTaskManifest &manifest)
{
    std::set<std::string> taskNames;
    for (const auto &entry : manifest.runtimeTuning) {
        ValidateRuntimeTuningEntry(manifest, entry);
        ValidateRuntimeTuningTaskUnique(manifest, taskNames, entry.taskName);
    }
}

void ValidateGraphRuntimeTuningDeclared(
    const EpgTaskManifest &manifest,
    const epg::GraphConfig &graphConfig)
{
    for (const auto &entry : manifest.runtimeTuning) {
        if (GraphUsesTaskName(graphConfig, entry.taskName)) {
            continue;
        }
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " runtime tuning task is missing: " +
                                 entry.taskName);
    }
}

std::uint64_t SolverReportTotalPenalty(
    const epg::SolverReportScore &score)
{
    return score.queuePressure * 1000 + score.periodicOverloadUs +
           score.schedulingErrors * 10000 + score.budgetOverruns * 2000 +
           score.deadlineMisses * 5000 + score.utilizationOverPpm;
}

void AddQueueDecisionScore(const epg::SolverReportDecision &decision,
                           epg::SolverReportScore &score)
{
    score.queuePressure += decision.pressureBefore;
}

void AddTaskDecisionScore(const epg::SolverReportDecision &decision,
                          epg::SolverReportScore &score)
{
    if (decision.effectiveLoopUs > decision.intervalBeforeMs * 1000) {
        score.periodicOverloadUs +=
            decision.effectiveLoopUs - decision.intervalBeforeMs * 1000;
    }
    score.schedulingErrors += decision.schedulingErrorCount;
    score.budgetOverruns += decision.budgetOverrunCount;
    score.deadlineMisses += decision.deadlineMissCount;
    if (decision.utilizationPpm > decision.targetUtilizationPpm) {
        score.utilizationOverPpm +=
            decision.utilizationPpm - decision.targetUtilizationPpm;
    }
}

epg::SolverReportScore BuildSolverReportScore(
    const std::vector<epg::SolverReportDecision> &decisions)
{
    epg::SolverReportScore score;
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

void ValidateSolverReportScore(const epg::SolverReport &report)
{
    const auto expected = BuildSolverReportScore(report.decisions);
    if (report.score.queuePressure != expected.queuePressure ||
        report.score.periodicOverloadUs != expected.periodicOverloadUs ||
        report.score.schedulingErrors != expected.schedulingErrors ||
        report.score.budgetOverruns != expected.budgetOverruns ||
        report.score.deadlineMisses != expected.deadlineMisses ||
        report.score.utilizationOverPpm != expected.utilizationOverPpm ||
        report.score.totalPenalty != expected.totalPenalty) {
        throw std::runtime_error("solver report score mismatch");
    }
}

void ValidateSolverReportDecisionCoverage(
    const epg::GraphConfig &graphConfig,
    const epg::SolverReport &report)
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

const epg::QueueConfig *FindQueueConfig(
    const epg::GraphConfig &graphConfig,
    const std::string &name)
{
    for (const auto &queue : graphConfig.queues) {
        if (queue.name == name) {
            return &queue;
        }
    }
    return nullptr;
}

const epg::TaskConfig *FindTaskConfig(
    const epg::GraphConfig &graphConfig,
    const std::string &name)
{
    for (const auto &task : graphConfig.tasks) {
        if (task.name == name) {
            return &task;
        }
    }
    return nullptr;
}

void ValidateSolverReportConstraints(const epg::SolverReport &report)
{
    const auto &constraints = report.constraints;
    if (constraints.maxQueueDepth == 0 ||
        constraints.maxPeriodicIntervalMs == 0 ||
        constraints.targetUtilizationPpm == 0) {
        throw std::runtime_error("solver report constraint invalid");
    }
}

void ValidateQueueSolverDecision(
    const epg::GraphConfig &graphConfig,
    const epg::SolverReportConstraints &constraints,
    const epg::SolverReportDecision &decision)
{
    const auto *queue = FindQueueConfig(graphConfig, decision.name);
    if (!queue) {
        throw std::runtime_error("solver report queue decision target missing: " +
                                 decision.name);
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
    const std::string expectedReason =
        decision.depthAfter != decision.depthBefore ? "increase_depth" : "keep";
    if (decision.reason != expectedReason) {
        throw std::runtime_error("solver report queue reason mismatch: " +
                                 decision.name);
    }
}

std::vector<std::string> TaskDecisionReasons(
    const epg::SolverReportDecision &decision)
{
    std::vector<std::string> reasons;
    if (decision.utilizationPpm > decision.targetUtilizationPpm) {
        reasons.push_back("utilization_over_target");
    }
    if (decision.budgetOverrunCount > 0 ||
        (decision.budgetUs > 0 && decision.effectiveLoopUs > decision.budgetUs)) {
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
    const epg::SolverReportDecision &decision)
{
    if (decision.intervalAfterMs != decision.intervalBeforeMs) {
        return "increase_interval";
    }
    const auto reason = JoinTaskDecisionReasons(TaskDecisionReasons(decision));
    if (decision.replaceable) {
        return reason;
    }
    return reason == "keep" ? "not_replaceable" : "not_replaceable+" + reason;
}

void ValidateTaskSolverDecision(
    const EpgTaskManifest &manifest,
    const epg::GraphConfig &graphConfig,
    const epg::SolverReportConstraints &constraints,
    const epg::SolverReportDecision &decision)
{
    const auto *task = FindTaskConfig(graphConfig, decision.name);
    if (!task) {
        throw std::runtime_error("solver report task decision target missing: " +
                                 decision.name);
    }
    const auto &catalog = RequireCatalogEntry(manifest, task->type);
    if (decision.intervalAfterMs >
        constraints.maxPeriodicIntervalMs) {
        throw std::runtime_error("solver report task interval constraint mismatch: " +
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
    const epg::GraphConfig &graphConfig,
    const epg::SolverReport &report)
{
    ValidateSolverReportConstraints(report);
    for (const auto &decision : report.decisions) {
        if (decision.kind == "queue") {
            ValidateQueueSolverDecision(
                graphConfig, report.constraints, decision);
            continue;
        }
        ValidateTaskSolverDecision(
            manifest, graphConfig, report.constraints, decision);
    }
}

std::string BuildPath(const char *directory, const std::string &stem,
                      const char *suffix)
{
    return std::string(directory) + "/" + stem + suffix;
}

} // namespace

EpgTaskArtifactPaths BuildEpgTaskArtifactPaths(
    const EpgTaskArtifactSpec &spec)
{
    return {
        BuildPath(EPG_SNAPSHOT_DIR, spec.snapshotStem, ".json"),
        BuildPath(EPG_SNAPSHOT_DIR, spec.snapshotStem, "_profile.json"),
        BuildPath(EPG_OPTIMIZED_DIR, spec.optimizedStem, ".json"),
        BuildPath(EPG_OPTIMIZED_DIR, spec.optimizedStem, "_report.json"),
    };
}

std::string BuildEpgTaskTopologyVersion(
    const EpgTaskTopologySpec &spec)
{
    return spec.path + ":" + spec.revision;
}

const EpgTaskManifest &EpgManifestForDomain(EpgDomain domain)
{
    switch (domain) {
    case EpgDomain::SystemRuntime:
        return SYSTEM_RUNTIME_MANIFEST;
    case EpgDomain::SlamSession:
        return SLAM_SESSION_MANIFEST;
    case EpgDomain::CalibSession:
        return CALIB_SESSION_MANIFEST;
    }
    throw std::runtime_error("unsupported EPG domain");
}

std::vector<std::string> EpgTaskCatalogTypes(
    const EpgTaskManifest &manifest)
{
    std::vector<std::string> taskTypes;
    taskTypes.reserve(manifest.catalog.size());
    for (const auto &entry : manifest.catalog) {
        taskTypes.push_back(entry.taskType);
    }
    return taskTypes;
}

std::string EpgTaskCatalogJson(const EpgTaskManifest &manifest)
{
    std::ostringstream out;
    out << "[";
    for (std::size_t i = 0; i < manifest.catalog.size(); ++i) {
        const auto &entry = manifest.catalog[i];
        if (i != 0) {
            out << ",";
        }
        out << "{\"taskType\":\"" << JsonEscape(entry.taskType) << "\",";
        out << "\"role\":\"" << JsonEscape(entry.role) << "\",";
        out << "\"resource\":\"" << JsonEscape(entry.resource) << "\",";
        out << "\"budgetUs\":" << entry.budgetUs << ",";
        out << "\"deadlineUs\":" << entry.deadlineUs << ",";
        out << "\"replaceable\":" << (entry.replaceable ? "true" : "false");
        out << "}";
    }
    out << "]";
    return out.str();
}

void ApplyEpgTaskCatalogDefaults(
    const EpgTaskManifest &manifest,
    epg::GraphConfig &graphConfig)
{
    ValidateManifestMetadata(manifest);
    ValidateManifestCatalog(manifest);
    ValidateManifestRuntimeTuning(manifest);
    for (auto &task : graphConfig.tasks) {
        const auto *entry = FindCatalogEntry(manifest, task.type);
        if (!entry) {
            continue;
        }
        ApplyCatalogDefaultsToTask(*entry, task);
    }
}

void ValidateEpgTaskRuntimeTuning(
    const EpgTaskManifest &manifest,
    const epg::GraphConfig &graphConfig,
    const std::vector<EpgTaskRuntimeTuningEntry> &requestedTuning)
{
    ValidateManifestMetadata(manifest);
    ValidateManifestRuntimeTuning(manifest);
    for (const auto &requested : requestedTuning) {
        ValidateRuntimeTuningEntry(manifest, requested);
        const auto *allowed =
            FindRuntimeTuning(manifest.runtimeTuning, requested.taskName);
        if (!allowed) {
            throw std::runtime_error(TaskGraphLabel(manifest) +
                                     " runtime tuning task is not declared: " +
                                     requested.taskName);
        }
        ValidateRuntimeTuningRequestAllowed(manifest, *allowed, requested);
        if (!GraphUsesTaskName(graphConfig, requested.taskName)) {
            throw std::runtime_error(TaskGraphLabel(manifest) +
                                     " runtime tuning task is missing: " +
                                     requested.taskName);
        }
    }
}

void ValidateEpgTaskManifest(
    const EpgTaskManifest &manifest)
{
    ValidateManifestMetadata(manifest);
    const auto catalogTypes = ValidateManifestCatalog(manifest);
    ValidateManifestAliases(manifest, catalogTypes);
    ValidateManifestRuntimeTuning(manifest);
}

void ValidateEpgTaskFactoryManifest(
    const EpgTaskManifest &manifest,
    const EpgTaskFactoryResolver &resolver)
{
    ValidateManifestMetadata(manifest);
    if (!resolver) {
        throw std::runtime_error(
            "EventPipelineGraph task factory resolver must be callable");
    }
    const auto catalogTypes = ValidateManifestCatalog(manifest);
    for (const auto &entry : manifest.catalog) {
        ValidateTaskFactory(resolver, entry.taskType);
    }
    ValidateManifestAliases(manifest, catalogTypes);
    ValidateManifestRuntimeTuning(manifest);
    for (const auto &alias : manifest.aliases) {
        ValidateTaskFactory(resolver, alias.targetType);
    }
}

void ValidateEpgTaskGraphManifest(
    const EpgTaskManifest &manifest,
    const epg::GraphConfig &graphConfig)
{
    ValidateManifestMetadata(manifest);
    const auto allowedTypes = ValidateManifestCatalog(manifest);
    ValidateManifestRuntimeTuning(manifest);
    std::set<std::string> usedTypes;
    for (const auto &task : graphConfig.tasks) {
        ValidateGraphTaskTypeAllowed(manifest, task, allowedTypes);
        ValidateTaskSchedulingCatalogMatch(manifest, task);
        usedTypes.insert(task.type);
    }
    for (const auto &entry : manifest.catalog) {
        ValidateManifestTaskTypeUsed(manifest, entry.taskType, usedTypes);
    }
    ValidateGraphRuntimeTuningDeclared(manifest, graphConfig);
}

void ValidateEpgOptimizedGraphManifest(
    const EpgTaskManifest &manifest,
    const epg::OptimizedGraph &optimizedGraph)
{
    const auto &metadata = optimizedGraph.metadata;
    if (metadata.schema != epg::OPTIMIZED_GRAPH_SCHEMA) {
        throw std::runtime_error("optimized graph schema mismatch");
    }
    if (metadata.targetGraph != manifest.subgraphName) {
        throw std::runtime_error("optimized graph target mismatch");
    }
    if (metadata.topologyVersion != manifest.topologyVersion) {
        throw std::runtime_error("optimized graph topology version mismatch");
    }
    if (metadata.sourceProfile != manifest.subgraphName ||
        metadata.sourceTimestampMs == 0) {
        throw std::runtime_error("optimized graph source profile mismatch");
    }
    if (metadata.generatedAtMs == 0 ||
        metadata.generatedAtMs < metadata.sourceTimestampMs) {
        throw std::runtime_error("optimized graph generation timestamp invalid");
    }
    if (metadata.solverVersion.empty()) {
        throw std::runtime_error("optimized graph solver version missing");
    }
    ValidateEpgTaskGraphManifest(manifest, optimizedGraph.config);
}

void ValidateEpgSolverReportManifest(
    const EpgTaskManifest &manifest,
    const epg::OptimizedGraphMetadata &optimizedMetadata,
    const epg::SolverReportMetadata &reportMetadata)
{
    if (reportMetadata.schema != epg::SOLVER_REPORT_SCHEMA) {
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
    const epg::OptimizedGraph &optimizedGraph,
    const epg::SolverReport &report)
{
    ValidateEpgSolverReportManifest(manifest, optimizedGraph.metadata,
                                    report.metadata);
    if (report.objectiveName.empty()) {
        throw std::runtime_error("solver report objective missing");
    }
    ValidateSolverReportScore(report);
    ValidateSolverReportDecisionCoverage(optimizedGraph.config, report);
    ValidateSolverReportDecisionDetails(
        manifest, optimizedGraph.config, report);
}

} // namespace smartdrone::core::application
