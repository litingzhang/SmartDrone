#include "core/application/epg/epg_task_manifest.h"

#include <sstream>
#include <set>
#include <stdexcept>
#include <string>

namespace smartdrone::core::application {
namespace {

constexpr const char *kSystemRuntimeGraphName =
    "cluster_system_runtime_graph";
constexpr const char *kSlamSessionGraphName = "cluster_slam_session_graph";
constexpr const char *kCalibSessionGraphName = "cluster_calib_session_graph";
constexpr const char *kVehicleTelemetryRxTaskType = "VehicleTelemetryRxTask";
constexpr const char *kLegacyMavlinkRxTaskType = "MavlinkRxTask";
constexpr const char *kEpgTopologyVersion = "config/epg/epg_topology.dot:v1";

const EpgTaskManifest kSystemRuntimeManifest{
    EpgDomain::SystemRuntime,
    kSystemRuntimeGraphName,
    kEpgTopologyVersion,
    "/tmp/smartdrone_epg_system.json",
    "/tmp/smartdrone_epg_system_profile.json",
    "output/epg/optimized_system_runtime_graph.json",
    {
        kVehicleTelemetryRxTaskType,
        "SetpointStreamTask",
        "UdpCommandTask",
        "ManualControlTask",
        "ForceRestartTask",
        "RuntimeSupervisorTask",
        "DiscoveryBeaconTask",
        "EpgDfxSnapshotTask",
        "EpgOptimizeTask",
    },
    {
        {kLegacyMavlinkRxTaskType, kVehicleTelemetryRxTaskType},
    },
    {
        {kVehicleTelemetryRxTaskType, "telemetry_rx", "cpu", 1000, 2000, false},
        {"SetpointStreamTask", "flight_setpoint", "cpu", 1000, 2000, false},
        {"UdpCommandTask", "command_rx", "cpu", 1000, 2000, false},
        {"ManualControlTask", "manual_control", "cpu", 1000, 5000, false},
        {"ForceRestartTask", "restart_guard", "cpu", 1000, 5000, false},
        {"RuntimeSupervisorTask", "session_supervisor", "cpu", 1000, 10000, false},
        {"DiscoveryBeaconTask", "discovery", "cpu", 1000, 10000, false},
        {"EpgDfxSnapshotTask", "dfx_snapshot", "cpu", 2000, 10000, false},
        {"EpgOptimizeTask", "epg_optimizer", "cpu", 5000, 50000, false},
    },
};

const EpgTaskManifest kSlamSessionManifest{
    EpgDomain::SlamSession,
    kSlamSessionGraphName,
    kEpgTopologyVersion,
    "/tmp/smartdrone_epg_slam.json",
    "/tmp/smartdrone_epg_slam_profile.json",
    "output/epg/optimized_slam_session_graph.json",
    {
        "SlamResourceTask",
        "SlamClockTask",
        "SlamImuPollTask",
        "SlamBackendTickTask",
        "SlamImuGateTask",
        "SlamAcquireTask",
        "SlamTrackingTask",
        "SlamPosePostprocessTask",
        "SlamPointCloudTask",
        "SlamLivePoseTask",
        "SlamMavlinkTask",
        "SlamUdpTask",
        "SlamDfxTask",
        "SlamMonitorTask",
        "EpgDfxSnapshotTask",
    },
    {},
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

const EpgTaskManifest kCalibSessionManifest{
    EpgDomain::CalibSession,
    kCalibSessionGraphName,
    kEpgTopologyVersion,
    "/tmp/smartdrone_epg_calib.json",
    "/tmp/smartdrone_epg_calib_profile.json",
    "output/epg/optimized_calib_session_graph.json",
    {
        "CalibResourceTask",
        "CalibClockTask",
        "CalibCameraAcquireTask",
        "CalibPacingFilterTask",
        "CalibStorageWriteTask",
        "CalibImuWriterTask",
        "CalibUdpPreviewTask",
        "CalibCompletionTask",
        "CalibFlushSyncTask",
        "CalibMonitorTask",
        "EpgDfxSnapshotTask",
    },
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

std::set<std::string> MakeTaskTypeSet(const std::vector<std::string> &taskTypes)
{
    return std::set<std::string>(taskTypes.begin(), taskTypes.end());
}

std::string TaskGraphLabel(const EpgTaskManifest &manifest)
{
    return "EventPipelineGraph subgraph '" + manifest.subgraphName + "'";
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

void ValidateCatalogCoversTaskType(const EpgTaskManifest &manifest,
                                   const std::string &taskType,
                                   const std::set<std::string> &catalogTypes)
{
    if (catalogTypes.find(taskType) != catalogTypes.end()) {
        return;
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " task type missing catalog metadata: " +
                             taskType);
}

} // namespace

const EpgTaskManifest &EpgManifestForDomain(EpgDomain domain)
{
    switch (domain) {
    case EpgDomain::SystemRuntime:
        return kSystemRuntimeManifest;
    case EpgDomain::SlamSession:
        return kSlamSessionManifest;
    case EpgDomain::CalibSession:
        return kCalibSessionManifest;
    }
    throw std::runtime_error("unsupported EPG domain");
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

void ValidateEpgTaskFactoryManifest(
    const EpgTaskManifest &manifest,
    const EpgTaskFactoryResolver &resolver)
{
    if (!resolver) {
        throw std::runtime_error(
            "EventPipelineGraph task factory resolver must be callable");
    }
    for (const auto &taskType : manifest.taskTypes) {
        ValidateTaskFactory(resolver, taskType);
    }
    std::set<std::string> catalogTypes;
    for (const auto &entry : manifest.catalog) {
        catalogTypes.insert(entry.taskType);
    }
    for (const auto &taskType : manifest.taskTypes) {
        ValidateCatalogCoversTaskType(manifest, taskType, catalogTypes);
    }
    for (const auto &alias : manifest.aliases) {
        ValidateTaskFactory(resolver, alias.targetType);
    }
}

void ValidateEpgTaskGraphManifest(
    const EpgTaskManifest &manifest,
    const epg::GraphConfig &graphConfig)
{
    const std::set<std::string> allowedTypes =
        MakeTaskTypeSet(manifest.taskTypes);
    std::set<std::string> usedTypes;
    for (const auto &task : graphConfig.tasks) {
        ValidateGraphTaskTypeAllowed(manifest, task, allowedTypes);
        usedTypes.insert(task.type);
    }
    for (const auto &taskType : manifest.taskTypes) {
        ValidateManifestTaskTypeUsed(manifest, taskType, usedTypes);
    }
}

} // namespace smartdrone::core::application
