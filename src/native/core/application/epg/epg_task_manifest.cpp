#include "core/application/epg/epg_task_manifest.h"

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

const EpgTaskManifest kSystemRuntimeManifest{
    EpgDomain::SystemRuntime,
    kSystemRuntimeGraphName,
    "/tmp/smartdrone_epg_system.json",
    {
        kVehicleTelemetryRxTaskType,
        "SetpointStreamTask",
        "UdpCommandTask",
        "ManualControlTask",
        "ForceRestartTask",
        "RuntimeSupervisorTask",
        "DiscoveryBeaconTask",
        "EpgDfxSnapshotTask",
    },
    {
        {kLegacyMavlinkRxTaskType, kVehicleTelemetryRxTaskType},
    },
};

const EpgTaskManifest kSlamSessionManifest{
    EpgDomain::SlamSession,
    kSlamSessionGraphName,
    "/tmp/smartdrone_epg_slam.json",
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
};

const EpgTaskManifest kCalibSessionManifest{
    EpgDomain::CalibSession,
    kCalibSessionGraphName,
    "/tmp/smartdrone_epg_calib.json",
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
};

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
