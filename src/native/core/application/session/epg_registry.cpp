#include "core/application/session/epg_registry.h"

#include <stdexcept>
#include <string>
#include <vector>

namespace smartdrone::core::application {
namespace {

constexpr const char *kEpgTopologyPath = "config/epg/epg_topology.dot";
constexpr const char *kSlamSessionSubgraphName = "cluster_slam_session_graph";
constexpr const char *kCalibSessionSubgraphName = "cluster_calib_session_graph";
constexpr const char *kVehicleTelemetryRxTaskType = "VehicleTelemetryRxTask";
constexpr const char *kLegacyMavlinkRxTaskType = "MavlinkRxTask";

const char *SubgraphName(EpgDomain domain)
{
    switch (domain) {
    case EpgDomain::SystemRuntime:
        return "cluster_system_runtime_graph";
    case EpgDomain::SlamSession:
        return kSlamSessionSubgraphName;
    case EpgDomain::CalibSession:
        return kCalibSessionSubgraphName;
    }
    throw std::runtime_error("unsupported EPG domain");
}

std::vector<std::string> TaskTypes(EpgDomain domain)
{
    switch (domain) {
    case EpgDomain::SystemRuntime:
        return {
            kVehicleTelemetryRxTaskType,
            "SetpointStreamTask",
            "UdpCommandTask",
            "ManualControlTask",
            "ForceRestartTask",
            "RuntimeSupervisorTask",
            "DiscoveryBeaconTask",
            "EpgDfxSnapshotTask",
        };
    case EpgDomain::SlamSession:
        return {
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
        };
    case EpgDomain::CalibSession:
        return {
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
        };
    }
    throw std::runtime_error("unsupported EPG domain");
}

void RegisterSystemRuntimeLegacyTaskTypes(epg::Registry &registry,
                                          const EpgTaskFactoryResolver &resolver)
{
    auto factory = resolver(kVehicleTelemetryRxTaskType);
    if (!factory) {
        throw std::runtime_error("missing EventPipelineGraph task factory: " +
                                 std::string(kVehicleTelemetryRxTaskType));
    }
    registry.RegisterTaskFactory(kLegacyMavlinkRxTaskType, {}, {}, factory);
}

} // namespace

void RegisterEpgTypes(epg::Registry &registry,
                                     EpgDomain domain,
                                     const EpgTaskFactoryResolver &resolver)
{
    auto &catalog = epg::TypeCatalog::Global();
    catalog.RegisterReflectedMessageTypes(registry);
    catalog.RegisterReflectedTaskTypes(registry, TaskTypes(domain), resolver);
    if (domain == EpgDomain::SystemRuntime) {
        RegisterSystemRuntimeLegacyTaskTypes(registry, resolver);
    }
}

epg::GraphConfig CompileEpgConfig(
    EpgDomain domain,
    epg::Registry &registry)
{
    return epg::ParseGraphConfigDotFile(
        kEpgTopologyPath, SubgraphName(domain), registry);
}

} // namespace smartdrone::core::application
