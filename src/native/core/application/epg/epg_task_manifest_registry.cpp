#include "core/application/epg/epg_task_manifest.h"

#include <stdexcept>
#include <string>

namespace SmartDrone::Core::Application {
namespace {

constexpr const char *SYSTEM_RUNTIME_GRAPH_NAME =
    "cluster_system_runtime_graph";
constexpr const char *SLAM_SESSION_GRAPH_NAME = "cluster_slam_session_graph";
constexpr const char *CALIB_SESSION_GRAPH_NAME = "cluster_calib_session_graph";
constexpr const char *VEHICLE_TELEMETRY_RX_TASK_TYPE =
    "VehicleTelemetryRxTask";
constexpr const char *LEGACY_MAVLINK_RX_TASK_TYPE = "MavlinkRxTask";
constexpr const char *EPG_TOPOLOGY_PATH = "config/epg/epg_topology.dot";
constexpr const char *EPG_TOPOLOGY_REVISION = "v6";
constexpr const char *EPG_SNAPSHOT_DIR = "/tmp";
constexpr const char *EPG_OPTIMIZED_DIR = "output/epg";
const EpgTaskTopologySpec EPG_TOPOLOGY_SPEC{
    EPG_TOPOLOGY_PATH,
    EPG_TOPOLOGY_REVISION,
};

std::string BuildPath(const char *directory, const std::string &stem,
                      const char *suffix)
{
    return std::string(directory) + "/" + stem + suffix;
}

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
        {VEHICLE_TELEMETRY_RX_TASK_TYPE, "telemetry_rx", "mavlink", 1000, 2000, false},
        {"SetpointStreamTask", "flight_setpoint", "mavlink", 1000, 2000, false},
        {"UdpReceiveTask", "command_rx", "udp_command", 1000, 2000, false},
        {"UdpHeartbeatTxTask", "heartbeat_tx", "udp_command", 1000, 2000, false},
        {"UdpHeartbeatTimeoutTask", "heartbeat_guard", "flight_control", 1000, 2000, false},
        {"UdpStateTxTask", "state_tx", "udp_command", 1000, 3000, false},
        {"UdpPointCloudTxTask", "point_cloud_tx", "udp_command", 5000, 10000, false},
        {"ManualControlTask", "manual_control", "flight_control", 1000, 5000, false},
        {"ForceRestartTask", "restart_guard", "cpu", 1000, 5000, false},
        {"RuntimeSupervisorTask", "session_supervisor", "cpu", 1000, 10000, false},
        {"EpgRedeployTask", "epg_redeploy", "cpu", 1000, 5000, false},
        {"DiscoveryBeaconTask", "discovery", "udp_discovery", 1000, 10000, false},
        {"EpgDfxSnapshotTask", "dfx_snapshot", "storage", 2000, 10000, false},
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
        {"SlamResourceTask", "resource_open", "session_resource", 5000, 50000, false},
        {"SlamClockTask", "frame_clock", "cpu", 500, 2000, false},
        {"SlamImuPollTask", "imu_poll", "imu", 1000, 2000, false},
        {"SlamBackendTickTask", "backend_maintenance", "slam_backend", 2000, 5000, true, {"slam_backend_maintenance"}, true},
        {"SlamImuGateTask", "sensor_gate", "cpu", 1000, 2000, false},
        {"SlamAcquireTask", "frame_acquire", "camera", 12000, 16000, true, {}, true},
        {"SlamTrackingRouteTask", "tracking_route", "cpu", 500, 2000, false},
        {"SlamKltTrackingTask", "klt_tracking", "slam_backend", 24000, 33000, true, {}, true},
        {"SlamDpvoTrackingTask", "dpvo_tracking", "slam_backend", 24000, 33000, true, {}, true},
        {"SlamOrbTrackingTask", "orb_tracking", "slam_backend", 24000, 33000, true, {"slam_orb_tracking"}, true},
        {"SlamVisualFeatureTrackingTask", "visual_feature_tracking", "slam_backend", 24000, 33000, true, {}, true},
        {"SlamPosePostprocessTask", "pose_postprocess", "cpu", 3000, 5000, true, {}, true},
        {"SlamPointCloudTask", "point_cloud", "slam_backend", 5000, 10000, true, {}, true},
        {"SlamLivePoseTask", "live_pose", "cpu", 2000, 5000, false},
        {"SlamMavlinkTask", "pose_publish", "mavlink", 2000, 5000, false},
        {"SlamUdpTask", "preview_stream", "udp_stream", 10000, 16000, true},
        {"SlamPreviewTxTask", "preview_tx", "udp_stream", 10000, 16000, true},
        {"SlamDfxTask", "slam_dfx", "storage", 5000, 10000, false},
        {"SlamMonitorTask", "session_monitor", "cpu", 2000, 10000, false},
        {"EpgDfxSnapshotTask", "dfx_snapshot", "storage", 2000, 10000, false},
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
        {"CalibResourceTask", "resource_open", "session_resource", 5000, 50000, false},
        {"CalibClockTask", "capture_clock", "cpu", 500, 2000, false},
        {"CalibCameraAcquireTask", "frame_acquire", "camera", 12000, 20000, true},
        {"CalibPacingFilterTask", "save_pacing", "cpu", 1000, 5000, false},
        {"CalibStorageWriteTask", "storage_write", "storage", 20000, 50000, true},
        {"CalibImuWriterTask", "imu_write", "imu", 2000, 5000, false},
        {"CalibUdpPreviewTask", "preview_stream", "udp_stream", 10000, 16000, true},
        {"CalibCompletionTask", "completion", "cpu", 1000, 5000, false},
        {"CalibFlushSyncTask", "flush_sync", "session_resource", 50000, 200000, false},
        {"CalibMonitorTask", "session_monitor", "cpu", 1000, 5000, false},
        {"EpgDfxSnapshotTask", "dfx_snapshot", "storage", 2000, 10000, false},
    },
};

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

} // namespace SmartDrone::Core::Application
