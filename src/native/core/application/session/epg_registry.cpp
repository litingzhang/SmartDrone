#include "core/application/session/epg_registry.h"

#include <stdexcept>
#include <string>
#include <vector>

namespace smartdrone::core::application {
namespace {

constexpr const char *kEpgTopologyPath = "config/epg/epg_topology.dot";
constexpr const char *kSlamSessionSubgraphName = "cluster_slam_session_graph";
constexpr const char *kCalibSessionSubgraphName = "cluster_calib_session_graph";

const char *SubgraphName(EpgDomain domain)
{
    switch (domain) {
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
    case EpgDomain::SlamSession:
        return {
            "SlamResourceTask",
            "SlamClockTask",
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
        };
    }
    throw std::runtime_error("unsupported EPG domain");
}

} // namespace

void RegisterEpgTypes(epg::Registry &registry,
                                     EpgDomain domain,
                                     const EpgTaskFactoryResolver &resolver)
{
    auto &catalog = epg::TypeCatalog::Global();
    catalog.RegisterReflectedMessageTypes(registry);
    catalog.RegisterReflectedTaskTypes(registry, TaskTypes(domain), resolver);
}

epg::GraphConfig CompileEpgConfig(
    EpgDomain domain,
    epg::Registry &registry)
{
    return epg::ParseGraphConfigDotFile(
        kEpgTopologyPath, SubgraphName(domain), registry);
}

} // namespace smartdrone::core::application
