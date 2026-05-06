#include "core/application/session/native_epg_registry.h"

#include <stdexcept>
#include <string>
#include <vector>

namespace smartdrone::core::application {
namespace {

constexpr const char *kNativeEventPipelineTopologyPath = "config/epg/native_epg_topology.dot";
constexpr const char *kSlamSessionSubgraphName = "cluster_slam_session_graph";
constexpr const char *kCalibSessionSubgraphName = "cluster_calib_session_graph";

const char *SubgraphName(NativeEventPipelineGraphDomain domain)
{
    switch (domain) {
    case NativeEventPipelineGraphDomain::SlamSession:
        return kSlamSessionSubgraphName;
    case NativeEventPipelineGraphDomain::CalibSession:
        return kCalibSessionSubgraphName;
    }
    throw std::runtime_error("unsupported native event pipeline graph domain");
}

std::vector<std::string> TaskTypes(NativeEventPipelineGraphDomain domain)
{
    switch (domain) {
    case NativeEventPipelineGraphDomain::SlamSession:
        return {
            "NativeSlamResourceTask",
            "NativeSlamClockTask",
            "NativeSlamImuGateTask",
            "NativeSlamAcquireTask",
            "NativeSlamTrackingTask",
            "NativeSlamPosePostprocessTask",
            "NativeSlamPointCloudTask",
            "NativeSlamLivePoseTask",
            "NativeSlamMavlinkTask",
            "NativeSlamUdpTask",
            "NativeSlamDfxTask",
            "NativeSlamMonitorTask",
        };
    case NativeEventPipelineGraphDomain::CalibSession:
        return {
            "CalibResourceTask",
            "NativeCalibClockTask",
            "CalibCameraAcquireTask",
            "CalibPacingFilterTask",
            "CalibStorageWriteTask",
            "CalibImuWriterTask",
            "CalibUdpPreviewTask",
            "CalibCompletionTask",
            "CalibFlushSyncTask",
            "NativeCalibMonitorTask",
        };
    }
    throw std::runtime_error("unsupported native event pipeline graph domain");
}

} // namespace

void RegisterNativeEventPipelineGraphTypes(epg::Registry &registry,
                                     NativeEventPipelineGraphDomain domain,
                                     const NativeEventPipelineGraphTaskFactoryResolver &resolver)
{
    auto &catalog = epg::TypeCatalog::Global();
    catalog.RegisterReflectedMessageTypes(registry);
    catalog.RegisterReflectedTaskTypes(registry, TaskTypes(domain), resolver);
}

epg::GraphConfig CompileNativeEventPipelineGraphConfig(
    NativeEventPipelineGraphDomain domain,
    epg::Registry &registry)
{
    return epg::ParseGraphConfigDotFile(
        kNativeEventPipelineTopologyPath, SubgraphName(domain), registry);
}

} // namespace smartdrone::core::application
