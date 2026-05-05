#include "core/application/session/native_runtime_graph_registry.h"

#include <stdexcept>
#include <string>
#include <vector>

namespace smartdrone::core::application {
namespace {

constexpr const char *kNativeRuntimeTopologyPath = "config/runtime_graph/native_runtime_topology.md";
constexpr const char *kSlamSessionSubgraphName = "slam_session_graph";
constexpr const char *kCalibSessionSubgraphName = "calib_session_graph";

const char *SubgraphName(NativeRuntimeGraphDomain domain)
{
    switch (domain) {
    case NativeRuntimeGraphDomain::SlamSession:
        return kSlamSessionSubgraphName;
    case NativeRuntimeGraphDomain::CalibSession:
        return kCalibSessionSubgraphName;
    }
    throw std::runtime_error("unsupported native runtime graph domain");
}

std::vector<std::string> TaskTypes(NativeRuntimeGraphDomain domain)
{
    switch (domain) {
    case NativeRuntimeGraphDomain::SlamSession:
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
    case NativeRuntimeGraphDomain::CalibSession:
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
    throw std::runtime_error("unsupported native runtime graph domain");
}

} // namespace

void RegisterNativeRuntimeGraphTypes(smartdrone::runtime_graph::Registry &registry,
                                     NativeRuntimeGraphDomain domain,
                                     const NativeRuntimeGraphTaskFactoryResolver &resolver)
{
    auto &catalog = smartdrone::runtime_graph::RuntimeGraphTypeCatalog::Global();
    catalog.RegisterReflectedMessageTypes(registry);
    catalog.RegisterReflectedTaskTypes(registry, TaskTypes(domain), resolver);
}

smartdrone::runtime_graph::RuntimeGraphConfig CompileNativeRuntimeGraphConfig(
    NativeRuntimeGraphDomain domain,
    const smartdrone::runtime_graph::Registry &registry)
{
    return smartdrone::runtime_graph::ParseRuntimeGraphConfigMermaidSubgraphFile(
        kNativeRuntimeTopologyPath, SubgraphName(domain), registry);
}

} // namespace smartdrone::core::application
