#include "core/application/runtime/system_runtime_task_factory.h"

#include <memory>
#include <utility>
#include <vector>

#include "common/epg/epg.h"
#include "core/application/epg/epg_task_manifest.h"
#include "core/application/runtime/epg_dfx_snapshot.h"
#include "core/application/runtime/epg_optimize_task.h"
#include "core/application/runtime/system_runtime_tasks.h"

namespace smartdrone::core::application {
namespace {

using SystemRuntimeTaskFactoryEntries =
    std::vector<epg::TypeCatalog::TaskFactoryEntry>;

template <class TaskType, class Factory>
void AddFactory(SystemRuntimeTaskFactoryEntries &entries,
                const epg::TypeCatalog &catalog,
                Factory factory)
{
    entries.push_back(catalog.MakeTaskFactoryEntry<TaskType>(
        [factory = std::move(factory)]() {
            return std::unique_ptr<epg::ITask>(factory());
        }));
}

void AddSystemStepTaskFactories(SystemRuntimeTaskFactoryEntries &entries,
                                const epg::TypeCatalog &catalog,
                                SystemRuntimeTaskFactoryDeps deps)
{
    AddFactory<VehicleTelemetryRxTask>(entries, catalog, [deps]() {
        return new VehicleTelemetryRxTask(deps.stepServices);
    });
    AddFactory<SetpointStreamTask>(entries, catalog, [deps]() {
        return new SetpointStreamTask(deps.stepServices);
    });
    AddFactory<ManualControlTask>(entries, catalog, [deps]() {
        return new ManualControlTask(deps.stepServices);
    });
    AddFactory<ForceRestartTask>(entries, catalog, [deps]() {
        return new ForceRestartTask(deps.stepServices);
    });
    AddFactory<RuntimeSupervisorTask>(entries, catalog, [deps]() {
        return new RuntimeSupervisorTask(deps.stepServices);
    });
    AddFactory<EpgRedeployTask>(entries, catalog, [deps]() {
        return new EpgRedeployTask(deps.stepServices, deps.redeploy);
    });
}

void AddSystemRuntimeTaskFactories(SystemRuntimeTaskFactoryEntries &entries,
                                   const epg::TypeCatalog &catalog,
                                   SystemRuntimeTaskFactoryDeps deps,
                                   const EpgTaskManifest &manifest)
{
    AddFactory<UdpCommandTask>(entries, catalog, [deps]() {
        return new UdpCommandTask(deps.commandRuntime);
    });
    AddFactory<DiscoveryBeaconTask>(entries, catalog, [deps]() {
        return new DiscoveryBeaconTask(deps.discoveryRuntime);
    });
    AddFactory<EpgDfxSnapshotTask>(
        entries, catalog,
        [target = EpgDfxSnapshotTarget{deps.graphRef, manifest.subgraphName,
                                       manifest.topologyVersion,
                                       EpgTaskCatalogJson(manifest),
                                       manifest.artifactPaths.dfxSnapshotPath,
                                       manifest.artifactPaths.profilePath}]() {
            return new EpgDfxSnapshotTask(target);
        });
    AddFactory<EpgOptimizeTask>(entries, catalog, [deps]() {
        return new EpgOptimizeTask({
                                       EpgDomain::SystemRuntime,
                                       EpgDomain::SlamSession,
                                       EpgDomain::CalibSession,
                                   },
                                   deps.redeploy);
    });
}

} // namespace

EpgTaskFactoryResolver MakeSystemRuntimeTaskFactoryResolver(
    SystemRuntimeTaskFactoryDeps deps)
{
    auto &catalog = epg::TypeCatalog::Global();
    const EpgTaskManifest &manifest =
        EpgManifestForDomain(EpgDomain::SystemRuntime);
    SystemRuntimeTaskFactoryEntries entries;
    entries.reserve(manifest.catalog.size());
    AddSystemStepTaskFactories(entries, catalog, deps);
    AddSystemRuntimeTaskFactories(entries, catalog, deps, manifest);
    auto resolver =
        epg::TypeCatalog::MakeTaskFactoryResolver(std::move(entries));
    ValidateEpgTaskFactoryManifest(manifest, resolver);
    return resolver;
}

} // namespace smartdrone::core::application
