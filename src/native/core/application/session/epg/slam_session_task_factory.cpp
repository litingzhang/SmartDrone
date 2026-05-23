#include "core/application/session/epg/slam_session_task_factory.h"

#include <memory>
#include <utility>
#include <vector>

#include "common/epg/epg.h"
#include "core/application/epg/epg_task_manifest.h"
#include "core/application/runtime/epg_dfx_snapshot.h"
#include "core/application/session/slam/slam_session_runtime_service.h"
#include "core/application/session/epg/slam_session_tasks.h"

namespace SmartDrone::Core::Application {
namespace {

using EpgTaskFactoryEntries = std::vector<Epg::TypeCatalog::TaskFactoryEntry>;

template <class TaskType, class Factory>
void AddFactory(EpgTaskFactoryEntries &entries, const Epg::TypeCatalog &catalog,
                Factory factory)
{
    entries.push_back(catalog.MakeTaskFactoryEntry<TaskType>(
        [factory = std::move(factory)]() {
            return std::unique_ptr<Epg::ITask>(factory());
        }));
}

void AddSlamStartupTaskFactories(EpgTaskFactoryEntries &entries,
                                 const Epg::TypeCatalog &catalog,
                                 SlamTaskFactoryDeps deps)
{
    AddFactory<SlamResourceTask>(entries, catalog, [deps]() {
        return new SlamResourceTask(deps.service, deps.stop,
                                    deps.runningFlag);
    });
    AddFactory<SlamClockTask>(entries, catalog, [deps]() {
        return new SlamClockTask(deps.stop, deps.runningFlag);
    });
}

void AddSlamProcessingTaskFactories(EpgTaskFactoryEntries &entries,
                                    const Epg::TypeCatalog &catalog,
                                    SlamTaskFactoryDeps deps)
{
    AddFactory<SlamImuPollTask>(entries, catalog, [deps]() {
        return new SlamImuPollTask(deps.service, deps.stop,
                                   deps.runningFlag);
    });
    AddFactory<SlamBackendTickTask>(entries, catalog, [deps]() {
        return new SlamBackendTickTask(deps.service, deps.stop,
                                       deps.runningFlag);
    });
    AddFactory<SlamImuGateTask>(entries, catalog, [deps]() {
        return new SlamImuGateTask(deps.service, deps.stop,
                                   deps.runningFlag);
    });
    AddFactory<SlamAcquireTask>(entries, catalog, [deps]() {
        return new SlamAcquireTask(deps.service, deps.stop,
                                   deps.runningFlag);
    });
    AddFactory<SlamTrackingRouteTask>(entries, catalog, [deps]() {
        return new SlamTrackingRouteTask(deps.stop, deps.runningFlag);
    });
    AddFactory<SlamKltTrackingTask>(entries, catalog, [deps]() {
        return new SlamKltTrackingTask(deps.service, deps.stop,
                                       deps.runningFlag);
    });
    AddFactory<SlamDpvoTrackingTask>(entries, catalog, [deps]() {
        return new SlamDpvoTrackingTask(deps.service, deps.stop,
                                        deps.runningFlag);
    });
    AddFactory<SlamOrbTrackingTask>(entries, catalog, [deps]() {
        return new SlamOrbTrackingTask(deps.service, deps.stop,
                                       deps.runningFlag);
    });
    AddFactory<SlamVisualFeatureTrackingTask>(entries, catalog, [deps]() {
        return new SlamVisualFeatureTrackingTask(deps.service, deps.stop,
                                                 deps.runningFlag);
    });
}

void AddSlamOutputTaskFactories(EpgTaskFactoryEntries &entries,
                                const Epg::TypeCatalog &catalog,
                                SlamTaskFactoryDeps deps)
{
    AddFactory<SlamPosePostprocessTask>(entries, catalog, [deps]() {
        return new SlamPosePostprocessTask(deps.service, deps.stop,
                                           deps.runningFlag);
    });
    AddFactory<SlamPointCloudTask>(entries, catalog, [deps]() {
        return new SlamPointCloudTask(deps.service, deps.stop,
                                      deps.runningFlag);
    });
    AddFactory<SlamLivePoseTask>(entries, catalog, [deps]() {
        return new SlamLivePoseTask(deps.service, deps.stop,
                                    deps.runningFlag);
    });
    AddFactory<SlamMavlinkTask>(entries, catalog, [deps]() {
        return new SlamMavlinkTask(deps.service, deps.stop,
                                   deps.runningFlag);
    });
    AddFactory<SlamUdpTask>(entries, catalog, [deps]() {
        return new SlamUdpTask(deps.service, deps.stop, deps.runningFlag);
    });
    AddFactory<SlamPreviewTxTask>(entries, catalog, [deps]() {
        return new SlamPreviewTxTask(deps.service, deps.stop,
                                     deps.runningFlag);
    });
    AddFactory<SlamDfxTask>(entries, catalog, [deps]() {
        return new SlamDfxTask(deps.service, deps.stop, deps.runningFlag);
    });
}

void AddSlamMonitorTaskFactories(EpgTaskFactoryEntries &entries,
                                 const Epg::TypeCatalog &catalog,
                                 SlamTaskFactoryDeps deps,
                                 const EpgTaskManifest &manifest)
{
    AddFactory<SlamMonitorTask>(entries, catalog, [deps]() {
        return new SlamMonitorTask(deps.stop, deps.sessionOk);
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
}

} // namespace

EpgTaskFactoryResolver MakeSlamGraphTaskFactoryResolver(
    SlamTaskFactoryDeps deps)
{
    auto &catalog = Epg::TypeCatalog::Global();
    const EpgTaskManifest &manifest =
        EpgManifestForDomain(EpgDomain::SlamSession);
    EpgTaskFactoryEntries entries;
    entries.reserve(manifest.catalog.size());
    AddSlamStartupTaskFactories(entries, catalog, deps);
    AddSlamProcessingTaskFactories(entries, catalog, deps);
    AddSlamOutputTaskFactories(entries, catalog, deps);
    AddSlamMonitorTaskFactories(entries, catalog, deps, manifest);
    auto resolver =
        Epg::TypeCatalog::MakeTaskFactoryResolver(std::move(entries));
    ValidateEpgTaskFactoryManifest(manifest, resolver);
    return resolver;
}

} // namespace SmartDrone::Core::Application
