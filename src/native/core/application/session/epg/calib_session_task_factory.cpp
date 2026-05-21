#include "core/application/session/epg/calib_session_task_factory.h"

#include <memory>
#include <utility>
#include <vector>

#include "common/epg/epg.h"
#include "core/application/epg/epg_task_manifest.h"
#include "core/application/runtime/epg_dfx_snapshot.h"
#include "core/application/session/epg/calib_session_tasks.h"

namespace smartdrone::core::application {
namespace {

using CalibTaskFactoryEntries =
    std::vector<epg::TypeCatalog::TaskFactoryEntry>;

template <class TaskType, class Factory>
void AddFactory(CalibTaskFactoryEntries &entries,
                const epg::TypeCatalog &catalog,
                Factory factory)
{
    entries.push_back(catalog.MakeTaskFactoryEntry<TaskType>(
        [factory = std::move(factory)]() {
            return std::unique_ptr<epg::ITask>(factory());
        }));
}

void AddCalibStartupTaskFactories(CalibTaskFactoryEntries &entries,
                                  const epg::TypeCatalog &catalog,
                                  CalibTaskFactoryDeps deps)
{
    AddFactory<CalibResourceTask>(entries, catalog, [deps]() {
        return new CalibResourceTask(deps.state, deps.stop,
                                     deps.runningFlag);
    });
    AddFactory<CalibClockTask>(entries, catalog, [deps]() {
        return new CalibClockTask(deps.stop, deps.runningFlag);
    });
}

void AddCalibProcessingTaskFactories(CalibTaskFactoryEntries &entries,
                                     const epg::TypeCatalog &catalog,
                                     CalibTaskFactoryDeps deps)
{
    AddFactory<CalibCameraAcquireTask>(entries, catalog, [deps]() {
        return new CalibCameraAcquireTask(deps.state, deps.stop,
                                          deps.runningFlag);
    });
    AddFactory<CalibPacingFilterTask>(entries, catalog, [deps]() {
        return new CalibPacingFilterTask(deps.state);
    });
    AddFactory<CalibStorageWriteTask>(entries, catalog, [deps]() {
        return new CalibStorageWriteTask(deps.state);
    });
    AddFactory<CalibImuWriterTask>(entries, catalog, [deps]() {
        return new CalibImuWriterTask(deps.state, deps.stop,
                                      deps.runningFlag);
    });
    AddFactory<CalibUdpPreviewTask>(entries, catalog, [deps]() {
        return new CalibUdpPreviewTask(deps.state);
    });
}

void AddCalibCompletionTaskFactories(CalibTaskFactoryEntries &entries,
                                     const epg::TypeCatalog &catalog,
                                     CalibTaskFactoryDeps deps,
                                     const EpgTaskManifest &manifest)
{
    AddFactory<CalibCompletionTask>(entries, catalog, [deps]() {
        return new CalibCompletionTask(deps.state);
    });
    AddFactory<CalibFlushSyncTask>(entries, catalog, [deps]() {
        return new CalibFlushSyncTask(deps.state, deps.completed,
                                      deps.sessionOk);
    });
    AddFactory<CalibMonitorTask>(entries, catalog, [deps]() {
        return new CalibMonitorTask(deps.sessionOk, deps.completed);
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

EpgTaskFactoryResolver MakeCalibGraphTaskFactoryResolver(
    CalibTaskFactoryDeps deps)
{
    auto &catalog = epg::TypeCatalog::Global();
    const EpgTaskManifest &manifest =
        EpgManifestForDomain(EpgDomain::CalibSession);
    CalibTaskFactoryEntries entries;
    entries.reserve(manifest.catalog.size());
    AddCalibStartupTaskFactories(entries, catalog, deps);
    AddCalibProcessingTaskFactories(entries, catalog, deps);
    AddCalibCompletionTaskFactories(entries, catalog, deps, manifest);
    auto resolver =
        epg::TypeCatalog::MakeTaskFactoryResolver(std::move(entries));
    ValidateEpgTaskFactoryManifest(manifest, resolver);
    return resolver;
}

} // namespace smartdrone::core::application
