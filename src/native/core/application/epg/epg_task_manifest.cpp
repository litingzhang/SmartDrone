#include "core/application/epg/epg_task_manifest.h"
#include "core/application/epg/epg_task_manifest_internal.h"
#include "core/application/epg/epg_task_manifest_runtime_tuning.h"
#include "core/application/epg/epg_task_manifest_validation.h"

#include <set>
#include <stdexcept>

namespace SmartDrone::Core::Application {
namespace {
using EpgTaskManifestInternal::FindCatalogEntry;

void ValidateTaskFactory(const EpgTaskFactoryResolver &resolver,
                         const std::string &taskType)
{
    if (resolver(taskType)) {
        return;
    }
    throw std::runtime_error("missing EventPipelineGraph task factory: " +
                             taskType);
}

void ApplyCatalogDefaultsToTask(const EpgTaskCatalogEntry &entry,
                                Epg::TaskConfig &task)
{
    task.scheduling.resource = entry.resource;
    task.scheduling.budgetUs = entry.budgetUs;
    task.scheduling.deadlineUs = entry.deadlineUs;
}

} // namespace

void ApplyEpgTaskCatalogDefaults(
    const EpgTaskManifest &manifest,
    Epg::GraphConfig &graphConfig)
{
    ValidateEpgManifestMetadata(manifest);
    ValidateEpgManifestCatalog(manifest);
    ValidateEpgManifestRuntimeTuning(manifest);
    for (auto &task : graphConfig.tasks) {
        const auto *entry = FindCatalogEntry(manifest, task.type);
        if (!entry) {
            continue;
        }
        ApplyCatalogDefaultsToTask(*entry, task);
    }
}

void ValidateEpgTaskManifest(
    const EpgTaskManifest &manifest)
{
    ValidateEpgManifestMetadata(manifest);
    const auto catalogTypes = ValidateEpgManifestCatalog(manifest);
    ValidateEpgManifestAliases(manifest, catalogTypes);
    ValidateEpgManifestRuntimeTuning(manifest);
}

void ValidateEpgTaskFactoryManifest(
    const EpgTaskManifest &manifest,
    const EpgTaskFactoryResolver &resolver)
{
    ValidateEpgManifestMetadata(manifest);
    if (!resolver) {
        throw std::runtime_error(
            "EventPipelineGraph task factory resolver must be callable");
    }
    const auto catalogTypes = ValidateEpgManifestCatalog(manifest);
    for (const auto &entry : manifest.catalog) {
        ValidateTaskFactory(resolver, entry.taskType);
    }
    ValidateEpgManifestAliases(manifest, catalogTypes);
    ValidateEpgManifestRuntimeTuning(manifest);
    for (const auto &alias : manifest.aliases) {
        ValidateTaskFactory(resolver, alias.targetType);
    }
}

void ValidateEpgTaskGraphManifest(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig &graphConfig)
{
    ValidateEpgManifestMetadata(manifest);
    const auto allowedTypes = ValidateEpgManifestCatalog(manifest);
    ValidateEpgManifestRuntimeTuning(manifest);
    std::set<std::string> usedTypes;
    for (const auto &task : graphConfig.tasks) {
        ValidateEpgGraphTaskTypeAllowed(manifest, task, allowedTypes);
        ValidateEpgTaskSchedulingCatalogMatch(manifest, task);
        usedTypes.insert(task.type);
    }
    for (const auto &entry : manifest.catalog) {
        ValidateEpgManifestTaskTypeUsed(manifest, entry.taskType, usedTypes);
    }
    ValidateEpgGraphRuntimeTuningDeclared(manifest, graphConfig);
}

} // namespace SmartDrone::Core::Application
