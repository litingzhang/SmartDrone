#include "core/application/epg/epg_task_manifest_validation.h"

#include "core/application/epg/epg_task_manifest_internal.h"

#include <algorithm>
#include <stdexcept>

namespace SmartDrone::Core::Application {
namespace {

using EpgTaskManifestInternal::RequireCatalogEntry;
using EpgTaskManifestInternal::TaskGraphLabel;

bool ArtifactPathsComplete(const EpgTaskArtifactPaths &paths)
{
    return !paths.dfxSnapshotPath.empty() &&
           !paths.profilePath.empty() &&
           !paths.optimizedConfigPath.empty() &&
           !paths.solverReportPath.empty();
}

bool StringVectorContains(const std::vector<std::string> &values,
                          const std::string &value)
{
    return std::find(values.begin(), values.end(), value) != values.end();
}

void ValidateCatalogTaskTypeUnique(const EpgTaskManifest &manifest,
                                   std::set<std::string> &catalogTypes,
                                   const std::string &taskType)
{
    if (catalogTypes.insert(taskType).second) {
        return;
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " duplicates catalog task type: " +
                             taskType);
}

void ValidateCatalogEntrySemantics(const EpgTaskManifest &manifest,
                                   const EpgTaskCatalogEntry &entry)
{
    if (entry.taskType.empty()) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " catalog task type is empty");
    }
    if (entry.role.empty() || entry.resource.empty()) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " catalog metadata is incomplete: " +
                                 entry.taskType);
    }
    if (entry.budgetUs == 0 || entry.deadlineUs == 0 ||
        entry.deadlineUs < entry.budgetUs) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " catalog timing is invalid: " +
                                 entry.taskType);
    }
    for (const auto &resource : entry.resourceAlternates) {
        if (!resource.empty()) {
            continue;
        }
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " catalog alternate resource is empty: " +
                                 entry.taskType);
    }
}

void ValidateAliasTargetDeclared(const EpgTaskManifest &manifest,
                                 const EpgTaskAliasManifestEntry &alias,
                                 const std::set<std::string> &catalogTypes)
{
    if (catalogTypes.find(alias.targetType) != catalogTypes.end()) {
        return;
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " alias target outside catalog: " +
                             alias.alias + " -> " + alias.targetType);
}

void ValidateAliasNameAvailable(const EpgTaskManifest &manifest,
                                std::set<std::string> &aliasTypes,
                                const EpgTaskAliasManifestEntry &alias,
                                const std::set<std::string> &catalogTypes)
{
    if (alias.alias.empty() || alias.targetType.empty()) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " alias metadata is incomplete");
    }
    if (catalogTypes.find(alias.alias) != catalogTypes.end()) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " alias shadows catalog task type: " +
                                 alias.alias);
    }
    if (aliasTypes.insert(alias.alias).second) {
        return;
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " duplicates task alias: " + alias.alias);
}

} // namespace

void ValidateEpgManifestMetadata(const EpgTaskManifest &manifest)
{
    if (manifest.subgraphName.empty()) {
        throw std::runtime_error("EventPipelineGraph manifest subgraph is empty");
    }
    if (manifest.topologyPath.empty() || manifest.topologyVersion.empty()) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " topology metadata is incomplete");
    }
    if (!ArtifactPathsComplete(manifest.artifactPaths)) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " artifact paths are incomplete");
    }
}

std::set<std::string> ValidateEpgManifestCatalog(
    const EpgTaskManifest &manifest)
{
    if (manifest.catalog.empty()) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " catalog is empty");
    }
    std::set<std::string> catalogTypes;
    for (const auto &entry : manifest.catalog) {
        ValidateCatalogEntrySemantics(manifest, entry);
        ValidateCatalogTaskTypeUnique(manifest, catalogTypes, entry.taskType);
    }
    return catalogTypes;
}

void ValidateEpgManifestAliases(
    const EpgTaskManifest &manifest,
    const std::set<std::string> &catalogTypes)
{
    std::set<std::string> aliasTypes;
    for (const auto &alias : manifest.aliases) {
        ValidateAliasNameAvailable(manifest, aliasTypes, alias, catalogTypes);
        ValidateAliasTargetDeclared(manifest, alias, catalogTypes);
    }
}

void ValidateEpgGraphTaskTypeAllowed(
    const EpgTaskManifest &manifest,
    const Epg::TaskConfig &task,
    const std::set<std::string> &allowedTypes)
{
    if (allowedTypes.find(task.type) != allowedTypes.end()) {
        return;
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " uses task type outside manifest: " +
                             task.name + " type=" + task.type);
}

void ValidateEpgTaskSchedulingCatalogMatch(
    const EpgTaskManifest &manifest,
    const Epg::TaskConfig &task)
{
    const auto &entry = RequireCatalogEntry(manifest, task.type);
    const auto &scheduling = task.scheduling;
    const bool resourceAllowed =
        scheduling.resource == entry.resource ||
        StringVectorContains(entry.resourceAlternates, scheduling.resource);
    if (!resourceAllowed || scheduling.budgetUs != entry.budgetUs ||
        scheduling.deadlineUs != entry.deadlineUs) {
        throw std::runtime_error(TaskGraphLabel(manifest) +
                                 " task scheduling catalog mismatch: " +
                                 task.name + " type=" + task.type);
    }
}

void ValidateEpgManifestTaskTypeUsed(
    const EpgTaskManifest &manifest,
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

} // namespace SmartDrone::Core::Application
