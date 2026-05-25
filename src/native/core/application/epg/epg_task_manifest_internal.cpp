#include "core/application/epg/epg_task_manifest_internal.h"

#include <stdexcept>

namespace SmartDrone::Core::Application::EpgTaskManifestInternal {

std::string TaskGraphLabel(const EpgTaskManifest &manifest)
{
    return "EventPipelineGraph subgraph '" + manifest.subgraphName + "'";
}

const EpgTaskCatalogEntry &RequireCatalogEntry(
    const EpgTaskManifest &manifest, const std::string &taskType)
{
    for (const auto &entry : manifest.catalog) {
        if (entry.taskType == taskType) {
            return entry;
        }
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " missing catalog task type: " + taskType);
}

const EpgTaskCatalogEntry *FindCatalogEntry(
    const EpgTaskManifest &manifest, const std::string &taskType)
{
    for (const auto &entry : manifest.catalog) {
        if (entry.taskType == taskType) {
            return &entry;
        }
    }
    return nullptr;
}

} // namespace SmartDrone::Core::Application::EpgTaskManifestInternal
