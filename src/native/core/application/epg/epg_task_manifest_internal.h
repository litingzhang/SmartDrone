#pragma once

#include "core/application/epg/epg_task_manifest.h"

#include <stdexcept>
#include <string>

namespace SmartDrone::Core::Application::EpgTaskManifestInternal {

inline constexpr const char *GLOBAL_TOPOLOGY_OBJECTIVE =
    Epg::EXACT_SOLVER_OBJECTIVE;

inline std::string TaskGraphLabel(const EpgTaskManifest &manifest)
{
    return "EventPipelineGraph subgraph '" + manifest.subgraphName + "'";
}

inline const EpgTaskCatalogEntry &RequireCatalogEntry(
    const EpgTaskManifest &manifest,
    const std::string &taskType)
{
    for (const auto &entry : manifest.catalog) {
        if (entry.taskType == taskType) {
            return entry;
        }
    }
    throw std::runtime_error(TaskGraphLabel(manifest) +
                             " missing catalog task type: " + taskType);
}

} // namespace SmartDrone::Core::Application::EpgTaskManifestInternal
