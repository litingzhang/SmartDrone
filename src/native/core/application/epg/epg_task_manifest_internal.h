#pragma once

#include "core/application/epg/epg_task_manifest.h"

#include <string>

namespace SmartDrone::Core::Application::EpgTaskManifestInternal {

inline constexpr const char *GLOBAL_TOPOLOGY_OBJECTIVE =
    Epg::EXACT_SOLVER_OBJECTIVE;

std::string TaskGraphLabel(const EpgTaskManifest &manifest);

const EpgTaskCatalogEntry &RequireCatalogEntry(
    const EpgTaskManifest &manifest, const std::string &taskType);
const EpgTaskCatalogEntry *FindCatalogEntry(
    const EpgTaskManifest &manifest, const std::string &taskType);

} // namespace SmartDrone::Core::Application::EpgTaskManifestInternal
