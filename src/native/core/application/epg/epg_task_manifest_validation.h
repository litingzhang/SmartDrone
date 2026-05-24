#pragma once

#include "common/epg/epg_types.h"
#include "core/application/epg/epg_task_manifest.h"

#include <set>
#include <string>

namespace SmartDrone::Core::Application {

void ValidateEpgManifestMetadata(const EpgTaskManifest &manifest);

std::set<std::string> ValidateEpgManifestCatalog(
    const EpgTaskManifest &manifest);

void ValidateEpgManifestAliases(
    const EpgTaskManifest &manifest,
    const std::set<std::string> &catalogTypes);

void ValidateEpgGraphTaskTypeAllowed(
    const EpgTaskManifest &manifest,
    const Epg::TaskConfig &task,
    const std::set<std::string> &allowedTypes);

void ValidateEpgTaskSchedulingCatalogMatch(
    const EpgTaskManifest &manifest,
    const Epg::TaskConfig &task);

void ValidateEpgManifestTaskTypeUsed(
    const EpgTaskManifest &manifest,
    const std::string &taskType,
    const std::set<std::string> &usedTypes);

void ValidateEpgOptimizedGraphManifest(
    const EpgTaskManifest &manifest,
    const Epg::OptimizedGraph &optimizedGraph);

} // namespace SmartDrone::Core::Application
