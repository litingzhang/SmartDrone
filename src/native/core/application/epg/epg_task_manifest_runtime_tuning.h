#pragma once

#include "common/epg/epg_types.h"
#include "core/application/epg/epg_task_manifest.h"

namespace SmartDrone::Core::Application {

void ValidateEpgManifestRuntimeTuning(const EpgTaskManifest &manifest);

void ValidateEpgGraphRuntimeTuningDeclared(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig &graphConfig);

void ValidateEpgTaskRuntimeTuning(
    const EpgTaskManifest &manifest,
    const Epg::GraphConfig &graphConfig,
    const std::vector<EpgTaskRuntimeTuningEntry> &requestedTuning);

} // namespace SmartDrone::Core::Application
