#pragma once

#include <string>

#include "core/application/epg/epg_task_manifest.h"

namespace smartdrone::core::application {

struct EpgRuntimeOptimizerResult {
    bool optimized{false};
    std::string message;
};

EpgRuntimeOptimizerResult OptimizeEpgProfileForManifest(
    const EpgTaskManifest &manifest,
    std::uint64_t nowMs);

} // namespace smartdrone::core::application
