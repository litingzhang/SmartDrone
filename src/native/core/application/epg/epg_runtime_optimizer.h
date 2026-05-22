#pragma once

#include <cstdint>
#include <string>

#include "core/application/epg/epg_task_manifest.h"

namespace SmartDrone::core::application {

struct EpgRuntimeOptimizerResult {
    bool optimized{false};
    bool configChanged{false};
    std::string message;
    std::string targetGraph;
    std::string topologyVersion;
    std::string sourceProfile;
    std::string sourceProfilePath;
    std::uint64_t sourceTimestampMs{0};
    std::uint64_t generatedAtMs{0};
    std::string solverVersion;
    std::string optimizedConfigPath;
    std::string solverReportPath;
};

EpgRuntimeOptimizerResult OptimizeEpgProfileForManifest(
    const EpgTaskManifest &manifest,
    std::uint64_t nowMs);

} // namespace SmartDrone::core::application
