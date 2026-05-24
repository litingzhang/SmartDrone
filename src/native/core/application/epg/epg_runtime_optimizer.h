#pragma once

#include <cstdint>
#include <string>

#include "common/epg/epg.h"

namespace SmartDrone::Core::Application {

struct EpgTaskManifest;

inline constexpr const char *EPG_EXACT_SOLVER_OBJECTIVE =
    Epg::EXACT_SOLVER_OBJECTIVE;

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

EpgRuntimeOptimizerResult WriteOptimizedConfigForProfile(
    const EpgTaskManifest &manifest,
    const Epg::GraphProfile &profile,
    std::uint64_t nowMs);

void ValidateEpgOptimizerProfileForManifest(
    const EpgTaskManifest &manifest,
    Epg::GraphProfile &profile);

EpgRuntimeOptimizerResult OptimizeEpgProfileForManifest(
    const EpgTaskManifest &manifest,
    std::uint64_t nowMs);

} // namespace SmartDrone::Core::Application
