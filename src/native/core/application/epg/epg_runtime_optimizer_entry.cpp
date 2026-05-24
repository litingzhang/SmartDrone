#include "core/application/epg/epg_runtime_optimizer.h"

#include <exception>

#include "core/application/epg/epg_runtime_optimizer_io.h"
#include "core/application/epg/epg_task_manifest.h"

namespace SmartDrone::Core::Application {
namespace {

constexpr std::uint64_t PROFILE_FRESHNESS_MS = 60000;

std::uint64_t ProfileAgeMs(std::uint64_t nowMs,
                           std::uint64_t profileTimestampMs)
{
    if (profileTimestampMs == 0 || nowMs < profileTimestampMs) {
        return PROFILE_FRESHNESS_MS + 1;
    }
    return nowMs - profileTimestampMs;
}

EpgRuntimeOptimizerResult ValidateAndOptimizeProfile(
    const EpgTaskManifest &manifest,
    Epg::GraphProfile &profile,
    std::uint64_t nowMs)
{
    ValidateEpgOptimizerProfileForManifest(manifest, profile);
    return WriteOptimizedConfigForProfile(manifest, profile, nowMs);
}

} // namespace

EpgRuntimeOptimizerResult
OptimizeEpgProfileForManifest(const EpgTaskManifest &manifest,
                              std::uint64_t nowMs)
{
    try {
        ValidateEpgTaskManifest(manifest);
    } catch (const std::exception &error) {
        return {false, false, error.what()};
    }
    const std::string profileText =
        ReadEpgOptimizerFile(manifest.artifactPaths.profilePath);
    if (profileText.empty()) {
        return {false, false, "profile missing"};
    }
    Epg::GraphProfile profile;
    try {
        profile = Epg::ParseGraphProfileJson(profileText);
    } catch (const std::exception &error) {
        return {false, false, error.what()};
    }
    const auto &metadata = profile.metadata;
    if (metadata.schema != Epg::GRAPH_PROFILE_SCHEMA) {
        return {false, false, "profile schema mismatch"};
    }
    if (metadata.graph != manifest.subgraphName) {
        return {false, false, "profile graph mismatch"};
    }
    if (metadata.topologyVersion != manifest.topologyVersion) {
        return {false, false, "profile topology version mismatch"};
    }
    if (ProfileAgeMs(nowMs, metadata.timestampMs) > PROFILE_FRESHNESS_MS) {
        return {false, false, "profile stale"};
    }
    try {
        return ValidateAndOptimizeProfile(manifest, profile, nowMs);
    } catch (const std::exception &error) {
        return {false, false, error.what()};
    }
}

} // namespace SmartDrone::Core::Application
