#include "core/application/epg/epg_runtime_optimizer.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

#include "common/epg/epg.h"
#include "core/application/runtime/epg_dfx_snapshot.h"

namespace smartdrone::core::application {
namespace {

constexpr std::uint64_t PROFILE_FRESHNESS_MS = 60000;
constexpr const char *OPTIMIZED_SCHEMA =
    "smartdrone.epg.optimized_config.v1";
constexpr const char *SOLVER_VERSION = "native-heuristic-v1";

std::string ReadFile(const std::string &path)
{
    std::ifstream input(path);
    if (!input) {
        return {};
    }
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
}

std::uint64_t ExtractUInt64(const std::string &text,
                            const std::string &field)
{
    const std::string key = "\"" + field + "\"";
    const auto keyPos = text.find(key);
    if (keyPos == std::string::npos) {
        return 0;
    }
    const auto colon = text.find(':', keyPos + key.size());
    if (colon == std::string::npos) {
        return 0;
    }
    const auto begin = text.find_first_of("0123456789", colon + 1);
    if (begin == std::string::npos) {
        return 0;
    }
    const auto end = text.find_first_not_of("0123456789", begin);
    return std::stoull(text.substr(begin, end - begin));
}

bool ContainsStringField(const std::string &text,
                         const std::string &field,
                         const std::string &value)
{
    const std::string needle =
        "\"" + field + "\": \"" + value + "\"";
    return text.find(needle) != std::string::npos;
}

std::uint64_t ProfileAgeMs(std::uint64_t nowMs,
                           std::uint64_t profileTimestampMs)
{
    if (profileTimestampMs == 0 || nowMs < profileTimestampMs) {
        return PROFILE_FRESHNESS_MS + 1;
    }
    return nowMs - profileTimestampMs;
}

epg::GraphConfig LoadProfileTopology(const std::string &profileText)
{
    const auto topologyKey = profileText.find("\"topology\"");
    if (topologyKey == std::string::npos) {
        throw std::runtime_error("profile missing topology");
    }
    const auto open = profileText.find('{', topologyKey);
    const auto diagnostics = profileText.find("\"diagnostics\"", open);
    if (open == std::string::npos || diagnostics == std::string::npos) {
        throw std::runtime_error("profile topology is incomplete");
    }
    const auto close = profileText.rfind('}', diagnostics);
    if (close == std::string::npos || close <= open) {
        throw std::runtime_error("profile topology object is invalid");
    }
    return epg::ParseGraphConfigJson(
        profileText.substr(open, close - open + 1));
}

std::map<std::string, std::uint64_t> MakeOptimizerNumbers(
    const std::string &profileText)
{
    return {
        {"sourceTimestampMs", ExtractUInt64(profileText, "timestampMs")},
    };
}

std::map<std::string, std::string> MakeOptimizerStrings(
    const EpgTaskManifest &manifest)
{
    return {
        {"schema", OPTIMIZED_SCHEMA},
        {"targetGraph", manifest.subgraphName},
        {"topologyVersion", manifest.topologyVersion},
        {"solverVersion", SOLVER_VERSION},
    };
}

EpgRuntimeOptimizerResult WriteOptimizedConfig(
    const EpgTaskManifest &manifest,
    const std::string &profileText)
{
    const auto config = LoadProfileTopology(profileText);
    const std::string json = epg::GraphConfigToJson(
        config, MakeOptimizerStrings(manifest),
        MakeOptimizerNumbers(profileText));
    WriteEpgDfxSnapshotFile(manifest.optimizedConfigPath, json);
    return {true, "optimized config refreshed"};
}

} // namespace

EpgRuntimeOptimizerResult OptimizeEpgProfileForManifest(
    const EpgTaskManifest &manifest,
    std::uint64_t nowMs)
{
    const std::string profileText = ReadFile(manifest.profilePath);
    if (profileText.empty()) {
        return {false, "profile missing"};
    }
    if (!ContainsStringField(profileText, "schema",
                             "smartdrone.epg.profile.v1")) {
        return {false, "profile schema mismatch"};
    }
    if (!ContainsStringField(profileText, "graph", manifest.subgraphName)) {
        return {false, "profile graph mismatch"};
    }
    if (!ContainsStringField(profileText, "topologyVersion",
                             manifest.topologyVersion)) {
        return {false, "profile topology version mismatch"};
    }
    const auto timestampMs = ExtractUInt64(profileText, "timestampMs");
    if (ProfileAgeMs(nowMs, timestampMs) > PROFILE_FRESHNESS_MS) {
        return {false, "profile stale"};
    }
    try {
        return WriteOptimizedConfig(manifest, profileText);
    } catch (const std::exception &error) {
        return {false, error.what()};
    }
}

} // namespace smartdrone::core::application
