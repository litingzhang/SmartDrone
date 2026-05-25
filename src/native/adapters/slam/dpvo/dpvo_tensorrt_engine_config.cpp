#include "adapters/slam/dpvo/dpvo_tensorrt_engine.h"

#include <algorithm>
#include <filesystem>
#include <system_error>
#include <vector>

#include "common/environment.h"

namespace SmartDrone::Adapters::Slam {
namespace {

bool DpvoRepoHasDefaultEngines(const std::filesystem::path &repo)
{
    if (repo.empty()) {
        return false;
    }
    std::error_code ec;
    return std::filesystem::exists(
               repo / "weights" / "dpvo_patchifier_fp16.engine", ec) &&
           std::filesystem::exists(repo / "weights" / "dpvo_update_fp16.engine",
                                   ec);
}

std::filesystem::path ResolveDpvoRepoPath(const std::string &configuredRepo)
{
    std::vector<std::filesystem::path> candidates;
    auto addCandidate = [&candidates](const std::filesystem::path &path) {
        if (!path.empty()) {
            candidates.emplace_back(path);
        }
    };

    addCandidate(configuredRepo);
    const std::string envRepo =
        SmartDrone::Common::EnvStringValue("SMART_DRONE_DPVO_REPO", "");
    if (!envRepo.empty()) {
        addCandidate(envRepo);
    }
    const std::string home = SmartDrone::Common::EnvStringValue("HOME", "");
    if (!home.empty()) {
        addCandidate(std::filesystem::path(home) / "DPVO");
    }
    addCandidate("/home/nvidia/DPVO");
    addCandidate("/home/ltz/DPVO");
    addCandidate(std::filesystem::current_path() / "DPVO");

    for (const std::filesystem::path &candidate : candidates) {
        const std::filesystem::path normalized = candidate.lexically_normal();
        if (DpvoRepoHasDefaultEngines(normalized)) {
            return normalized;
        }
    }
    if (!configuredRepo.empty()) {
        return std::filesystem::path(configuredRepo).lexically_normal();
    }
    if (!envRepo.empty()) {
        return std::filesystem::path(envRepo).lexically_normal();
    }
    return {};
}

} // namespace

DpvoTensorRtConfig MakeDpvoTensorRtConfig(
    const DpvoRuntimeConfig &runtime,
    const std::string &settingsPath)
{
    DpvoTensorRtConfig out{};
    out.repoPath = ResolveDpvoRepoPath(runtime.repoPath).string();
    out.patchEnginePath = runtime.patchEnginePath;
    out.updateEnginePath = runtime.updateEnginePath;
    out.settingsPath = settingsPath;
    out.inputWidth = std::clamp(runtime.inputWidth, 160, 1280);
    out.inputHeight = std::clamp(runtime.inputHeight, 120, 960);
    out.patchesPerFrame = std::clamp(runtime.patchesPerFrame, 16, 256);
    out.optimizationWindow = std::clamp(runtime.optimizationWindow, 4, 32);
    return out;
}

} // namespace SmartDrone::Adapters::Slam
