#include "adapters/slam/dpvo/dpvo_tensorrt_engine.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <sophus/se3.hpp>

#include "adapters/slam/engine/slam_engine_factory.h"
#include "adapters/slam/engine/slam_env.h"
#include "adapters/slam/engine/slam_image_utils.h"
#include "adapters/slam/engine/slam_mode_state.h"
#include "adapters/slam/engine/slam_pose_utils.h"
#include "adapters/slam/dpvo/dpvo_runtime_options.h"
#include "core/ports/slam_tracking_state.h"

namespace SmartDrone::Adapters::Slam {

namespace {

std::filesystem::path ResolveEnginePath(const std::string &explicitPath,
                                        const std::string &repoPath,
                                        const std::vector<std::string> &names)
{
    if (!explicitPath.empty() && std::filesystem::exists(explicitPath)) {
        return std::filesystem::path(explicitPath);
    }
    const std::filesystem::path repo(repoPath);
    if (!repo.empty()) {
        for (const std::string &name : names) {
            const std::filesystem::path candidate = repo / "weights" / name;
            if (std::filesystem::exists(candidate)) {
                return candidate;
            }
        }
    }
    return {};
}

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
    if (const char *envRepo = std::getenv("SMART_DRONE_DPVO_REPO");
        envRepo != nullptr && envRepo[0] != '\0') {
        addCandidate(envRepo);
    }
    if (const char *home = std::getenv("HOME");
        home != nullptr && home[0] != '\0') {
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
    if (const char *envRepo = std::getenv("SMART_DRONE_DPVO_REPO");
        envRepo != nullptr && envRepo[0] != '\0') {
        return std::filesystem::path(envRepo).lexically_normal();
    }
    return {};
}

double ElapsedMs(const std::chrono::steady_clock::time_point &start,
                 const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

int64_t SteadyNowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

} // namespace

#include "dpvo_tensorrt_engine_fallback.h"

DpvoTensorRtEngine::DpvoTensorRtEngine(DpvoTensorRtConfig config)
    : m_impl(std::make_unique<Impl>(std::move(config)))
{
}
DpvoTensorRtEngine::~DpvoTensorRtEngine() = default;

bool DpvoTensorRtEngine::Start()
{
    return m_impl != nullptr && m_impl->Start();
}

void DpvoTensorRtEngine::Stop()
{
    if (m_impl != nullptr) {
        m_impl->Stop();
    }
}

Core::Ports::SlamOutput
DpvoTensorRtEngine::Process(const Core::Ports::SlamInputBatch &input,
                            bool extractFeatures, bool extractPointCloud)
{
    return m_impl != nullptr
               ? m_impl->Process(input, extractFeatures, extractPointCloud)
               : Core::Ports::SlamOutput{};
}

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

namespace {

ControlledSlamEngine
CreateDpvoTensorRtSlamEngine(const SlamEngineFactoryConfig &config)
{
    DpvoTensorRtConfig dpvoConfig =
        MakeDpvoTensorRtConfig(config.dpvoRuntime, config.settingsPath);
    ControlledSlamEngine out{};
    out.engine = std::make_unique<DpvoTensorRtEngine>(std::move(dpvoConfig));
    return out;
}

const SlamEngineFactoryRegistrar
    kDpvoTensorRtSlamEngineRegistrar(SlamBackend::DpvoTensorRt,
                                     CreateDpvoTensorRtSlamEngine);

} // namespace

} // namespace SmartDrone::Adapters::Slam
