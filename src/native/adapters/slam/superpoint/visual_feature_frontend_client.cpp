#include "adapters/slam/superpoint/visual_feature_frontend_client.h"

#include <array>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <system_error>
#include <vector>

#include "common/environment.h"

namespace SmartDrone::Adapters::Slam {

namespace {

constexpr size_t kMaxVisualFeatureFrontendClientSlots = 16;

struct VisualFeatureFrontendClientRegistryEntry {
    FeatureFrontend frontend{FeatureFrontend::Orb};
    VisualFeatureFrontendClientFactory factory{nullptr};
};

std::array<VisualFeatureFrontendClientRegistryEntry,
           kMaxVisualFeatureFrontendClientSlots> &
VisualFeatureFrontendClientRegistry()
{
    static std::array<VisualFeatureFrontendClientRegistryEntry,
                      kMaxVisualFeatureFrontendClientSlots>
        registry{};
    return registry;
}

std::mutex &VisualFeatureFrontendClientRegistryMutex()
{
    static std::mutex mutex;
    return mutex;
}

void SetStereoFeatureEnvDefault(const char *name, const char *legacyName,
                                const char *value)
{
    if (!SmartDrone::Common::EnvVarIsUnsetOrEmpty(name) ||
        !SmartDrone::Common::EnvVarIsUnsetOrEmpty(legacyName)) {
        return;
    }
    SmartDrone::Common::SetEnvVarIfUnset(name, value);
}

std::string FirstExistingPathOrFallback(
    const std::vector<std::filesystem::path> &candidates,
    const std::string &fallback)
{
    for (const std::filesystem::path &candidate : candidates) {
        if (candidate.empty()) {
            continue;
        }
        std::error_code ec;
        if (std::filesystem::exists(candidate, ec)) {
            return candidate.string();
        }
    }
    return fallback;
}

std::string ResolveLightGlueRepo(const std::string &configuredRepo)
{
    const char *home = std::getenv("HOME");
    std::vector<std::filesystem::path> candidates;
    if (home != nullptr && home[0] != '\0') {
        candidates.emplace_back(std::filesystem::path(home) / "LightGlue");
        candidates.emplace_back(std::filesystem::path(home) / "lightglue");
        candidates.emplace_back(std::filesystem::path(home) / "third_party" /
                                "LightGlue");
        candidates.emplace_back(std::filesystem::path(home) / "third_party" /
                                "lightglue");
    }
    candidates.emplace_back("LightGlue");
    candidates.emplace_back("lightglue");
    candidates.emplace_back("third_party/LightGlue");
    candidates.emplace_back("third_party/lightglue");
    candidates.emplace_back(configuredRepo);
    return FirstExistingPathOrFallback(candidates, configuredRepo);
}

void ConfigureLightGlueEnvironmentDefaults()
{
    SmartDrone::Common::SetEnvVarIfUnset("SMART_DRONE_FEATURE_PRECISION",
                                         "auto");
    SmartDrone::Common::SetEnvVarIfUnset("SMART_DRONE_LIGHTGLUE_LAYERS", "6");
    SmartDrone::Common::SetEnvVarIfUnset(
        "SMART_DRONE_SP_LG_FILTERED_STEREO_INJECT", "1");
    SetStereoFeatureEnvDefault("SMART_DRONE_STEREO_FEATURE_INIT_MIN_FEATURES",
                               "SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_FEATURES",
                               "72");
    SetStereoFeatureEnvDefault(
        "SMART_DRONE_STEREO_FEATURE_INIT_MIN_CLOSE_POINTS",
        "SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_POINTS", "24");
    SetStereoFeatureEnvDefault("SMART_DRONE_STEREO_FEATURE_INIT_MIN_CLOSE_RATIO",
                               "SMART_DRONE_EXTERNAL_STEREO_INIT_MIN_CLOSE_RATIO",
                               "0.30");
    SetStereoFeatureEnvDefault("SMART_DRONE_STEREO_FEATURE_DEPTH_SCALE",
                               "SMART_DRONE_EXTERNAL_STEREO_DEPTH_SCALE",
                               "0.965");
    SmartDrone::Common::SetEnvVarIfUnset(
        "SMART_DRONE_SP_LG_INIT_TRUST_FRONTEND_PAIRS", "1");
    SmartDrone::Common::SetEnvVarIfUnset(
        "SMART_DRONE_SP_LG_RECOVERY_TRUST_FRONTEND_PAIRS", "1");
    SmartDrone::Common::SetEnvVarIfUnset(
        "SMART_DRONE_SP_LG_BOOTSTRAP_TRUST_FRONTEND_PAIRS", "1");
    SmartDrone::Common::SetEnvVarIfUnset(
        "SMART_DRONE_SP_LG_TRUST_FRONTEND_PAIRS_OK_STREAK", "120");
    SmartDrone::Common::SetEnvVarIfUnset(
        "SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE", "1");
    SmartDrone::Common::SetEnvVarIfUnset(
        "SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE_TIMEOUT_MS", "35");
    SmartDrone::Common::SetEnvVarIfUnset(
        "SMART_DRONE_ORB_LIVE_EUROC_TRAJECTORY_POSE", "0");
    SmartDrone::Common::SetEnvVarIfUnset("SMART_DRONE_REALTIME_POSE_CONTINUITY",
                                         "1");
}

std::vector<std::string> BuildSuperPointEngineCandidates(
    const VisualFeatureFrontendRuntimeConfig &config)
{
    std::vector<std::string> engineNames;
    if (config.inputMaxWidth > 0 && config.inputMaxHeight > 0) {
        const std::string sizeText =
            std::to_string(config.inputMaxWidth) + "x" +
            std::to_string(config.inputMaxHeight);
        engineNames.push_back("superpoint_dense_" + sizeText + "_fp16.engine");
        engineNames.push_back("superpoint_dense_" + sizeText + "_fp32.engine");
    }
    engineNames.push_back("superpoint_dense_640x409_fp16.engine");
    engineNames.push_back("superpoint_dense_640x409_fp32.engine");
    engineNames.push_back("superpoint_dense_640x480_fp16.engine");
    engineNames.push_back("superpoint_dense_640x480_fp32.engine");
    return engineNames;
}

void ConfigureSuperPointEngineDefault(
    const VisualFeatureFrontendRuntimeConfig &config)
{
    if (!SmartDrone::Common::EnvVarIsUnsetOrEmpty(
            "SMART_DRONE_SUPERPOINT_TRT_ENGINE") ||
        config.repoPath.empty()) {
        return;
    }

    for (const std::string &engineName :
         BuildSuperPointEngineCandidates(config)) {
        const std::filesystem::path engine =
            std::filesystem::path(config.repoPath) / "weights" / engineName;
        std::error_code ec;
        if (std::filesystem::exists(engine, ec)) {
            SmartDrone::Common::SetEnvVarIfUnset(
                "SMART_DRONE_SUPERPOINT_TRT_ENGINE",
                engine.string().c_str());
            return;
        }
    }
}

void ConfigureLightGlueFrontendDefaults(
    const VisualFeatureFrontendRuntimeConfig &config)
{
    ConfigureLightGlueEnvironmentDefaults();
    ConfigureSuperPointEngineDefault(config);
}

} // namespace

void RegisterVisualFeatureFrontendClient(
    FeatureFrontend frontend, VisualFeatureFrontendClientFactory factory)
{
    if (factory == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(VisualFeatureFrontendClientRegistryMutex());
    auto &registry = VisualFeatureFrontendClientRegistry();
    for (VisualFeatureFrontendClientRegistryEntry &entry : registry) {
        if (entry.factory != nullptr && entry.frontend == frontend) {
            entry.factory = factory;
            return;
        }
    }
    for (VisualFeatureFrontendClientRegistryEntry &entry : registry) {
        if (entry.factory == nullptr) {
            entry.frontend = frontend;
            entry.factory = factory;
            return;
        }
    }
}

VisualFeatureFrontendClientRegistrar::VisualFeatureFrontendClientRegistrar(
    FeatureFrontend frontend, VisualFeatureFrontendClientFactory factory)
{
    RegisterVisualFeatureFrontendClient(frontend, factory);
}

std::unique_ptr<IManagedVisualFeatureFrontend>
CreateVisualFeatureFrontendClient(FeatureFrontend frontend)
{
    std::lock_guard<std::mutex> lock(VisualFeatureFrontendClientRegistryMutex());
    for (const VisualFeatureFrontendClientRegistryEntry &entry :
         VisualFeatureFrontendClientRegistry()) {
        if (entry.factory != nullptr && entry.frontend == frontend) {
            return entry.factory();
        }
    }
    return nullptr;
}

bool VisualFeatureFrontendClientEnabled(FeatureFrontend frontend)
{
    if (frontend != FeatureFrontend::SuperPointLightGlue &&
        frontend != FeatureFrontend::XFeatLightGlue) {
        return false;
    }
    const char *value = std::getenv("SMART_DRONE_SUPERPOINT_LIGHTGLUE_INJECT");
    if (value == nullptr || value[0] == '\0') {
        return true;
    }
    const std::string text(value);
    return !(text == "0" || text == "false" || text == "FALSE" || text == "off" ||
             text == "OFF");
}

std::string
ResolveVisualFeatureFrontendRepo(FeatureFrontend frontend,
                                 const std::string &configuredRepo)
{
    if (frontend == FeatureFrontend::SuperPointLightGlue ||
        frontend == FeatureFrontend::XFeatLightGlue) {
        return ResolveLightGlueRepo(configuredRepo);
    }
    return configuredRepo;
}

void ConfigureVisualFeatureFrontendDefaults(
    FeatureFrontend frontend,
    const VisualFeatureFrontendRuntimeConfig &config)
{
    if (frontend == FeatureFrontend::SuperPointLightGlue ||
        frontend == FeatureFrontend::XFeatLightGlue) {
        ConfigureLightGlueFrontendDefaults(config);
    }
}

} // namespace SmartDrone::Adapters::Slam
