#include "core/application/session/slam/slam_session_resource_factory.h"

#include <algorithm>
#include <cstdint>
#include <utility>

#include "adapters/imu/icm42688_imu_provider.h"
#include "adapters/slam/slam_engine_factory.h"
#include "adapters/slam/visual_feature_frontend_client.h"
#include "core/application/config/app_args.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/sensors/camera_runtime_provider.h"
#include "core/application/sensors/imu_runtime_state.h"
#include "core/application/session/slam/slam_runtime_control_port.h"

namespace smartdrone::core::application {

namespace {

using SlamInputMode = smartdrone::core::ports::SlamInputMode;

SlamInputMode ResolveSlamInputMode(const MainRuntimeAliases &aliases)
{
    const bool monoMode = aliases.sensorMode == SensorMode::Mono ||
                          aliases.sensorMode == SensorMode::MonoImu;
    return monoMode ? SlamInputMode::MonoRight : SlamInputMode::Stereo;
}

smartdrone::adapters::imu::Icm42688ImuProviderConfig
MakeImuProviderConfig(const MainRuntimeAliases &aliases)
{
    const int64_t imuDtNs = 1000000000LL / std::max(1, aliases.imuHz);
    const int64_t slackBeforeNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    const int64_t slackAfterNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    return {slackBeforeNs, slackAfterNs};
}

smartdrone::adapters::slam::SlamEngineFactoryConfig
BuildEngineConfig(const SlamSessionEngineResourceConfig &config)
{
    smartdrone::adapters::slam::SlamEngineFactoryConfig engineConfig{};
    engineConfig.backend = config.aliases.slamBackend;
    engineConfig.vocabularyPath = config.cfg.app.vocab;
    engineConfig.settingsPath = config.settingsPath;
    engineConfig.sensorMode = config.aliases.sensorMode;
    engineConfig.useViewer = false;
    engineConfig.useImu = config.useImu;
    engineConfig.inputMode = ResolveSlamInputMode(config.aliases);
    engineConfig.runtime = config.cfg.app.runtime;
    return engineConfig;
}

smartdrone::adapters::slam::VisualFeatureFrontendRuntimeConfig
BuildVisualFeatureConfig(const MainRuntimeAliases &aliases,
                         const UnifiedConfig &cfg)
{
    smartdrone::adapters::slam::VisualFeatureFrontendRuntimeConfig config{};
    config.repoPath = smartdrone::adapters::slam::ResolveVisualFeatureFrontendRepo(
        aliases.featureFrontend, cfg.app.runtime.visualFeatureRepo);
    config.device = cfg.app.runtime.visualFeatureDevice;
    config.topK = cfg.app.runtime.visualFeatureTopK;
    config.maxPoints = cfg.app.runtime.visualFeatureMaxPoints;
    config.inputMaxWidth = cfg.app.runtime.visualFeatureInputMaxWidth;
    config.inputMaxHeight = cfg.app.runtime.visualFeatureInputMaxHeight;
    return config;
}

class SlamVisualFeatureFrontendSession final
    : public ISlamVisualFeatureFrontendSession {
  public:
    explicit SlamVisualFeatureFrontendSession(
        std::unique_ptr<smartdrone::adapters::slam::IManagedVisualFeatureFrontend>
            client)
        : m_client(std::move(client))
    {
    }

    void Stop() override
    {
        if (m_client == nullptr) {
            return;
        }
        m_client->Stop();
    }

    smartdrone::core::ports::IVisualFeatureFrontend *Frontend()
    {
        return m_client.get();
    }

  private:
    std::unique_ptr<smartdrone::adapters::slam::IManagedVisualFeatureFrontend>
        m_client;
};

} // namespace

SlamSessionEngineResources CreateSlamSessionEngineResources(
    const SlamSessionEngineResourceConfig &config)
{
    smartdrone::adapters::slam::ControlledSlamEngine slamEngine =
        smartdrone::adapters::slam::CreateSlamEngine(BuildEngineConfig(config));

    SlamSessionEngineResources resources{};
    resources.control =
        std::make_unique<SlamRuntimeControlPort>(slamEngine.control);
    resources.engine = std::move(slamEngine.engine);
    return resources;
}

std::unique_ptr<smartdrone::core::ports::ICameraProvider>
CreateSlamSessionCameraProvider()
{
    return CreateCameraProvider();
}

std::unique_ptr<smartdrone::core::ports::IImuProvider>
CreateSlamSessionImuProvider(ImuThreadState &state,
                             const MainRuntimeAliases &aliases)
{
    return std::make_unique<smartdrone::adapters::imu::Icm42688ImuProvider>(
        state.imuBuffer, MakeImuProviderConfig(aliases));
}

SlamVisualFeatureFrontendStartResult StartSlamVisualFeatureFrontendSession(
    const MainRuntimeAliases &aliases, const UnifiedConfig &cfg)
{
    SlamVisualFeatureFrontendStartResult result{};
    if (!IsVisualFeatureLightGlueFrontend(aliases.featureFrontend)) {
        return result;
    }

    result.routeAvailable = true;
    auto featureConfig = BuildVisualFeatureConfig(aliases, cfg);
    smartdrone::adapters::slam::ConfigureVisualFeatureFrontendDefaults(
        aliases.featureFrontend, featureConfig);
    result.repoPath = featureConfig.repoPath;
    if (!smartdrone::adapters::slam::VisualFeatureFrontendClientEnabled(
            aliases.featureFrontend)) {
        return result;
    }

    auto client = smartdrone::adapters::slam::CreateVisualFeatureFrontendClient(
        aliases.featureFrontend);
    if (client == nullptr) {
        result.clientMissing = true;
        return result;
    }

    if (!client->Start(featureConfig, &result.error)) {
        return result;
    }

    auto session =
        std::make_unique<SlamVisualFeatureFrontendSession>(std::move(client));
    result.frontend = session->Frontend();
    result.session = std::move(session);
    result.started = true;
    return result;
}

} // namespace smartdrone::core::application
