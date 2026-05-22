#include "adapters/runtime/default_application_runtime_factories.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>

#include "adapters/camera/camera_provider_factory.h"
#include "adapters/imu/icm42688_imu_provider.h"
#include "adapters/slam/slam_engine_factory.h"
#include "adapters/slam/visual_feature_frontend_client.h"
#include "adapters/stream/udp_image_sender.h"
#include "core/application/config/app_args.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/sensors/imu_runtime_state.h"
#include "core/application/session/calib/calib_image_utils.h"
#include "core/application/session/epg/messages/calib_epg_messages.h"
#include "core/application/session/slam/slam_runtime_control_port.h"
#include "core/application/session/stream/preview_output_port.h"

namespace SmartDrone::adapters::runtime {
namespace {

using ApplicationRuntimeFactories =
    SmartDrone::core::application::ApplicationRuntimeFactories;
using CalibStereoFrame = SmartDrone::core::application::CalibStereoFrame;
using IPreviewOutputPort =
    SmartDrone::core::application::IPreviewOutputPort;
using IPreviewOutputRuntime =
    SmartDrone::core::application::IPreviewOutputRuntime;
using MainRuntimeAliases =
    SmartDrone::core::application::MainRuntimeAliases;
using PreviewOutputFrame =
    SmartDrone::core::application::PreviewOutputFrame;
using PreviewOutputOpenConfig =
    SmartDrone::core::application::PreviewOutputOpenConfig;
using SlamInputMode = SmartDrone::core::ports::SlamInputMode;
using SlamRuntimeControlPort =
    SmartDrone::core::application::SlamRuntimeControlPort;
using SlamSessionEngineResourceConfig =
    SmartDrone::core::application::SlamSessionEngineResourceConfig;
using SlamSessionEngineResources =
    SmartDrone::core::application::SlamSessionEngineResources;
using SlamVisualFeatureFrontendStartResult =
    SmartDrone::core::application::SlamVisualFeatureFrontendStartResult;
using UnifiedConfig = SmartDrone::core::application::UnifiedConfig;

class UdpPreviewOutputPort final : public IPreviewOutputPort {
  public:
    explicit UdpPreviewOutputPort(UdpImageSender &udpSender)
        : m_udpSender(udpSender)
    {
    }

    void Enqueue(const PreviewOutputFrame &frame) override
    {
        m_udpSender.Enqueue(frame.camIndex, frame.frameId, frame.sequence,
                            frame.frameTime, frame.gray, frame.trackedPoints,
                            frame.sendImage, frame.sendFeature);
    }

    void StepAll() override
    {
        m_udpSender.StepAll();
    }

  private:
    UdpImageSender &m_udpSender;
};

class UdpPreviewOutputRuntime final : public IPreviewOutputRuntime {
  public:
    bool Open(const PreviewOutputOpenConfig &config,
              DestinationResolver destinationResolver) override
    {
        return m_udp.Open(config.ip, config.port, config.jpegQuality,
                          config.maxPayload, config.maxQueue,
                          std::move(destinationResolver));
    }

    bool OpenStaticPeer(const PreviewOutputOpenConfig &config) override
    {
        return m_udp.Open(config.ip, config.port, config.jpegQuality,
                          config.maxPayload, config.maxQueue);
    }

    void Close() override
    {
        m_udp.Close();
    }

    void EnqueueCalibStereoFrame(const CalibStereoFrame &frame) override
    {
        const auto &left = frame.stereo.left;
        const auto &right = frame.stereo.right;
        const std::int64_t pairNs =
            static_cast<std::int64_t>((left.timestampNs + right.timestampNs) /
                                      2);
        bool convertedLeft = false;
        bool convertedRight = false;
        const cv::Mat leftGray =
            SmartDrone::core::application::EnsureCalibGray8(
                left.gray, convertedLeft);
        const cv::Mat rightGray =
            SmartDrone::core::application::EnsureCalibGray8(
                right.gray, convertedRight);
        m_udp.Enqueue(0, static_cast<std::uint64_t>(left.sequence),
                      left.sequence, pairNs * 1e-9, leftGray, {}, true,
                      false);
        m_udp.Enqueue(1, static_cast<std::uint64_t>(right.sequence),
                      right.sequence, pairNs * 1e-9, rightGray, {}, true,
                      false);
        m_udp.StepAll();
    }

    IPreviewOutputPort &OutputPort() override
    {
        return m_outputPort;
    }

  private:
    UdpImageSender m_udp;
    UdpPreviewOutputPort m_outputPort{m_udp};
};

SlamInputMode ResolveSlamInputMode(const MainRuntimeAliases &aliases)
{
    const bool monoMode = aliases.sensorMode ==
                              SensorMode::Mono ||
                          aliases.sensorMode ==
                              SensorMode::MonoImu;
    return monoMode ? SlamInputMode::MonoRight : SlamInputMode::Stereo;
}

SmartDrone::adapters::imu::Icm42688ImuProviderConfig
MakeImuProviderConfig(const MainRuntimeAliases &aliases)
{
    const int64_t imuDtNs = 1000000000LL / std::max(1, aliases.imuHz);
    const int64_t slackBeforeNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    const int64_t slackAfterNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    return {slackBeforeNs, slackAfterNs};
}

SmartDrone::adapters::slam::SlamEngineFactoryConfig
BuildEngineConfig(const SlamSessionEngineResourceConfig &config)
{
    SmartDrone::adapters::slam::SlamEngineFactoryConfig engineConfig{};
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

SmartDrone::adapters::slam::VisualFeatureFrontendRuntimeConfig
BuildVisualFeatureConfig(const MainRuntimeAliases &aliases,
                         const UnifiedConfig &cfg)
{
    SmartDrone::adapters::slam::VisualFeatureFrontendRuntimeConfig config{};
    config.repoPath =
        SmartDrone::adapters::slam::ResolveVisualFeatureFrontendRepo(
            aliases.featureFrontend, cfg.app.runtime.visualFeatureRepo);
    config.device = cfg.app.runtime.visualFeatureDevice;
    config.topK = cfg.app.runtime.visualFeatureTopK;
    config.maxPoints = cfg.app.runtime.visualFeatureMaxPoints;
    config.inputMaxWidth = cfg.app.runtime.visualFeatureInputMaxWidth;
    config.inputMaxHeight = cfg.app.runtime.visualFeatureInputMaxHeight;
    return config;
}

class SlamVisualFeatureFrontendSession final
    : public SmartDrone::core::application::ISlamVisualFeatureFrontendSession {
  public:
    explicit SlamVisualFeatureFrontendSession(
        std::unique_ptr<
            SmartDrone::adapters::slam::IManagedVisualFeatureFrontend>
            client)
        : m_client(std::move(client))
    {
    }

    void Stop() override
    {
        if (m_client != nullptr) {
            m_client->Stop();
        }
    }

    SmartDrone::core::ports::IVisualFeatureFrontend *Frontend()
    {
        return m_client.get();
    }

  private:
    std::unique_ptr<SmartDrone::adapters::slam::IManagedVisualFeatureFrontend>
        m_client;
};

SlamSessionEngineResources CreateSlamEngineResources(
    const SlamSessionEngineResourceConfig &config)
{
    SmartDrone::adapters::slam::ControlledSlamEngine slamEngine =
        SmartDrone::adapters::slam::CreateSlamEngine(BuildEngineConfig(config));

    SlamSessionEngineResources resources{};
    resources.control =
        std::make_unique<SlamRuntimeControlPort>(slamEngine.control);
    resources.engine = std::move(slamEngine.engine);
    return resources;
}

SlamVisualFeatureFrontendStartResult StartVisualFeatureFrontendSession(
    const MainRuntimeAliases &aliases, const UnifiedConfig &cfg)
{
    SlamVisualFeatureFrontendStartResult result{};
    if (!IsVisualFeatureLightGlueFrontend(aliases.featureFrontend)) {
        return result;
    }

    result.routeAvailable = true;
    auto featureConfig = BuildVisualFeatureConfig(aliases, cfg);
    SmartDrone::adapters::slam::ConfigureVisualFeatureFrontendDefaults(
        aliases.featureFrontend, featureConfig);
    result.repoPath = featureConfig.repoPath;
    if (!SmartDrone::adapters::slam::VisualFeatureFrontendClientEnabled(
            aliases.featureFrontend)) {
        return result;
    }

    auto client = SmartDrone::adapters::slam::CreateVisualFeatureFrontendClient(
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

} // namespace

ApplicationRuntimeFactories CreateDefaultApplicationRuntimeFactories()
{
    ApplicationRuntimeFactories factories{};
    factories.createCameraProvider =
        []() { return SmartDrone::adapters::camera::CreateCameraProvider(); };
    factories.makeCameraOpenConfig =
        [](const MainRuntimeAliases &aliases) {
            return SmartDrone::adapters::camera::MakeCameraOpenConfig(aliases);
        };
    factories.createSlamEngineResources =
        [](const SlamSessionEngineResourceConfig &config) {
            return CreateSlamEngineResources(config);
        };
    factories.createImuProvider =
        [](SmartDrone::core::application::ImuThreadState &state,
           const MainRuntimeAliases &aliases) {
            return std::make_unique<
                SmartDrone::adapters::imu::Icm42688ImuProvider>(
                state.imuBuffer, MakeImuProviderConfig(aliases));
        };
    factories.startVisualFeatureFrontendSession =
        [](const MainRuntimeAliases &aliases, const UnifiedConfig &cfg) {
            return StartVisualFeatureFrontendSession(aliases, cfg);
        };
    factories.createPreviewOutputRuntime =
        []() { return std::make_unique<UdpPreviewOutputRuntime>(); };
    factories.cameraProvider = {
        std::string(SmartDrone::adapters::camera::CompiledCameraProviderName()),
        SmartDrone::adapters::camera::CompiledCameraProviderUsesPackedStereo(),
    };
    return factories;
}

} // namespace SmartDrone::adapters::runtime
