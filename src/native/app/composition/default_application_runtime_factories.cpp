#include "app/composition/default_application_runtime_factories.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>

#include "adapters/camera/camera_provider_factory.h"
#include "adapters/imu/icm42688_imu_provider.h"
#include "adapters/slam/engine/slam_engine_factory.h"
#include "adapters/slam/superpoint/visual_feature_frontend_client.h"
#include "adapters/stream/udp_image_sender.h"
#include "core/application/config/app_args.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/sensors/imu_runtime_state.h"
#include "core/application/session/calib/calib_image_utils.h"
#include "core/application/session/epg/messages/calib_epg_messages.h"
#include "core/application/session/slam/slam_runtime_control_port.h"
#include "core/application/session/stream/preview_output_port.h"

namespace SmartDrone::App::Composition {
namespace {

using ApplicationRuntimeFactories =
    SmartDrone::Core::Application::ApplicationRuntimeFactories;
using CalibStereoFrame = SmartDrone::Core::Application::CalibStereoFrame;
using IPreviewOutputPort =
    SmartDrone::Core::Application::IPreviewOutputPort;
using IPreviewOutputRuntime =
    SmartDrone::Core::Application::IPreviewOutputRuntime;
using MainRuntimeAliases =
    SmartDrone::Core::Application::MainRuntimeAliases;
using PreviewOutputFrame =
    SmartDrone::Core::Application::PreviewOutputFrame;
using PreviewOutputOpenConfig =
    SmartDrone::Core::Application::PreviewOutputOpenConfig;
using CameraOpenConfig = SmartDrone::Core::Ports::CameraOpenConfig;
using SlamInputMode = SmartDrone::Core::Ports::SlamInputMode;
using SlamRuntimeControlPort =
    SmartDrone::Core::Application::SlamRuntimeControlPort;
using SlamSessionEngineResourceConfig =
    SmartDrone::Core::Application::SlamSessionEngineResourceConfig;
using SlamSessionEngineResources =
    SmartDrone::Core::Application::SlamSessionEngineResources;
using SlamVisualFeatureFrontendStartResult =
    SmartDrone::Core::Application::SlamVisualFeatureFrontendStartResult;
using UnifiedConfig = SmartDrone::Core::Application::UnifiedConfig;

class UdpPreviewOutputPort final : public IPreviewOutputPort {
  public:
    explicit UdpPreviewOutputPort(UdpImageSender &udpSender)
        : m_udpSender(udpSender)
    {
    }

    void Enqueue(const PreviewOutputFrame &frame) override
    {
        m_udpSender.Enqueue(UdpImageSender::EnqueueRequest{
            frame.camIndex, frame.frameId, frame.sequence, frame.frameTime,
            frame.gray, frame.trackedPoints, frame.sendImage,
            frame.sendFeature});
    }

    void StepAll() override
    {
        m_udpSender.StepAll();
    }

    void StepOnce() override
    {
        m_udpSender.StepOnce();
    }

  private:
    UdpImageSender &m_udpSender;
};

class UdpPreviewOutputRuntime final : public IPreviewOutputRuntime {
  public:
    bool Open(const PreviewOutputOpenConfig &config,
              DestinationResolver destinationResolver) override
    {
        return m_udp.Open(UdpImageSender::OpenConfig{
            config.ip, config.port, config.jpegQuality, config.maxPayload,
            config.maxQueue, std::move(destinationResolver)});
    }

    bool OpenStaticPeer(const PreviewOutputOpenConfig &config) override
    {
        return m_udp.Open(UdpImageSender::OpenConfig{
            config.ip, config.port, config.jpegQuality, config.maxPayload,
            config.maxQueue});
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
            SmartDrone::Core::Application::EnsureCalibGray8(
                left.gray, convertedLeft);
        const cv::Mat rightGray =
            SmartDrone::Core::Application::EnsureCalibGray8(
                right.gray, convertedRight);
        const std::vector<cv::Point2f> trackedPoints{};
        m_udp.Enqueue(UdpImageSender::EnqueueRequest{
            0, static_cast<std::uint64_t>(left.sequence), left.sequence,
            pairNs * 1e-9, leftGray, trackedPoints, true, false});
        m_udp.Enqueue(UdpImageSender::EnqueueRequest{
            1, static_cast<std::uint64_t>(right.sequence), right.sequence,
            pairNs * 1e-9, rightGray, trackedPoints, true, false});
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

SmartDrone::Adapters::Imu::Icm42688ImuProviderConfig
MakeImuProviderConfig(const MainRuntimeAliases &aliases)
{
    const int64_t imuDtNs = 1000000000LL / std::max(1, aliases.imuHz);
    const int64_t slackBeforeNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    const int64_t slackAfterNs = std::max<int64_t>(2 * imuDtNs, 5000000);
    return {slackBeforeNs, slackAfterNs};
}

CameraOpenConfig BuildCameraOpenConfig(const MainRuntimeAliases &aliases)
{
    CameraOpenConfig config{};
    config.width = aliases.width;
    config.height = aliases.height;
    config.fps = aliases.fps;
    config.leftCameraIndex = aliases.leftCamIndex;
    config.rightCameraIndex = aliases.rightCamIndex;
    config.exposureUs = aliases.exposureUs;
    config.pairWindowMs = aliases.pairMs;
    config.keepWindowMs = aliases.keepMs;
    config.pairQueue = aliases.pairQueue;
    config.uvcDeviceIndex = aliases.uvcDeviceIndex;
    config.uvcEyeWidth = aliases.uvcEyeWidth;
    config.uvcEyeHeight = aliases.uvcEyeHeight;
    config.autoExposureDisabled = aliases.aeDisable;
    config.requestY8 = aliases.requestY8;
    config.r16Normalize = aliases.r16Norm;
    config.uvcPackedStereo = aliases.uvcPackedStereo;
    config.uvcSwapEyes = aliases.uvcSwapEyes;
    config.gain = aliases.gain;
    return config;
}

SmartDrone::Adapters::Slam::SlamEngineFactoryConfig
BuildEngineConfig(const SlamSessionEngineResourceConfig &config)
{
    SmartDrone::Adapters::Slam::SlamEngineFactoryConfig engineConfig{};
    engineConfig.backend = config.aliases.slamBackend;
    engineConfig.vocabularyPath = config.cfg.app.vocab;
    engineConfig.settingsPath = config.settingsPath;
    engineConfig.sensorMode = config.aliases.sensorMode;
    engineConfig.useViewer = false;
    engineConfig.useImu = config.useImu;
    engineConfig.inputMode = ResolveSlamInputMode(config.aliases);
    engineConfig.dpvoRuntime.repoPath = config.cfg.app.runtime.dpvoRepo;
    engineConfig.dpvoRuntime.patchEnginePath =
        config.cfg.app.runtime.dpvoPatchEngine;
    engineConfig.dpvoRuntime.updateEnginePath =
        config.cfg.app.runtime.dpvoUpdateEngine;
    engineConfig.dpvoRuntime.inputWidth =
        config.cfg.app.runtime.dpvoInputWidth;
    engineConfig.dpvoRuntime.inputHeight =
        config.cfg.app.runtime.dpvoInputHeight;
    engineConfig.dpvoRuntime.patchesPerFrame =
        config.cfg.app.runtime.dpvoPatchesPerFrame;
    engineConfig.dpvoRuntime.optimizationWindow =
        config.cfg.app.runtime.dpvoOptimizationWindow;
    return engineConfig;
}

SmartDrone::Adapters::Slam::VisualFeatureFrontendRuntimeConfig
BuildVisualFeatureConfig(const MainRuntimeAliases &aliases,
                         const UnifiedConfig &cfg)
{
    SmartDrone::Adapters::Slam::VisualFeatureFrontendRuntimeConfig config{};
    config.repoPath =
        SmartDrone::Adapters::Slam::ResolveVisualFeatureFrontendRepo(
            aliases.featureFrontend, cfg.app.runtime.visualFeatureRepo);
    config.device = cfg.app.runtime.visualFeatureDevice;
    config.topK = cfg.app.runtime.visualFeatureTopK;
    config.maxPoints = cfg.app.runtime.visualFeatureMaxPoints;
    config.inputMaxWidth = cfg.app.runtime.visualFeatureInputMaxWidth;
    config.inputMaxHeight = cfg.app.runtime.visualFeatureInputMaxHeight;
    return config;
}

class SlamVisualFeatureFrontendSession final
    : public SmartDrone::Core::Application::ISlamVisualFeatureFrontendSession {
  public:
    explicit SlamVisualFeatureFrontendSession(
        std::unique_ptr<
            SmartDrone::Adapters::Slam::IManagedVisualFeatureFrontend>
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

    SmartDrone::Core::Ports::IVisualFeatureFrontend *Frontend()
    {
        return m_client.get();
    }

  private:
    std::unique_ptr<SmartDrone::Adapters::Slam::IManagedVisualFeatureFrontend>
        m_client;
};

SlamSessionEngineResources CreateSlamEngineResources(
    const SlamSessionEngineResourceConfig &config)
{
    SmartDrone::Adapters::Slam::ControlledSlamEngine slamEngine =
        SmartDrone::Adapters::Slam::CreateSlamEngine(BuildEngineConfig(config));

    SlamSessionEngineResources resources{};
    resources.control =
        std::make_unique<SlamRuntimeControlPort>(slamEngine.control);
    resources.backendMaintenance = slamEngine.backendMaintenance;
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
    SmartDrone::Adapters::Slam::ConfigureVisualFeatureFrontendDefaults(
        aliases.featureFrontend, featureConfig);
    result.repoPath = featureConfig.repoPath;
    if (!SmartDrone::Adapters::Slam::VisualFeatureFrontendClientEnabled(
            aliases.featureFrontend)) {
        return result;
    }

    auto client = SmartDrone::Adapters::Slam::CreateVisualFeatureFrontendClient(
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
    factories.measurementClock =
        SmartDrone::Adapters::Camera::CreateMeasurementClock();
    const auto measurementClock = factories.measurementClock;
    factories.externalPoseSource =
        SmartDrone::Adapters::Camera::CreateExternalPoseSource(
            measurementClock);
    factories.createCameraProvider =
        [measurementClock]() {
            return SmartDrone::Adapters::Camera::CreateCameraProvider(
                measurementClock);
        };
    factories.makeCameraOpenConfig =
        [](const MainRuntimeAliases &aliases) {
            return BuildCameraOpenConfig(aliases);
        };
    factories.createSlamEngineResources =
        [](const SlamSessionEngineResourceConfig &config) {
            return CreateSlamEngineResources(config);
        };
    factories.createImuProvider =
        [](SmartDrone::Core::Application::ImuThreadState &state,
           const MainRuntimeAliases &aliases) {
            return std::make_unique<
                SmartDrone::Adapters::Imu::Icm42688ImuProvider>(
                state.imuBuffer, MakeImuProviderConfig(aliases));
        };
    factories.startVisualFeatureFrontendSession =
        [](const MainRuntimeAliases &aliases, const UnifiedConfig &cfg) {
            return StartVisualFeatureFrontendSession(aliases, cfg);
        };
    factories.createPreviewOutputRuntime =
        []() { return std::make_unique<UdpPreviewOutputRuntime>(); };
    factories.cameraProvider = {
        std::string(SmartDrone::Adapters::Camera::CompiledCameraProviderName()),
        SmartDrone::Adapters::Camera::CompiledCameraProviderUsesPackedStereo(),
    };
    factories.cameraOverrides =
        SmartDrone::Adapters::Camera::LoadCameraProviderRuntimeOverrides();
    return factories;
}

} // namespace SmartDrone::App::Composition
