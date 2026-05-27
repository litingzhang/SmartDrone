#pragma once

#include <functional>
#include <memory>
#include <netinet/in.h>
#include <string>

#include "core/application/session/slam/slam_session_resource_factory.h"

namespace SmartDrone::Core::Ports {
class ICameraProvider;
class IImuProvider;
class ISlamEngine;
class IVisualFeatureFrontend;
} // namespace SmartDrone::Core::Ports

namespace SmartDrone::Core::Application {

class ImuSensorPoller;
class IPreviewOutputRuntime;

struct SlamSessionResourceLifecycleConfig {
    SmartDrone::Core::Ports::ISlamEngine &slamEngine;
    SmartDrone::Core::Ports::ICameraProvider *cameraProvider{nullptr};
    SmartDrone::Core::Ports::IImuProvider *imuProvider{nullptr};
    ImuSensorPoller *imuPoller{nullptr};
    IPreviewOutputRuntime *previewOutput{nullptr};
    bool useImu{false};
    bool udpEnabled{false};
    const MainRuntimeAliases &aliases;
    std::function<SmartDrone::Core::Ports::CameraOpenConfig(
        const MainRuntimeAliases &)>
        makeCameraOpenConfig;
    std::function<bool(sockaddr_in &)> resolveUdpDestination;
    std::function<SlamVisualFeatureFrontendStartResult()> startVisualFrontend;
    std::function<void(SmartDrone::Core::Ports::IVisualFeatureFrontend *)>
        attachVisualFrontend;
};

struct SlamSessionResourceStartResult {
    bool ok{false};
    bool featureRouteAvailable{false};
    bool featureClientMissing{false};
    bool featureStarted{false};
    std::string featureRepoPath;
    std::string featureError;
    SmartDrone::Core::Ports::IVisualFeatureFrontend *featureFrontend{nullptr};
};

class SlamSessionResourceLifecycle final {
  public:
    explicit SlamSessionResourceLifecycle(
        SlamSessionResourceLifecycleConfig config);
    ~SlamSessionResourceLifecycle();

    bool StartSlamEngine();
    SlamSessionResourceStartResult StartFrameResources();
    void Stop(bool logProgress);

  private:
    void StartVisualFeatureFrontend(SlamSessionResourceStartResult &result);
    bool OpenUdp();
    bool StartImuResources();
    bool OpenCamera();

    SlamSessionResourceLifecycleConfig m_config;
    std::unique_ptr<ISlamVisualFeatureFrontendSession> m_visualFeatureSession;
    bool m_slamStarted{false};
    bool m_udpOpen{false};
    bool m_imuProviderStarted{false};
    bool m_cameraOpen{false};
};

} // namespace SmartDrone::Core::Application
