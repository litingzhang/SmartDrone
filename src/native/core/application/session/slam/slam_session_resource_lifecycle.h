#pragma once

#include <functional>
#include <memory>
#include <netinet/in.h>
#include <string>

#include "core/application/session/slam/slam_session_resource_factory.h"

namespace SmartDrone::core::ports {
class ICameraProvider;
class IImuProvider;
class ISlamEngine;
class IVisualFeatureFrontend;
} // namespace SmartDrone::core::ports

namespace SmartDrone::core::application {

class ImuSensorPoller;
class IPreviewOutputRuntime;

struct SlamSessionResourceLifecycleConfig {
    SmartDrone::core::ports::ISlamEngine &slamEngine;
    SmartDrone::core::ports::ICameraProvider *cameraProvider{nullptr};
    ImuSensorPoller *imuPoller{nullptr};
    IPreviewOutputRuntime *previewOutput{nullptr};
    bool useImu{false};
    bool udpEnabled{false};
    const MainRuntimeAliases &aliases;
    std::function<SmartDrone::core::ports::CameraOpenConfig(
        const MainRuntimeAliases &)>
        makeCameraOpenConfig;
    std::function<bool(sockaddr_in &)> resolveUdpDestination;
    std::function<SlamVisualFeatureFrontendStartResult()> startVisualFrontend;
    std::function<void(SmartDrone::core::ports::IVisualFeatureFrontend *)>
        attachVisualFrontend;
};

struct SlamSessionResourceStartResult {
    bool ok{false};
    bool featureRouteAvailable{false};
    bool featureClientMissing{false};
    bool featureStarted{false};
    std::string featureRepoPath;
    std::string featureError;
    SmartDrone::core::ports::IVisualFeatureFrontend *featureFrontend{nullptr};
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
    bool StartImuPoller();
    bool OpenCamera();

    SlamSessionResourceLifecycleConfig m_config;
    std::unique_ptr<ISlamVisualFeatureFrontendSession> m_visualFeatureSession;
    bool m_slamStarted{false};
    bool m_udpOpen{false};
    bool m_cameraOpen{false};
};

} // namespace SmartDrone::core::application
