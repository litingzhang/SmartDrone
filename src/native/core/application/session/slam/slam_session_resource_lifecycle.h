#pragma once

#include <functional>
#include <memory>
#include <netinet/in.h>
#include <string>

#include "core/application/session/slam/slam_session_resource_factory.h"

namespace smartdrone::core::ports {
class ICameraProvider;
class IImuProvider;
class ISlamEngine;
class IVisualFeatureFrontend;
} // namespace smartdrone::core::ports

namespace smartdrone::core::application {

class ImuSensorPoller;
class SlamPreviewOutputRuntime;

struct SlamSessionResourceLifecycleConfig {
    smartdrone::core::ports::ISlamEngine &slamEngine;
    smartdrone::core::ports::ICameraProvider *cameraProvider{nullptr};
    ImuSensorPoller *imuPoller{nullptr};
    SlamPreviewOutputRuntime *previewOutput{nullptr};
    bool useImu{false};
    bool udpEnabled{false};
    const MainRuntimeAliases &aliases;
    std::function<bool(sockaddr_in &)> resolveUdpDestination;
    std::function<SlamVisualFeatureFrontendStartResult()> startVisualFrontend;
    std::function<void(smartdrone::core::ports::IVisualFeatureFrontend *)>
        attachVisualFrontend;
};

struct SlamSessionResourceStartResult {
    bool ok{false};
    bool featureRouteAvailable{false};
    bool featureClientMissing{false};
    bool featureStarted{false};
    std::string featureRepoPath;
    std::string featureError;
    smartdrone::core::ports::IVisualFeatureFrontend *featureFrontend{nullptr};
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

} // namespace smartdrone::core::application
