#pragma once

#include <memory>
#include <string>

#include "core/ports/camera_provider.h"
#include "core/ports/imu_provider.h"
#include "core/ports/slam_engine.h"
#include "core/ports/visual_feature_frontend.h"

namespace SmartDrone::Core::Application {

struct ImuThreadState;
struct MainRuntimeAliases;
struct UnifiedConfig;
class SlamRuntimeControlPort;

} // namespace SmartDrone::Core::Application

namespace SmartDrone::Core::Ports {
class ISlamBackendMaintenance;
} // namespace SmartDrone::Core::Ports

namespace SmartDrone::Core::Application {

class ISlamVisualFeatureFrontendSession {
  public:
    virtual ~ISlamVisualFeatureFrontendSession() = default;

    virtual void Stop() = 0;
};

struct SlamSessionEngineResourceConfig {
    const UnifiedConfig &cfg;
    const MainRuntimeAliases &aliases;
    const std::string &settingsPath;
    bool useImu{false};
};

struct SlamSessionEngineResources {
    std::unique_ptr<SmartDrone::Core::Ports::ISlamEngine> engine;
    std::unique_ptr<SlamRuntimeControlPort> control;
    SmartDrone::Core::Ports::ISlamBackendMaintenance *backendMaintenance{
        nullptr};
};

struct SlamVisualFeatureFrontendStartResult {
    bool routeAvailable{false};
    bool clientMissing{false};
    bool started{false};
    std::string repoPath;
    std::string error;
    std::unique_ptr<ISlamVisualFeatureFrontendSession> session;
    SmartDrone::Core::Ports::IVisualFeatureFrontend *frontend{nullptr};
};

} // namespace SmartDrone::Core::Application
