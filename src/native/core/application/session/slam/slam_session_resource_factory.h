#pragma once

#include <memory>
#include <string>

#include "core/ports/camera_provider.h"
#include "core/ports/imu_provider.h"
#include "core/ports/slam_engine.h"
#include "core/ports/visual_feature_frontend.h"

namespace SmartDrone::core::application {

struct ImuThreadState;
struct MainRuntimeAliases;
struct UnifiedConfig;
class SlamRuntimeControlPort;

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
    std::unique_ptr<SmartDrone::core::ports::ISlamEngine> engine;
    std::unique_ptr<SlamRuntimeControlPort> control;
};

struct SlamVisualFeatureFrontendStartResult {
    bool routeAvailable{false};
    bool clientMissing{false};
    bool started{false};
    std::string repoPath;
    std::string error;
    std::unique_ptr<ISlamVisualFeatureFrontendSession> session;
    SmartDrone::core::ports::IVisualFeatureFrontend *frontend{nullptr};
};

} // namespace SmartDrone::core::application
