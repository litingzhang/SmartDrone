#pragma once

#include <functional>
#include <mutex>
#include <string>

#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/runtime_command_service.h"

namespace SmartDrone::core::application {

class RuntimeConfigService {
  public:
    using RestartFn = std::function<void()>;

    RuntimeConfigService(UnifiedConfig &config, LiveRuntimeTuning &tuning, std::mutex &configMutex,
                         RestartFn requestRestart);

    bool UpdateRemoteConfig(RemoteRuntimeConfig remote, std::string *err);
    CommandResult ApplyConfig(const ConfigUpdate &update, const UnifiedConfig &currentConfig);

  private:
    static RemoteRuntimeConfig BuildRemoteConfig(const UnifiedConfig &currentConfig);

    UnifiedConfig &m_config;
    LiveRuntimeTuning &m_tuning;
    std::mutex &m_configMutex;
    RestartFn m_requestRestart;
};

} // namespace SmartDrone::core::application
