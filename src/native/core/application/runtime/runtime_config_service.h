#pragma once

#include <functional>
#include <memory>
#include <string>

#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/runtime_command_service.h"

namespace SmartDrone::Core::Application {

class RuntimeConfigService {
  public:
    using RestartFn = std::function<void()>;

    RuntimeConfigService(std::shared_ptr<const UnifiedConfig> &config,
                         LiveRuntimeTuning &tuning,
                         RestartFn requestRestart);

    bool UpdateRemoteConfig(RemoteRuntimeConfig remote, std::string *err);
    CommandResult ApplyConfig(const ConfigUpdate &update, const UnifiedConfig &currentConfig);

  private:
    static RemoteRuntimeConfig BuildRemoteConfig(const UnifiedConfig &currentConfig);
    std::shared_ptr<const UnifiedConfig> LoadConfig() const;
    bool ReplaceConfig(std::shared_ptr<const UnifiedConfig> &expected,
                       std::shared_ptr<const UnifiedConfig> next);

    std::shared_ptr<const UnifiedConfig> &m_config;
    LiveRuntimeTuning &m_tuning;
    RestartFn m_requestRestart;
};

} // namespace SmartDrone::Core::Application
