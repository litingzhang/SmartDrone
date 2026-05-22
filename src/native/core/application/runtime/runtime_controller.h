#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>

#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/runtime_command_service.h"
#include "core/application/runtime/runtime_config_service.h"
#include "core/application/runtime/epg_redeploy_coordinator.h"
#include "core/application/runtime/runtime_session_supervisor.h"
#include "core/domain/runtime_mode.h"

namespace SmartDrone::core::application {

using PublishRuntimeModeFn = std::function<void(domain::RuntimeMode)>;

struct UnifiedRuntimeControllerConfig {
    UnifiedConfig initialConfig;
    LiveRuntimeTuning &tuning;
    std::atomic<bool> &runningFlag;
    RuntimeSessionSupervisor::CreateSessionFn createSession;
    PublishRuntimeModeFn publishRuntimeMode;
    std::function<int(const std::string &)> cleanupCalibData;
};

class UnifiedRuntimeController final : public IRuntimeCommandTarget {
  public:
    using ControllerMode = SmartDrone::core::domain::RuntimeMode;
    using CleanupCalibDataFn = std::function<int(const std::string &)>;

    explicit UnifiedRuntimeController(UnifiedRuntimeControllerConfig config);

    void Stop();
    bool SetMode(ControllerMode mode, std::string *err);
    bool UpdateRemoteConfig(const RemoteRuntimeConfig &r, std::string *err);
    UnifiedConfig CurrentConfig();
    ControllerMode CurrentDesiredMode();
    void OnSessionSupervisorGraphTick();
    void StepSessionSupervisor();
    void StepForceRestart();
    void StepEpgRedeploy(EpgRedeployCoordinator &coordinator);
    CommandResult ExecuteAction(const RuntimeAction &action) override;
    CommandResult ApplyConfig(const ConfigUpdate &update) override;

  private:
    UnifiedConfig CurrentConfigUnlocked() const;
    CommandResult RequestCalibCleanup();
    CommandResult RunCalibCleanup();
    void StepPendingCalibCleanup();
    void ApplySessionRedeployRequest(EpgRedeployCoordinator &coordinator);

    UnifiedConfig m_config;
    LiveRuntimeTuning &m_tuning;
    PublishRuntimeModeFn m_publishRuntimeMode;
    CleanupCalibDataFn m_cleanupCalibData;
    mutable std::mutex m_mu;
    RuntimeConfigService m_configService;
    RuntimeSessionSupervisor m_sessionSupervisor;
    mutable std::mutex m_forceRestartMtx;
    std::chrono::steady_clock::time_point m_forceRestartAt{};
    std::mutex m_calibCleanupMtx;
    bool m_calibCleanupPending{false};
    std::chrono::steady_clock::time_point m_calibCleanupDeadline{};
};

} // namespace SmartDrone::core::application
