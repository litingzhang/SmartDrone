#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/runtime_command_service.h"
#include "core/application/runtime/runtime_config_service.h"
#include "core/application/runtime/epg_redeploy_coordinator.h"
#include "core/application/runtime/runtime_session_supervisor.h"
#include "core/domain/runtime_mode.h"

namespace SmartDrone::Core::Application {

using PublishRuntimeModeFn = std::function<void(Domain::RuntimeMode)>;

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
    using ControllerMode = SmartDrone::Core::Domain::RuntimeMode;
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
    CommandResult RequestCalibCleanup();
    CommandResult RunCalibCleanup();
    void StepPendingCalibCleanup();
    bool CalibCleanupPending() const;
    bool TryScheduleCalibCleanup();
    bool TakePendingCalibCleanup(std::int64_t deadlineMs);
    void ApplySessionRedeployRequest(EpgRedeployCoordinator &coordinator);

    std::shared_ptr<const UnifiedConfig> m_config;
    LiveRuntimeTuning &m_tuning;
    PublishRuntimeModeFn m_publishRuntimeMode;
    CleanupCalibDataFn m_cleanupCalibData;
    RuntimeConfigService m_configService;
    RuntimeSessionSupervisor m_sessionSupervisor;
    std::atomic<std::int64_t> m_forceRestartAtMs{0};
    std::atomic<std::int64_t> m_calibCleanupDeadlineMs{0};
};

} // namespace SmartDrone::Core::Application
