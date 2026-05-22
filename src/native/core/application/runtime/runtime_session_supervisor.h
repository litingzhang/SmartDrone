#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/mode_manager.h"

namespace SmartDrone::Core::Application {

class ISessionGraphRuntime;

class RuntimeSessionSupervisor {
  public:
    using ControllerMode = SmartDrone::Core::Domain::RuntimeMode;
    using CurrentConfigFn = std::function<UnifiedConfig()>;

    struct IdleStatus {
        bool idle{false};
        bool stopping{false};
    };

    struct SessionStartRequest {
        ControllerMode mode{ControllerMode::Idle};
        const UnifiedConfig &cfg;
        std::atomic<bool> &stop;
        std::atomic<bool> &runningFlag;
    };

    using CreateSessionFn = std::function<std::unique_ptr<ISessionGraphRuntime>(const SessionStartRequest &)>;

    struct Config {
        std::atomic<bool> &runningFlag;
        CurrentConfigFn currentConfig;
        CreateSessionFn createSession;
    };

    explicit RuntimeSessionSupervisor(Config config);
    ~RuntimeSessionSupervisor();

    void Stop();
    void OnGraphTick();
    void Step();
    bool RequestMode(ControllerMode mode, std::string *err);
    void RequestRestart();
    ControllerMode DesiredMode() const;
    ControllerMode ActiveMode() const;
    IdleStatus GetIdleStatus() const;

  private:
    void ApplyGlobalStop();
    void StopRequestedSession();
    void StepActiveSession();
    void FinishCompletedSession();
    void LaunchRequestedSession();
    void StopActiveSessionSynchronously();
    void MarkSessionJoined();
    void StepSupervisor();
    bool PrepareLaunch(ControllerMode &mode, UnifiedConfig &cfg);
    bool SessionIdleUnlocked() const;
    std::shared_ptr<ISessionGraphRuntime> MakeSessionRuntime(ControllerMode mode, const UnifiedConfig &cfg);

    std::atomic<bool> &m_runningFlag;
    CurrentConfigFn m_currentConfig;
    CreateSessionFn m_createSession;

    std::mutex m_stepMu;
    mutable std::mutex m_mu;
    ModeManager m_modeManager{};
    bool m_stopping{false};
    std::atomic<bool> m_sessionStop{false};
    std::shared_ptr<ISessionGraphRuntime> m_session;
};

} // namespace SmartDrone::Core::Application
