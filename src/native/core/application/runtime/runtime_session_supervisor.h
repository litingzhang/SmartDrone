#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/mode_manager.h"
#include "core/application/state/live_pose_state.h"

namespace smartdrone::core::application {

class RuntimeSessionSupervisor {
  public:
    using ControllerMode = smartdrone::core::domain::RuntimeMode;
    using CurrentConfigFn = std::function<UnifiedConfig()>;
    using SlamSessionRunner = std::function<bool(const UnifiedConfig &, LiveRuntimeTuning &, Px4MavlinkGateway &,
                                                 std::atomic<bool> &, LivePoseState &)>;
    using CalibSessionRunner = std::function<bool(const UnifiedConfig &, std::atomic<bool> &, LivePoseState &)>;

    RuntimeSessionSupervisor(std::atomic<bool> &runningFlag, LiveRuntimeTuning &tuning, Px4MavlinkGateway &mav,
                             LivePoseState &livePose, CurrentConfigFn currentConfig,
                             SlamSessionRunner slamSessionRunner, CalibSessionRunner calibSessionRunner);

    void Start();
    void Stop();
    bool RequestMode(ControllerMode mode, std::string *err);
    void RequestRestart();
    ControllerMode DesiredMode() const;
    ControllerMode ActiveMode() const;
    bool WaitForIdle(std::chrono::milliseconds timeout, bool *stoppingOut = nullptr);

  private:
    void JoinSession();
    void Loop();

    std::atomic<bool> &m_runningFlag;
    LiveRuntimeTuning &m_tuning;
    Px4MavlinkGateway &m_mav;
    LivePoseState &m_livePose;
    CurrentConfigFn m_currentConfig;
    SlamSessionRunner m_slamSessionRunner;
    CalibSessionRunner m_calibSessionRunner;

    mutable std::mutex m_mu;
    std::condition_variable m_cv;
    ModeManager m_modeManager{};
    bool m_sessionDone{false};
    bool m_stopping{false};
    std::atomic<bool> m_sessionStop{false};
    std::thread m_worker;
    std::thread m_session;
};

} // namespace smartdrone::core::application
