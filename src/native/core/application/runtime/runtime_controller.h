#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "core/application/state/live_pose_state.h"
#include "core/application/runtime/mode_manager.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/runtime_command_service.h"
#include "core/domain/runtime_mode.h"

namespace smartdrone::core::application {

class UnifiedRuntimeController final : public IRuntimeCommandTarget {
public:
    using ControllerMode = smartdrone::core::domain::RuntimeMode;
    using SlamSessionRunner = std::function<bool(
        const UnifiedConfig&, LiveRuntimeTuning&, Px4MavlinkGateway&, std::atomic<bool>&, LivePoseState&)>;
    using CalibSessionRunner = std::function<bool(const UnifiedConfig&, std::atomic<bool>&, LivePoseState&)>;
    using CleanupCalibDataFn = std::function<int(const std::string&)>;

    UnifiedRuntimeController(
        UnifiedConfig initialConfig,
        LiveRuntimeTuning& tuning,
        Px4MavlinkGateway& mav,
        LivePoseState& livePose,
        std::atomic<bool>& runningFlag,
        SlamSessionRunner slamSessionRunner,
        CalibSessionRunner calibSessionRunner,
        CleanupCalibDataFn cleanupCalibData);

    void Start();
    void Stop();
    bool SetMode(ControllerMode mode, std::string* err);
    bool UpdateRemoteConfig(const RemoteRuntimeConfig& r, std::string* err);
    bool CleanupCalibData(std::string* msg);
    UnifiedConfig CurrentConfig();
    ControllerMode CurrentDesiredMode();
    CommandResult ExecuteAction(const RuntimeAction& action) override;
    CommandResult ApplyConfig(const ConfigUpdate& update) override;

private:
    void JoinSession();
    void Loop();

    std::mutex m_mu;
    std::condition_variable m_cv;
    UnifiedConfig m_config;
    LiveRuntimeTuning& m_tuning;
    Px4MavlinkGateway& m_mav;
    LivePoseState& m_livePose;
    std::atomic<bool>& m_runningFlag;
    SlamSessionRunner m_slamSessionRunner;
    CalibSessionRunner m_calibSessionRunner;
    CleanupCalibDataFn m_cleanupCalibData;
    ModeManager m_modeManager{};
    bool m_sessionDone{false};
    bool m_stopping{false};
    std::atomic<bool> m_sessionStop{false};
    std::thread m_worker;
    std::thread m_session;
};

}  // namespace smartdrone::core::application
