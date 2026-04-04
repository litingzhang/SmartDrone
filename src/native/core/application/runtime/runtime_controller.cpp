#include "core/application/runtime/runtime_controller.h"

#include <chrono>
#include <csignal>
#include <utility>

#include "common/thread_launch.h"

namespace smartdrone::core::application {

UnifiedRuntimeController::UnifiedRuntimeController(UnifiedConfig initialConfig, LiveRuntimeTuning &tuning,
                                                   Px4MavlinkGateway &mav, LivePoseState &livePose,
                                                   std::atomic<bool> &runningFlag, SlamSessionRunner slamSessionRunner,
                                                   CalibSessionRunner calibSessionRunner,
                                                   CleanupCalibDataFn cleanupCalibData)
    : m_config(std::move(initialConfig)), m_tuning(tuning), m_mav(mav), m_livePose(livePose),
      m_cleanupCalibData(std::move(cleanupCalibData)),
      m_configService(m_config, m_tuning, m_mu,
                      [this]() {
                          if (m_sessionSupervisor.DesiredMode() != ControllerMode::Idle) {
                              m_sessionSupervisor.RequestRestart();
                          }
                      }),
      m_sessionSupervisor(
          runningFlag, tuning, mav, livePose, [this]() { return CurrentConfig(); }, std::move(slamSessionRunner),
          std::move(calibSessionRunner))
{
    m_tuning.slamInputFps.store(m_config.app.runtime.slamInputFps, std::memory_order_relaxed);
    m_tuning.slamOperationMode.store(static_cast<uint8_t>(m_config.app.runtime.slamOperationMode),
                                     std::memory_order_relaxed);
    m_tuning.sendImage.store(m_config.app.udp.sendImage, std::memory_order_relaxed);
    m_tuning.sendFeature.store(m_config.app.udp.sendFeature, std::memory_order_relaxed);
    m_tuning.sendMap.store(m_config.app.udp.sendMap, std::memory_order_relaxed);
}

void UnifiedRuntimeController::Start() { m_sessionSupervisor.Start(); }

void UnifiedRuntimeController::Stop() { m_sessionSupervisor.Stop(); }

bool UnifiedRuntimeController::SetMode(ControllerMode mode, std::string *err)
{
    if (!m_sessionSupervisor.RequestMode(mode, err)) {
        return false;
    }
    m_livePose.SetRuntimeMode(static_cast<uint8_t>(mode));
    return true;
}

bool UnifiedRuntimeController::UpdateRemoteConfig(const RemoteRuntimeConfig &r, std::string *err)
{
    return m_configService.UpdateRemoteConfig(r, err);
}

bool UnifiedRuntimeController::CleanupCalibData(std::string *msg)
{
    if (m_sessionSupervisor.ActiveMode() != ControllerMode::Idle ||
        m_sessionSupervisor.DesiredMode() != ControllerMode::Idle) {
        std::string err;
        if (!m_sessionSupervisor.RequestMode(ControllerMode::Idle, &err)) {
            if (msg) {
                *msg = err.empty() ? "runtime stopping" : err;
            }
            return false;
        }
        bool stopping = false;
        if (!m_sessionSupervisor.WaitForIdle(std::chrono::seconds(5), &stopping)) {
            if (msg) {
                *msg = stopping ? "runtime stopping" : "runtime busy";
            }
            return false;
        }
    }

    const int removed = m_cleanupCalibData ? m_cleanupCalibData(m_config.calib.root) : 0;
    if (msg) {
        *msg = "calib clean removed=" + std::to_string(removed);
    }
    return true;
}

UnifiedConfig UnifiedRuntimeController::CurrentConfig()
{
    std::lock_guard<std::mutex> lock(m_mu);
    return CurrentConfigUnlocked();
}

UnifiedRuntimeController::ControllerMode UnifiedRuntimeController::CurrentDesiredMode()
{
    return m_sessionSupervisor.DesiredMode();
}

CommandResult UnifiedRuntimeController::ExecuteAction(const RuntimeAction &action)
{
    std::string err;
    switch (action.type) {
    case RuntimeAction::Type::StartRuntime:
        if (!SetMode(action.selection.runtimeMode, &err)) {
            return {false, err.empty() ? "start runtime failed" : err};
        }
        return {true, std::string("runtime -> ") + smartdrone::core::domain::ToString(action.selection.runtimeMode)};
    case RuntimeAction::Type::StopRuntime:
        if (!SetMode(ControllerMode::Idle, &err)) {
            return {false, err.empty() ? "stop runtime failed" : err};
        }
        return {true, "runtime -> idle"};
    case RuntimeAction::Type::CleanCalibration: {
        std::string msg;
        if (!CleanupCalibData(&msg)) {
            return {false, msg.empty() ? "calib clean failed" : msg};
        }
        return {true, msg};
    }
    case RuntimeAction::Type::ForceRestart:
        SMARTDRONE_START_DETACHED_THREAD(smartdrone::common::ThreadRole::RuntimeForceRestart,
                                         "UnifiedRuntimeController", []() {
                                             std::this_thread::sleep_for(std::chrono::milliseconds(150));
                                             std::raise(SIGKILL);
                                         });
        return {true, "service restart scheduled"};
    case RuntimeAction::Type::ResetMap:
        return {false, "reset map not implemented"};
    case RuntimeAction::Type::SaveMap:
        return {false, "save map not implemented"};
    default:
        return {false, "unknown action"};
    }
}

CommandResult UnifiedRuntimeController::ApplyConfig(const ConfigUpdate &update)
{
    return m_configService.ApplyConfig(update, CurrentConfig());
}

UnifiedConfig UnifiedRuntimeController::CurrentConfigUnlocked() const { return m_config; }

} // namespace smartdrone::core::application
