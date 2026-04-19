#include "core/application/runtime/runtime_controller.h"

#include <chrono>
#include <csignal>
#include <utility>

#include "common/thread_launch.h"
#include "core/application/session/runtime_session_common.h"

namespace smartdrone::core::application {

namespace {

void SyncDefaultTbcFromSettings(UnifiedConfig &config)
{
    if (config.app.runtime.useCustomTbc) {
        return;
    }

    const auto extrinsics = LoadStereoBodyExtrinsics(config.app.settings);
    if (!extrinsics.loaded) {
        return;
    }

    const Eigen::Vector3f t = extrinsics.Tbc.translation();
    config.app.runtime.tbcTx = t.x();
    config.app.runtime.tbcTy = t.y();
    config.app.runtime.tbcTz = t.z();
    // Runtime override now means "dynamic pitch on top of calibrated T_b_c1", so
    // the neutral UI state is zero delta instead of the calibrated absolute pose.
    config.app.runtime.tbcRollDeg = 0.0f;
    config.app.runtime.tbcPitchDeg = 0.0f;
    config.app.runtime.tbcYawDeg = 0.0f;
}

void SyncDefaultOrbFromSettings(UnifiedConfig &config)
{
    if (config.app.runtime.orbNFeatures > 0 && config.app.runtime.orbScaleFactor > 0.0f &&
        config.app.runtime.orbNLevels > 0 && config.app.runtime.orbIniThFAST > 0 &&
        config.app.runtime.orbMinThFAST > 0) {
        return;
    }

    const auto orb = LoadOrbExtractorSettings(config.app.settings);
    if (!orb.loaded) {
        config.app.runtime.orbNFeatures = 1200;
        config.app.runtime.orbScaleFactor = 1.2f;
        config.app.runtime.orbNLevels = 8;
        config.app.runtime.orbIniThFAST = 16;
        config.app.runtime.orbMinThFAST = 6;
        return;
    }

    config.app.runtime.orbNFeatures = orb.nFeatures;
    config.app.runtime.orbScaleFactor = orb.scaleFactor;
    config.app.runtime.orbNLevels = orb.nLevels;
    config.app.runtime.orbIniThFAST = orb.iniThFAST;
    config.app.runtime.orbMinThFAST = orb.minThFAST;
}

} // namespace

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
    SyncDefaultTbcFromSettings(m_config);
    SyncDefaultOrbFromSettings(m_config);

    m_tuning.slamInputFps.store(m_config.app.runtime.slamInputFps, std::memory_order_relaxed);
    m_tuning.slamOperationMode.store(static_cast<uint8_t>(m_config.app.runtime.slamOperationMode),
                                     std::memory_order_relaxed);
    m_tuning.featureFrontend.store(static_cast<uint8_t>(m_config.app.runtime.featureFrontend),
                                   std::memory_order_relaxed);
    m_tuning.sendImage.store(m_config.app.udp.sendImage, std::memory_order_relaxed);
    m_tuning.sendFeature.store(m_config.app.udp.sendFeature, std::memory_order_relaxed);
    m_tuning.sendMap.store(m_config.app.udp.sendMap, std::memory_order_relaxed);
    m_tuning.useCustomTbc.store(m_config.app.runtime.useCustomTbc, std::memory_order_relaxed);
    m_tuning.tbcTx.store(m_config.app.runtime.tbcTx, std::memory_order_relaxed);
    m_tuning.tbcTy.store(m_config.app.runtime.tbcTy, std::memory_order_relaxed);
    m_tuning.tbcTz.store(m_config.app.runtime.tbcTz, std::memory_order_relaxed);
    m_tuning.tbcRollDeg.store(m_config.app.runtime.tbcRollDeg, std::memory_order_relaxed);
    m_tuning.tbcPitchDeg.store(m_config.app.runtime.tbcPitchDeg, std::memory_order_relaxed);
    m_tuning.tbcYawDeg.store(m_config.app.runtime.tbcYawDeg, std::memory_order_relaxed);
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
        smartdrone::common::StartDetachedThread(
            smartdrone::common::MakeThreadLaunchInfo(smartdrone::common::ThreadRole::RuntimeForceRestart,
                                                     "UnifiedRuntimeController"),
            []() {
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
