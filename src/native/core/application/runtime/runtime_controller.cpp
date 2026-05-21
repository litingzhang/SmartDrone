#include "core/application/runtime/runtime_controller.h"

#include <chrono>
#include <csignal>
#include <iostream>
#include <utility>

#include "core/application/session/slam/slam_settings_loader.h"

namespace smartdrone::core::application {

namespace {

constexpr auto kCalibCleanupIdleTimeout = std::chrono::seconds(5);

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
    if (config.app.runtime.slamBackend != SlamBackend::OrbSlam3) {
        return;
    }
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

UnifiedRuntimeController::UnifiedRuntimeController(UnifiedRuntimeControllerConfig config)
    : m_config(std::move(config.initialConfig)), m_tuning(config.tuning),
      m_publishRuntimeMode(std::move(config.publishRuntimeMode)),
      m_cleanupCalibData(std::move(config.cleanupCalibData)),
      m_configService(m_config, m_tuning, m_mu,
                      [this]() {
                          if (m_sessionSupervisor.DesiredMode() != ControllerMode::Idle) {
                              m_sessionSupervisor.RequestRestart();
                          }
                      }),
      m_sessionSupervisor(RuntimeSessionSupervisor::Config{
          config.runningFlag,
          [this]() { return CurrentConfig(); },
          std::move(config.createSession)})
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

void UnifiedRuntimeController::Stop() { m_sessionSupervisor.Stop(); }

bool UnifiedRuntimeController::SetMode(ControllerMode mode, std::string *err)
{
    if (mode != ControllerMode::Idle) {
        std::lock_guard<std::mutex> lock(m_calibCleanupMtx);
        if (m_calibCleanupPending) {
            if (err) {
                *err = "calib clean pending";
            }
            return false;
        }
    }
    if (!m_sessionSupervisor.RequestMode(mode, err)) {
        return false;
    }
    if (m_publishRuntimeMode) {
        m_publishRuntimeMode(mode);
    }
    return true;
}

bool UnifiedRuntimeController::UpdateRemoteConfig(const RemoteRuntimeConfig &r, std::string *err)
{
    return m_configService.UpdateRemoteConfig(r, err);
}

CommandResult UnifiedRuntimeController::RequestCalibCleanup()
{
    {
        std::lock_guard<std::mutex> lock(m_calibCleanupMtx);
        if (m_calibCleanupPending) {
            return {true, "calib clean pending"};
        }
        m_calibCleanupPending = true;
        m_calibCleanupDeadline = std::chrono::steady_clock::now() + kCalibCleanupIdleTimeout;
    }

    const auto status = m_sessionSupervisor.GetIdleStatus();
    if (status.stopping) {
        std::lock_guard<std::mutex> lock(m_calibCleanupMtx);
        m_calibCleanupPending = false;
        return {false, "runtime stopping"};
    }
    if (status.idle) {
        const auto result = RunCalibCleanup();
        std::lock_guard<std::mutex> lock(m_calibCleanupMtx);
        m_calibCleanupPending = false;
        return result;
    }

    std::string err;
    if (!m_sessionSupervisor.RequestMode(ControllerMode::Idle, &err)) {
        std::lock_guard<std::mutex> lock(m_calibCleanupMtx);
        m_calibCleanupPending = false;
        return {false, err.empty() ? "runtime stopping" : err};
    }
    return {true, "calib clean pending"};
}

CommandResult UnifiedRuntimeController::RunCalibCleanup()
{
    std::string root;
    {
        std::lock_guard<std::mutex> lock(m_mu);
        root = m_config.calib.root;
    }
    const int removed = m_cleanupCalibData ? m_cleanupCalibData(root) : 0;
    return {true, "calib clean removed=" + std::to_string(removed)};
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

void UnifiedRuntimeController::OnSessionSupervisorGraphTick()
{
    m_sessionSupervisor.OnGraphTick();
    StepPendingCalibCleanup();
}

void UnifiedRuntimeController::StepSessionSupervisor()
{
    OnSessionSupervisorGraphTick();
}

void UnifiedRuntimeController::StepPendingCalibCleanup()
{
    const auto status = m_sessionSupervisor.GetIdleStatus();
    const auto now = std::chrono::steady_clock::now();
    bool shouldRun = false;
    bool expired = false;
    bool stopping = false;
    {
        std::lock_guard<std::mutex> lock(m_calibCleanupMtx);
        if (!m_calibCleanupPending) {
            return;
        }
        if (status.stopping) {
            m_calibCleanupPending = false;
            stopping = true;
        } else if (status.idle) {
            shouldRun = true;
        } else if (now >= m_calibCleanupDeadline) {
            m_calibCleanupPending = false;
            expired = true;
        }
    }
    if (shouldRun) {
        const auto result = RunCalibCleanup();
        {
            std::lock_guard<std::mutex> lock(m_calibCleanupMtx);
            m_calibCleanupPending = false;
        }
        std::cerr << "[runtime] " << result.message << "\n";
    } else if (expired) {
        std::cerr << "[runtime] calib clean skipped: runtime busy\n";
    } else if (stopping) {
        std::cerr << "[runtime] calib clean skipped: runtime stopping\n";
    }
}

void UnifiedRuntimeController::StepForceRestart()
{
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(m_forceRestartMtx);
        if (m_forceRestartAt.time_since_epoch().count() == 0 || now < m_forceRestartAt) {
            return;
        }
        m_forceRestartAt = std::chrono::steady_clock::time_point{};
    }
    std::raise(SIGKILL);
}

void UnifiedRuntimeController::StepEpgRedeploy(
    EpgRedeployCoordinator &coordinator)
{
    ApplySessionRedeployRequest(coordinator);
}

void UnifiedRuntimeController::ApplySessionRedeployRequest(
    EpgRedeployCoordinator &coordinator)
{
    EpgRedeployRequest request;
    if (!coordinator.TakeSessionRedeployRequest(request)) {
        return;
    }
    if (m_sessionSupervisor.DesiredMode() == ControllerMode::Idle) {
        std::cerr << "[epg] session graph redeploy skipped while idle: "
                  << DescribeEpgRedeployRequest(request) << "\n";
        return;
    }
    m_sessionSupervisor.RequestRestart();
    std::cerr << "[epg] session graph redeploy requested: "
              << DescribeEpgRedeployRequest(request) << "\n";
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
        return RequestCalibCleanup();
    }
    case RuntimeAction::Type::ForceRestart:
        {
            std::lock_guard<std::mutex> lock(m_forceRestartMtx);
            m_forceRestartAt = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);
        }
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
