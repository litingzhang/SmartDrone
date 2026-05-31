#include "core/application/runtime/runtime_controller.h"

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <utility>

#include "core/application/runtime/runtime_config_application.h"
#include "core/application/runtime/runtime_config_projection.h"
#include "core/application/session/slam/slam_settings_loader.h"

namespace SmartDrone::Core::Application {

namespace {

constexpr auto CALIB_CLEANUP_IDLE_TIMEOUT = std::chrono::seconds(5);

std::int64_t SteadyNowMs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}

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
    : m_config(std::make_shared<const UnifiedConfig>(std::move(config.initialConfig))),
      m_tuning(config.tuning),
      m_publishRuntimeMode(std::move(config.publishRuntimeMode)),
      m_cleanupCalibData(std::move(config.cleanupCalibData)),
      m_configService(m_config, m_tuning,
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
    UnifiedConfig initialConfig = CurrentConfig();
    SyncDefaultTbcFromSettings(initialConfig);
    SyncDefaultOrbFromSettings(initialConfig);
    std::atomic_store_explicit(&m_config,
                               std::make_shared<const UnifiedConfig>(std::move(initialConfig)),
                               std::memory_order_release);
    const UnifiedConfig runtimeConfig = CurrentConfig();

    const RemoteRuntimeConfig remote = BuildRemoteConfig(runtimeConfig);
    const RuntimeTbcValues tbc{
        runtimeConfig.app.runtime.tbcTx,
        runtimeConfig.app.runtime.tbcTy,
        runtimeConfig.app.runtime.tbcTz,
        runtimeConfig.app.runtime.tbcRollDeg,
        runtimeConfig.app.runtime.tbcPitchDeg,
        runtimeConfig.app.runtime.tbcYawDeg};
    SyncRuntimeTuning(m_tuning, remote, tbc);
}

void UnifiedRuntimeController::Stop()
{
    m_sessionSupervisor.Stop();
}

bool UnifiedRuntimeController::SetMode(ControllerMode mode, std::string *err)
{
    if (mode != ControllerMode::Idle) {
        if (CalibCleanupPending()) {
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
    if (!TryScheduleCalibCleanup()) {
        return {true, "calib clean pending"};
    }

    const auto status = m_sessionSupervisor.GetIdleStatus();
    if (status.stopping) {
        m_calibCleanupDeadlineMs.store(0, std::memory_order_release);
        return {false, "runtime stopping"};
    }
    if (status.idle) {
        const std::int64_t deadlineMs =
            m_calibCleanupDeadlineMs.load(std::memory_order_acquire);
        const bool taken = TakePendingCalibCleanup(deadlineMs);
        const auto result =
            taken ? RunCalibCleanup() : CommandResult{true, "calib clean pending"};
        return result;
    }

    std::string err;
    if (!m_sessionSupervisor.RequestMode(ControllerMode::Idle, &err)) {
        m_calibCleanupDeadlineMs.store(0, std::memory_order_release);
        return {false, err.empty() ? "runtime stopping" : err};
    }
    return {true, "calib clean pending"};
}

CommandResult UnifiedRuntimeController::RunCalibCleanup()
{
    const std::string root = CurrentConfig().calib.root;
    const int removed = m_cleanupCalibData ? m_cleanupCalibData(root) : 0;
    return {true, "calib clean removed=" + std::to_string(removed)};
}

UnifiedConfig UnifiedRuntimeController::CurrentConfig()
{
    std::shared_ptr<const UnifiedConfig> config =
        std::atomic_load_explicit(&m_config, std::memory_order_acquire);
    return config ? *config : UnifiedConfig{};
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
    const std::int64_t deadlineMs =
        m_calibCleanupDeadlineMs.load(std::memory_order_acquire);
    if (deadlineMs == 0) {
        return;
    }
    bool expired = false;
    bool stopping = false;
    if (status.stopping) {
        stopping = TakePendingCalibCleanup(deadlineMs);
    } else if (status.idle) {
        if (!TakePendingCalibCleanup(deadlineMs)) {
            return;
        }
        const auto result = RunCalibCleanup();
        std::cerr << "[runtime] " << result.message << "\n";
        return;
    } else if (SteadyNowMs() >= deadlineMs) {
        expired = TakePendingCalibCleanup(deadlineMs);
    }
    if (expired) {
        std::cerr << "[runtime] calib clean skipped: runtime busy\n";
    } else if (stopping) {
        std::cerr << "[runtime] calib clean skipped: runtime stopping\n";
    }
}

void UnifiedRuntimeController::StepForceRestart()
{
    const std::int64_t restartAt =
        m_forceRestartAtMs.load(std::memory_order_acquire);
    if (restartAt == 0 || SteadyNowMs() < restartAt) {
        return;
    }
    if (m_forceRestartAtMs.exchange(0, std::memory_order_acq_rel) == 0) {
        return;
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
        return {true, std::string("runtime -> ") + SmartDrone::Core::Domain::ToString(action.selection.runtimeMode)};
    case RuntimeAction::Type::StopRuntime:
        if (!SetMode(ControllerMode::Idle, &err)) {
            return {false, err.empty() ? "stop runtime failed" : err};
        }
        return {true, "runtime -> idle"};
    case RuntimeAction::Type::CleanCalibration: {
        return RequestCalibCleanup();
    }
    case RuntimeAction::Type::ForceRestart: {
        m_forceRestartAtMs.store(SteadyNowMs() + 150,
                                 std::memory_order_release);
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

bool UnifiedRuntimeController::CalibCleanupPending() const
{
    return m_calibCleanupDeadlineMs.load(std::memory_order_acquire) != 0;
}

bool UnifiedRuntimeController::TryScheduleCalibCleanup()
{
    std::int64_t expected = 0;
    const auto timeoutMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            CALIB_CLEANUP_IDLE_TIMEOUT)
            .count();
    return m_calibCleanupDeadlineMs.compare_exchange_strong(
        expected,
        SteadyNowMs() + timeoutMs,
        std::memory_order_acq_rel,
        std::memory_order_acquire);
}

bool UnifiedRuntimeController::TakePendingCalibCleanup(
    std::int64_t deadlineMs)
{
    if (deadlineMs == 0) {
        return false;
    }
    return m_calibCleanupDeadlineMs.compare_exchange_strong(
        deadlineMs,
        0,
        std::memory_order_acq_rel,
        std::memory_order_acquire);
}

} // namespace SmartDrone::Core::Application
