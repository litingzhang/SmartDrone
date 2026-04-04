#include "core/application/runtime/runtime_controller.h"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <utility>
#include <variant>

#include "common/thread_launch.h"
#include "core/application/config/app_args.h"
#include "core/application/config/config_registry.h"

namespace smartdrone::core::application {

UnifiedRuntimeController::UnifiedRuntimeController(
    UnifiedConfig initialConfig,
    LiveRuntimeTuning& tuning,
    Px4MavlinkGateway& mav,
    LivePoseState& livePose,
    std::atomic<bool>& runningFlag,
    SlamSessionRunner slamSessionRunner,
    CalibSessionRunner calibSessionRunner,
    CleanupCalibDataFn cleanupCalibData)
    : m_config(std::move(initialConfig)),
      m_tuning(tuning),
      m_mav(mav),
      m_livePose(livePose),
      m_runningFlag(runningFlag),
      m_slamSessionRunner(std::move(slamSessionRunner)),
      m_calibSessionRunner(std::move(calibSessionRunner)),
      m_cleanupCalibData(std::move(cleanupCalibData))
{
    m_tuning.slamInputFps.store(m_config.app.runtime.slamInputFps, std::memory_order_relaxed);
    m_tuning.slamOperationMode.store(
        static_cast<uint8_t>(m_config.app.runtime.slamOperationMode),
        std::memory_order_relaxed);
    m_tuning.sendImage.store(m_config.app.udp.sendImage, std::memory_order_relaxed);
    m_tuning.sendFeature.store(m_config.app.udp.sendFeature, std::memory_order_relaxed);
    m_tuning.sendMap.store(m_config.app.udp.sendMap, std::memory_order_relaxed);
}

void UnifiedRuntimeController::Start()
{
    m_worker = SMARTDRONE_START_THREAD(
        smartdrone::common::ThreadRole::RuntimeWorker,
        "UnifiedRuntimeController",
        [this]() { Loop(); });
}

void UnifiedRuntimeController::Stop()
{
    {
        std::lock_guard<std::mutex> lock(m_mu);
        m_stopping = true;
        m_modeManager.RequestMode(ControllerMode::Idle);
        m_sessionStop.store(true);
    }
    m_cv.notify_all();
    if (m_worker.joinable()) {
        m_worker.join();
    }
    JoinSession();
}

bool UnifiedRuntimeController::SetMode(ControllerMode mode, std::string* err)
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (m_stopping) {
        if (err) {
            *err = "runtime stopping";
        }
        return false;
    }
    m_modeManager.RequestMode(mode);
    m_livePose.SetRuntimeMode(static_cast<uint8_t>(mode));
    m_cv.notify_all();
    return true;
}

bool UnifiedRuntimeController::UpdateRemoteConfig(const RemoteRuntimeConfig& r, std::string* err)
{
    if (r.exposureUs <= 0 || !(r.gain > 0.0f) || r.pairMs <= 0 || r.slamInputFps < 0) {
        if (err) {
            *err = "bad runtime config";
        }
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mu);
    CameraConfig& cam = m_config.app.camera;
    const bool sensorModeChanged = m_config.app.sensorMode != r.sensorMode;
    const bool udpIpChanged = m_config.app.udp.ip != r.udpIp;
    const bool udpEnableChanged = m_config.app.udp.enable != r.udpEnabled;
    cam.exposureUs = r.exposureUs;
    cam.gain = r.gain;
    cam.pairMs = r.pairMs;
    m_config.app.runtime.slamInputFps = r.slamInputFps;
    m_config.app.runtime.slamOperationMode = r.slamOperationMode;
    m_config.app.sensorMode = r.sensorMode;
    m_config.app.settings = ResolveSettingsForSensorMode(r.sensorMode, m_config.app.settings);
    m_config.app.udp.ip = r.udpIp;
    m_config.app.udp.enable = r.udpEnabled;
    m_config.app.udp.sendImage = r.sendImage;
    m_config.app.udp.sendFeature = r.sendFeature;
    m_config.app.udp.sendMap = r.sendMap;
    m_tuning.slamInputFps.store(r.slamInputFps, std::memory_order_relaxed);
    m_tuning.slamOperationMode.store(static_cast<uint8_t>(r.slamOperationMode), std::memory_order_relaxed);
    m_tuning.sendImage.store(r.sendImage, std::memory_order_relaxed);
    m_tuning.sendFeature.store(r.sendFeature, std::memory_order_relaxed);
    m_tuning.sendMap.store(r.sendMap, std::memory_order_relaxed);
    const bool restartNeeded = sensorModeChanged || udpIpChanged || udpEnableChanged;
    if (restartNeeded && m_modeManager.DesiredMode() != ControllerMode::Idle) {
        m_modeManager.RequestRestart();
        m_sessionStop.store(true);
    }
    m_cv.notify_all();
    return true;
}

bool UnifiedRuntimeController::CleanupCalibData(std::string* msg)
{
    {
        std::unique_lock<std::mutex> lock(m_mu);
        if (m_stopping) {
            if (msg) {
                *msg = "runtime stopping";
            }
            return false;
        }

        if (m_modeManager.ActiveMode() != ControllerMode::Idle ||
            m_modeManager.DesiredMode() != ControllerMode::Idle ||
            m_session.joinable()) {
            m_modeManager.RequestMode(ControllerMode::Idle);
            m_sessionStop.store(true);
            m_cv.notify_all();
            const bool idleReady = m_cv.wait_for(lock, std::chrono::seconds(5), [this]() {
                return m_stopping ||
                       (m_modeManager.ActiveMode() == ControllerMode::Idle &&
                        m_modeManager.DesiredMode() == ControllerMode::Idle &&
                        !m_session.joinable() &&
                        !m_sessionDone);
            });
            if (!idleReady) {
                if (msg) {
                    *msg = "runtime busy";
                }
                return false;
            }
            if (m_stopping) {
                if (msg) {
                    *msg = "runtime stopping";
                }
                return false;
            }
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
    return m_config;
}

UnifiedRuntimeController::ControllerMode UnifiedRuntimeController::CurrentDesiredMode()
{
    std::lock_guard<std::mutex> lock(m_mu);
    return m_modeManager.DesiredMode();
}

CommandResult UnifiedRuntimeController::ExecuteAction(const RuntimeAction& action)
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
        case RuntimeAction::Type::CleanCalibration:
        {
            std::string msg;
            if (!CleanupCalibData(&msg)) {
                return {false, msg.empty() ? "calib clean failed" : msg};
            }
            return {true, msg};
        }
        case RuntimeAction::Type::ForceRestart:
            SMARTDRONE_START_DETACHED_THREAD(
                smartdrone::common::ThreadRole::RuntimeForceRestart,
                "UnifiedRuntimeController",
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

CommandResult UnifiedRuntimeController::ApplyConfig(const ConfigUpdate& update)
{
    UnifiedConfig current = CurrentConfig();
    RemoteRuntimeConfig remote{};
    remote.exposureUs = current.app.camera.exposureUs;
    remote.gain = current.app.camera.gain;
    remote.pairMs = current.app.camera.pairMs > 0 ? current.app.camera.pairMs : 1;
    remote.slamInputFps = current.app.runtime.slamInputFps;
    remote.slamOperationMode = current.app.runtime.slamOperationMode;
    remote.sensorMode = current.app.sensorMode;
    remote.udpIp = current.app.udp.ip;
    remote.udpEnabled = current.app.udp.enable;
    remote.sendImage = current.app.udp.sendImage;
    remote.sendFeature = current.app.udp.sendFeature;
    remote.sendMap = current.app.udp.sendMap;

    for (const auto& [key, value] : update.values) {
        if (key == ConfigRegistry::kCameraExposureUs) {
            if (const auto* v = std::get_if<int64_t>(&value)) {
                remote.exposureUs = static_cast<int>(*v);
            } else {
                return {false, "camera.exposure_us type mismatch"};
            }
        } else if (key == ConfigRegistry::kCameraGain) {
            if (const auto* v = std::get_if<double>(&value)) {
                remote.gain = static_cast<float>(*v);
            } else if (const auto* vInt = std::get_if<int64_t>(&value)) {
                remote.gain = static_cast<float>(*vInt);
            } else {
                return {false, "camera.gain type mismatch"};
            }
        } else if (key == ConfigRegistry::kCameraPairWindowMs) {
            if (const auto* v = std::get_if<int64_t>(&value)) {
                remote.pairMs = static_cast<int>(*v);
            } else {
                return {false, "camera.pair_window_ms type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamInputFps) {
            if (const auto* v = std::get_if<int64_t>(&value)) {
                remote.slamInputFps = static_cast<int>(*v);
            } else {
                return {false, "slam.input_fps type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamOperationMode) {
            if (const auto* v = std::get_if<std::string>(&value)) {
                remote.slamOperationMode = ParseSlamOperationModeText(*v);
            } else {
                return {false, "slam.operation_mode type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamPerceptionMode) {
            if (const auto* v = std::get_if<std::string>(&value)) {
                remote.sensorMode = ParseSensorModeText(*v);
            } else {
                return {false, "slam.perception_mode type mismatch"};
            }
        } else if (key == ConfigRegistry::kStreamUdpEnabled) {
            if (const auto* v = std::get_if<bool>(&value)) {
                remote.udpEnabled = *v;
            } else {
                return {false, "stream.udp_enabled type mismatch"};
            }
        } else if (key == ConfigRegistry::kStreamUdpIp) {
            if (const auto* v = std::get_if<std::string>(&value)) {
                remote.udpIp = *v;
            } else {
                return {false, "stream.udp_ip type mismatch"};
            }
        } else if (key == ConfigRegistry::kStreamSendImage) {
            if (const auto* v = std::get_if<bool>(&value)) {
                remote.sendImage = *v;
            } else {
                return {false, "stream.send_image type mismatch"};
            }
        } else if (key == ConfigRegistry::kStreamSendFeature) {
            if (const auto* v = std::get_if<bool>(&value)) {
                remote.sendFeature = *v;
            } else {
                return {false, "stream.send_feature type mismatch"};
            }
        } else if (key == ConfigRegistry::kStreamSendMap) {
            if (const auto* v = std::get_if<bool>(&value)) {
                remote.sendMap = *v;
            } else {
                return {false, "stream.send_map type mismatch"};
            }
        } else {
            return {false, "unsupported config key: " + key};
        }
    }

    std::string err;
    if (!UpdateRemoteConfig(remote, &err)) {
        return {false, err.empty() ? "runtime cfg failed" : err};
    }
    const int cameraFps = current.app.camera.fps > 0 ? current.app.camera.fps : 1;
    const int clampedSlamFps = remote.slamInputFps <= 0 ? cameraFps : std::min(cameraFps, remote.slamInputFps);
    return {
        true,
        "runtime cfg updated sensor=" + std::string(ToSensorModeText(remote.sensorMode)) +
            " slam_mode=" + std::string(smartdrone::core::domain::ToString(remote.slamOperationMode)) +
            " pair_ms=" + std::to_string(remote.pairMs) +
            " slam_fps=" + std::to_string(clampedSlamFps)
    };
}

void UnifiedRuntimeController::JoinSession()
{
    if (m_session.joinable()) {
        m_session.join();
    }
}

void UnifiedRuntimeController::Loop()
{
    while (m_runningFlag.load()) {
        ControllerMode startMode = ControllerMode::Idle;
        UnifiedConfig cfg{};
        bool startSession = false;
        bool needJoin = false;
        {
            std::unique_lock<std::mutex> lock(m_mu);
            m_cv.wait_for(lock, std::chrono::milliseconds(100), [this]() {
                return m_stopping || m_modeManager.RestartRequested() || m_sessionDone;
            });
            if (m_sessionDone) {
                needJoin = true;
            }
            if (m_stopping) {
                m_sessionStop.store(true);
            }
            if (m_modeManager.ShouldStopActiveSession()) {
                m_sessionStop.store(true);
                needJoin = true;
            }
        }
        if (needJoin) {
            JoinSession();
        }
        {
            std::lock_guard<std::mutex> lock(m_mu);
            if (needJoin) {
                m_sessionDone = false;
                m_modeManager.MarkSessionJoined();
                m_cv.notify_all();
            }
            if (m_stopping) {
                break;
            }
            if (m_modeManager.ActiveMode() != ControllerMode::Idle &&
                m_modeManager.DesiredMode() == m_modeManager.ActiveMode() &&
                !m_modeManager.RestartRequested()) {
                continue;
            }
            if (m_modeManager.DesiredMode() != ControllerMode::Idle) {
                cfg = m_config;
                startMode = m_modeManager.DesiredMode();
                m_modeManager.MarkSessionLaunching(startMode);
                m_sessionStop.store(false);
                startSession = true;
            } else {
                m_modeManager.MarkSessionJoined();
            }
        }
        if (startSession) {
            m_session = SMARTDRONE_START_THREAD(
                smartdrone::common::ThreadRole::RuntimeSession,
                "UnifiedRuntimeController",
                [this, cfg, startMode]() mutable {
                    bool ok = false;
                    if (startMode == ControllerMode::Slam && m_slamSessionRunner) {
                        ok = m_slamSessionRunner(cfg, m_tuning, m_mav, m_sessionStop, m_livePose);
                    } else if (startMode == ControllerMode::Calib && m_calibSessionRunner) {
                        ok = m_calibSessionRunner(cfg, m_sessionStop, m_livePose);
                    }
                    (void)ok;
                    std::lock_guard<std::mutex> lock(m_mu);
                    m_sessionDone = true;
                    m_cv.notify_all();
                });
        }
    }
}

}  // namespace smartdrone::core::application
