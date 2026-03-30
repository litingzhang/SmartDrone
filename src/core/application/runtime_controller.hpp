#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <variant>

#include "adapters/telemetry/px4_mavlink_gateway.hpp"
#include "core/application/config_registry.hpp"
#include "core/application/live_pose_state.hpp"
#include "core/application/mode_manager.hpp"
#include "core/application/runtime_app_types.hpp"
#include "core/application/runtime_command_service.hpp"
#include "core/domain/runtime_mode.hpp"

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
    }

    void Start() { m_worker = std::thread([this]() { Loop(); }); }

    void Stop()
    {
        {
            std::lock_guard<std::mutex> lock(m_mu);
            m_stopping = true;
            m_modeManager.RequestMode(ControllerMode::Idle);
            m_sessionStop.store(true);
        }
        m_cv.notify_all();
        if (m_worker.joinable()) m_worker.join();
        JoinSession();
    }

    bool SetMode(ControllerMode mode, std::string* err)
    {
        std::lock_guard<std::mutex> lock(m_mu);
        if (m_stopping) {
            if (err) *err = "runtime stopping";
            return false;
        }
        m_modeManager.RequestMode(mode);
        m_livePose.SetRuntimeMode(static_cast<uint8_t>(mode));
        m_cv.notify_all();
        return true;
    }

    bool UpdateRemoteConfig(const RemoteRuntimeConfig& r, std::string* err)
    {
        if (r.exposureUs <= 0 || !(r.gain > 0.0f) || r.pairMs <= 0 || r.slamInputFps < 0) {
            if (err) *err = "bad runtime config";
            return false;
        }
        std::lock_guard<std::mutex> lock(m_mu);
        CameraConfig& cam = m_config.app.camera;
        const bool exposureChanged = cam.exposureUs != r.exposureUs;
        const bool gainChanged = cam.gain != r.gain;
        const bool pairMsChanged = cam.pairMs != r.pairMs;
        const bool sensorModeChanged = m_config.app.sensorMode != r.sensorMode;
        const bool udpIpChanged = m_config.app.udp.ip != r.udpIp;
        const bool udpEnableChanged = m_config.app.udp.enable != r.udpEnabled;
        const bool sendImageChanged = m_config.app.udp.sendImage != r.sendImage;
        const bool sendFeatureChanged = m_config.app.udp.sendFeature != r.sendFeature;
        const bool sendMapChanged = m_config.app.udp.sendMap != r.sendMap;
        cam.exposureUs = r.exposureUs;
        cam.gain = r.gain;
        cam.pairMs = r.pairMs;
        m_config.app.runtime.slamInputFps = r.slamInputFps;
        m_config.app.sensorMode = r.sensorMode;
        m_config.app.settings = ResolveSettingsForSensorMode(r.sensorMode, m_config.app.settings);
        m_config.app.udp.ip = r.udpIp;
        m_config.app.udp.enable = r.udpEnabled;
        m_config.app.udp.sendImage = r.sendImage;
        m_config.app.udp.sendFeature = r.sendFeature;
        m_config.app.udp.sendMap = r.sendMap;
        m_tuning.slamInputFps.store(r.slamInputFps, std::memory_order_relaxed);
        const bool restartNeeded = exposureChanged || gainChanged || pairMsChanged || sensorModeChanged ||
                                   udpIpChanged || udpEnableChanged || sendImageChanged ||
                                   sendFeatureChanged || sendMapChanged;
        if (restartNeeded && m_modeManager.DesiredMode() != ControllerMode::Idle) {
            m_modeManager.RequestRestart();
            m_sessionStop.store(true);
        }
        m_cv.notify_all();
        return true;
    }

    bool CleanupCalibData(std::string* msg)
    {
        {
            std::unique_lock<std::mutex> lock(m_mu);
            if (m_stopping) {
                if (msg) *msg = "runtime stopping";
                return false;
            }

            if (m_modeManager.ActiveMode() != ControllerMode::Idle ||
                m_modeManager.DesiredMode() != ControllerMode::Idle ||
                m_session.joinable()) {
                m_modeManager.RequestMode(ControllerMode::Idle);
                m_sessionStop.store(true);
                m_cv.notify_all();
                const bool idleReady = m_cv.wait_for(lock, std::chrono::seconds(5), [this]() {
                    return m_stopping
                        || (m_modeManager.ActiveMode() == ControllerMode::Idle
                            && m_modeManager.DesiredMode() == ControllerMode::Idle
                            && !m_session.joinable()
                            && !m_sessionDone);
                });
                if (!idleReady) {
                    if (msg) *msg = "runtime busy";
                    return false;
                }
                if (m_stopping) {
                    if (msg) *msg = "runtime stopping";
                    return false;
                }
            }
        }
        const int removed = m_cleanupCalibData ? m_cleanupCalibData(m_config.calib.root) : 0;
        if (msg) *msg = "calib clean removed=" + std::to_string(removed);
        return true;
    }

    UnifiedConfig CurrentConfig()
    {
        std::lock_guard<std::mutex> lock(m_mu);
        return m_config;
    }

    ControllerMode CurrentDesiredMode()
    {
        std::lock_guard<std::mutex> lock(m_mu);
        return m_modeManager.DesiredMode();
    }

    CommandResult ExecuteAction(const RuntimeAction& action) override
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
            case RuntimeAction::Type::ResetMap:
                return {false, "reset map not implemented"};
            case RuntimeAction::Type::SaveMap:
                return {false, "save map not implemented"};
            default:
                return {false, "unknown action"};
        }
    }

    CommandResult ApplyConfig(const ConfigUpdate& update) override
    {
        UnifiedConfig current = CurrentConfig();
        RemoteRuntimeConfig remote{};
        remote.exposureUs = current.app.camera.exposureUs;
        remote.gain = current.app.camera.gain;
        remote.pairMs = current.app.camera.pairMs > 0 ? current.app.camera.pairMs : 1;
        remote.slamInputFps = current.app.runtime.slamInputFps;
        remote.sensorMode = current.app.sensorMode;
        remote.udpIp = current.app.udp.ip;
        remote.udpEnabled = current.app.udp.enable;
        remote.sendImage = current.app.udp.sendImage;
        remote.sendFeature = current.app.udp.sendFeature;
        remote.sendMap = current.app.udp.sendMap;

        for (const auto& [key, value] : update.values) {
            if (key == ConfigRegistry::kCameraExposureUs) {
                if (const auto* v = std::get_if<int64_t>(&value)) remote.exposureUs = static_cast<int>(*v);
                else return {false, "camera.exposure_us type mismatch"};
            } else if (key == ConfigRegistry::kCameraGain) {
                if (const auto* v = std::get_if<double>(&value)) remote.gain = static_cast<float>(*v);
                else if (const auto* vInt = std::get_if<int64_t>(&value)) remote.gain = static_cast<float>(*vInt);
                else return {false, "camera.gain type mismatch"};
            } else if (key == ConfigRegistry::kCameraPairWindowMs) {
                if (const auto* v = std::get_if<int64_t>(&value)) remote.pairMs = static_cast<int>(*v);
                else return {false, "camera.pair_window_ms type mismatch"};
            } else if (key == ConfigRegistry::kSlamInputFps) {
                if (const auto* v = std::get_if<int64_t>(&value)) remote.slamInputFps = static_cast<int>(*v);
                else return {false, "slam.input_fps type mismatch"};
            } else if (key == ConfigRegistry::kSlamPerceptionMode) {
                if (const auto* v = std::get_if<std::string>(&value)) remote.sensorMode = ParseSensorModeText(*v);
                else return {false, "slam.perception_mode type mismatch"};
            } else if (key == ConfigRegistry::kStreamUdpEnabled) {
                if (const auto* v = std::get_if<bool>(&value)) remote.udpEnabled = *v;
                else return {false, "stream.udp_enabled type mismatch"};
            } else if (key == ConfigRegistry::kStreamUdpIp) {
                if (const auto* v = std::get_if<std::string>(&value)) remote.udpIp = *v;
                else return {false, "stream.udp_ip type mismatch"};
            } else if (key == ConfigRegistry::kStreamSendImage) {
                if (const auto* v = std::get_if<bool>(&value)) remote.sendImage = *v;
                else return {false, "stream.send_image type mismatch"};
            } else if (key == ConfigRegistry::kStreamSendFeature) {
                if (const auto* v = std::get_if<bool>(&value)) remote.sendFeature = *v;
                else return {false, "stream.send_feature type mismatch"};
            } else if (key == ConfigRegistry::kStreamSendMap) {
                if (const auto* v = std::get_if<bool>(&value)) remote.sendMap = *v;
                else return {false, "stream.send_map type mismatch"};
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
                " pair_ms=" + std::to_string(remote.pairMs) +
                " slam_fps=" + std::to_string(clampedSlamFps)
        };
    }

private:
    void JoinSession()
    {
        if (m_session.joinable()) m_session.join();
    }

    void Loop()
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
                if (m_stopping) m_sessionStop.store(true);
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
                if (m_stopping) break;
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
                m_session = std::thread([this, cfg, startMode]() mutable {
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
