#include "core/application/runtime/runtime_config_service.h"

#include <algorithm>
#include <variant>

#include "core/application/config/app_args.h"
#include "core/application/config/config_registry.h"

namespace smartdrone::core::application {

RuntimeConfigService::RuntimeConfigService(UnifiedConfig &config, LiveRuntimeTuning &tuning, std::mutex &configMutex,
                                           RestartFn requestRestart)
    : m_config(config), m_tuning(tuning), m_configMutex(configMutex), m_requestRestart(std::move(requestRestart))
{
}

bool RuntimeConfigService::UpdateRemoteConfig(const RemoteRuntimeConfig &remote, std::string *err)
{
    if (remote.exposureUs <= 0 || !(remote.gain > 0.0f) || remote.pairMs <= 0 || remote.slamInputFps < 0) {
        if (err) {
            *err = "bad runtime config";
        }
        return false;
    }

    bool restartNeeded = false;
    {
        std::lock_guard<std::mutex> lock(m_configMutex);
        CameraConfig &cam = m_config.app.camera;
        const bool sensorModeChanged = m_config.app.sensorMode != remote.sensorMode;
        const bool udpIpChanged = m_config.app.udp.ip != remote.udpIp;
        const bool udpEnableChanged = m_config.app.udp.enable != remote.udpEnabled;
        cam.exposureUs = remote.exposureUs;
        cam.gain = remote.gain;
        cam.pairMs = remote.pairMs;
        m_config.app.runtime.slamInputFps = remote.slamInputFps;
        m_config.app.runtime.slamOperationMode = remote.slamOperationMode;
        m_config.app.sensorMode = remote.sensorMode;
        m_config.app.settings = ResolveSettingsForSensorMode(remote.sensorMode, m_config.app.settings);
        m_config.app.udp.ip = remote.udpIp;
        m_config.app.udp.enable = remote.udpEnabled;
        m_config.app.udp.sendImage = remote.sendImage;
        m_config.app.udp.sendFeature = remote.sendFeature;
        m_config.app.udp.sendMap = remote.sendMap;
        restartNeeded = sensorModeChanged || udpIpChanged || udpEnableChanged;
    }

    m_tuning.slamInputFps.store(remote.slamInputFps, std::memory_order_relaxed);
    m_tuning.slamOperationMode.store(static_cast<uint8_t>(remote.slamOperationMode), std::memory_order_relaxed);
    m_tuning.sendImage.store(remote.sendImage, std::memory_order_relaxed);
    m_tuning.sendFeature.store(remote.sendFeature, std::memory_order_relaxed);
    m_tuning.sendMap.store(remote.sendMap, std::memory_order_relaxed);

    if (restartNeeded && m_requestRestart) {
        m_requestRestart();
    }
    return true;
}

CommandResult RuntimeConfigService::ApplyConfig(const ConfigUpdate &update, const UnifiedConfig &currentConfig)
{
    RemoteRuntimeConfig remote = BuildRemoteConfig(currentConfig);

    for (const auto &[key, value] : update.values) {
        if (key == ConfigRegistry::kCameraExposureUs) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.exposureUs = static_cast<int>(*v);
            } else {
                return {false, "camera.exposure_us type mismatch"};
            }
        } else if (key == ConfigRegistry::kCameraGain) {
            if (const auto *v = std::get_if<double>(&value)) {
                remote.gain = static_cast<float>(*v);
            } else if (const auto *vInt = std::get_if<int64_t>(&value)) {
                remote.gain = static_cast<float>(*vInt);
            } else {
                return {false, "camera.gain type mismatch"};
            }
        } else if (key == ConfigRegistry::kCameraPairWindowMs) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.pairMs = static_cast<int>(*v);
            } else {
                return {false, "camera.pair_window_ms type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamInputFps) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.slamInputFps = static_cast<int>(*v);
            } else {
                return {false, "slam.input_fps type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamOperationMode) {
            if (const auto *v = std::get_if<std::string>(&value)) {
                remote.slamOperationMode = ParseSlamOperationModeText(*v);
            } else {
                return {false, "slam.operation_mode type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamPerceptionMode) {
            if (const auto *v = std::get_if<std::string>(&value)) {
                remote.sensorMode = ParseSensorModeText(*v);
            } else {
                return {false, "slam.perception_mode type mismatch"};
            }
        } else if (key == ConfigRegistry::kStreamUdpEnabled) {
            if (const auto *v = std::get_if<bool>(&value)) {
                remote.udpEnabled = *v;
            } else {
                return {false, "stream.udp_enabled type mismatch"};
            }
        } else if (key == ConfigRegistry::kStreamUdpIp) {
            if (const auto *v = std::get_if<std::string>(&value)) {
                remote.udpIp = *v;
            } else {
                return {false, "stream.udp_ip type mismatch"};
            }
        } else if (key == ConfigRegistry::kStreamSendImage) {
            if (const auto *v = std::get_if<bool>(&value)) {
                remote.sendImage = *v;
            } else {
                return {false, "stream.send_image type mismatch"};
            }
        } else if (key == ConfigRegistry::kStreamSendFeature) {
            if (const auto *v = std::get_if<bool>(&value)) {
                remote.sendFeature = *v;
            } else {
                return {false, "stream.send_feature type mismatch"};
            }
        } else if (key == ConfigRegistry::kStreamSendMap) {
            if (const auto *v = std::get_if<bool>(&value)) {
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

    const int cameraFps = currentConfig.app.camera.fps > 0 ? currentConfig.app.camera.fps : 1;
    const int clampedSlamFps = remote.slamInputFps <= 0 ? cameraFps : std::min(cameraFps, remote.slamInputFps);
    return {true, "runtime cfg updated sensor=" + std::string(ToSensorModeText(remote.sensorMode)) +
                      " slam_mode=" + std::string(smartdrone::core::domain::ToString(remote.slamOperationMode)) +
                      " pair_ms=" + std::to_string(remote.pairMs) + " slam_fps=" + std::to_string(clampedSlamFps)};
}

RemoteRuntimeConfig RuntimeConfigService::BuildRemoteConfig(const UnifiedConfig &currentConfig)
{
    RemoteRuntimeConfig remote{};
    remote.exposureUs = currentConfig.app.camera.exposureUs;
    remote.gain = currentConfig.app.camera.gain;
    remote.pairMs = currentConfig.app.camera.pairMs > 0 ? currentConfig.app.camera.pairMs : 1;
    remote.slamInputFps = currentConfig.app.runtime.slamInputFps;
    remote.slamOperationMode = currentConfig.app.runtime.slamOperationMode;
    remote.sensorMode = currentConfig.app.sensorMode;
    remote.udpIp = currentConfig.app.udp.ip;
    remote.udpEnabled = currentConfig.app.udp.enable;
    remote.sendImage = currentConfig.app.udp.sendImage;
    remote.sendFeature = currentConfig.app.udp.sendFeature;
    remote.sendMap = currentConfig.app.udp.sendMap;
    return remote;
}

} // namespace smartdrone::core::application
