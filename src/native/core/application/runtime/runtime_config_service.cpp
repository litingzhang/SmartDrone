#include "core/application/runtime/runtime_config_service.h"

#include <algorithm>
#include <cmath>
#include <variant>

#include "core/application/config/app_args.h"
#include "core/application/config/config_registry.h"
#include "core/application/session/runtime_session_common.h"

namespace smartdrone::core::application {

RuntimeConfigService::RuntimeConfigService(UnifiedConfig &config, LiveRuntimeTuning &tuning, std::mutex &configMutex,
                                           RestartFn requestRestart)
    : m_config(config), m_tuning(tuning), m_configMutex(configMutex), m_requestRestart(std::move(requestRestart))
{
}

bool RuntimeConfigService::UpdateRemoteConfig(RemoteRuntimeConfig remote, std::string *err)
{
    if (remote.featureFrontend == FeatureFrontend::XFeat) {
        remote.featureFrontend = FeatureFrontend::Orb;
    }
    if (remote.exposureUs <= 0 || !std::isfinite(remote.gain) || remote.gain < 0.0f || remote.pairMs <= 0 ||
        remote.slamInputFps < 0 ||
        remote.uvcDeviceIndex < 0 || remote.uvcEyeWidth <= 0 || remote.uvcEyeHeight <= 0) {
        if (err) {
            *err = "bad runtime config";
        }
        return false;
    }
    if (!std::isfinite(remote.tbcTx) || !std::isfinite(remote.tbcTy) || !std::isfinite(remote.tbcTz) ||
        !std::isfinite(remote.tbcRollDeg) || !std::isfinite(remote.tbcPitchDeg) || !std::isfinite(remote.tbcYawDeg)) {
        if (err) {
            *err = "bad tbc override config";
        }
        return false;
    }
    if (remote.orbNFeatures <= 0 || !(remote.orbScaleFactor > 0.0f) || remote.orbNLevels <= 0 ||
        remote.orbIniThFAST <= 0 || remote.orbMinThFAST <= 0) {
        if (err) {
            *err = "bad orb extractor config";
        }
        return false;
    }
    if (remote.orbMinThFAST > remote.orbIniThFAST) {
        if (err) {
            *err = "orb minThFAST must be <= iniThFAST";
        }
        return false;
    }
    if (remote.orbNFeatures < 100 || remote.orbNFeatures > 5000 || remote.orbScaleFactor < 1.01f ||
        remote.orbScaleFactor > 3.0f || remote.orbNLevels < 1 || remote.orbNLevels > 16 ||
        remote.orbIniThFAST > 100 || remote.orbMinThFAST > 100) {
        if (err) {
            *err = "orb extractor config out of range";
        }
        return false;
    }
    if (remote.xfeatTopK < 1 || remote.xfeatTopK > 4096 || remote.xfeatMaxPoints < 1 ||
        remote.xfeatMaxPoints > 4096 || remote.xfeatMaxPoints > remote.xfeatTopK) {
        if (err) {
            *err = "xfeat config out of range";
        }
        return false;
    }
    if (remote.xfeatInputMaxWidth < 0 || remote.xfeatInputMaxWidth > 4096 || remote.xfeatInputMaxHeight < 0 ||
        remote.xfeatInputMaxHeight > 4096) {
        if (err) {
            *err = "xfeat input size config out of range";
        }
        return false;
    }
    if (remote.lkPerFrameAcceleration != "cpu" && remote.lkPerFrameAcceleration != "vpi-cuda" &&
        remote.lkPerFrameAcceleration != "auto") {
        if (err) {
            *err = "bad lk per-frame acceleration";
        }
        return false;
    }

    bool restartNeeded = false;
    float effectiveTbcTx = remote.tbcTx;
    float effectiveTbcTy = remote.tbcTy;
    float effectiveTbcTz = remote.tbcTz;
    float effectiveTbcRollDeg = remote.tbcRollDeg;
    float effectiveTbcPitchDeg = remote.tbcPitchDeg;
    float effectiveTbcYawDeg = remote.tbcYawDeg;
    {
        std::lock_guard<std::mutex> lock(m_configMutex);
        CameraConfig &cam = m_config.app.camera;
        const bool sensorModeChanged = m_config.app.sensorMode != remote.sensorMode;
        const bool frontendChanged = m_config.app.runtime.featureFrontend != remote.featureFrontend;
        const bool aeModeChanged = cam.aeDisable != (!remote.autoExposureEnabled);
        const bool uvcConfigChanged = cam.uvcDeviceIndex != remote.uvcDeviceIndex ||
                                      cam.uvcEyeWidth != remote.uvcEyeWidth ||
                                      cam.uvcEyeHeight != remote.uvcEyeHeight ||
                                      cam.uvcPackedStereo != remote.uvcPackedStereo;
        const bool orbChanged = m_config.app.runtime.orbNFeatures != remote.orbNFeatures ||
                                std::abs(m_config.app.runtime.orbScaleFactor - remote.orbScaleFactor) > 1e-6f ||
                                m_config.app.runtime.orbNLevels != remote.orbNLevels ||
                                m_config.app.runtime.orbIniThFAST != remote.orbIniThFAST ||
                                m_config.app.runtime.orbMinThFAST != remote.orbMinThFAST;
        const bool xfeatChanged = m_config.app.runtime.xfeatTopK != remote.xfeatTopK ||
                                  m_config.app.runtime.xfeatMaxPoints != remote.xfeatMaxPoints ||
                                  m_config.app.runtime.xfeatInputMaxWidth != remote.xfeatInputMaxWidth ||
                                  m_config.app.runtime.xfeatInputMaxHeight != remote.xfeatInputMaxHeight;
        const bool lkSeedChanged = m_config.app.runtime.lkXFeatSeeding != remote.lkXFeatSeeding;
        const bool lkPerFrameAccelChanged =
            m_config.app.runtime.lkPerFrameAcceleration != remote.lkPerFrameAcceleration;
        cam.exposureUs = remote.exposureUs;
        cam.gain = remote.gain;
        cam.aeDisable = !remote.autoExposureEnabled;
        cam.pairMs = remote.pairMs;
        cam.uvcDeviceIndex = remote.uvcDeviceIndex;
        cam.uvcEyeWidth = remote.uvcEyeWidth;
        cam.uvcEyeHeight = remote.uvcEyeHeight;
        cam.uvcPackedStereo = remote.uvcPackedStereo;
        m_config.app.runtime.slamInputFps = remote.slamInputFps;
        m_config.app.runtime.slamOperationMode = remote.slamOperationMode;
        m_config.app.runtime.featureFrontend = remote.featureFrontend;
        m_config.app.runtime.orbNFeatures = remote.orbNFeatures;
        m_config.app.runtime.orbScaleFactor = remote.orbScaleFactor;
        m_config.app.runtime.orbNLevels = remote.orbNLevels;
        m_config.app.runtime.orbIniThFAST = remote.orbIniThFAST;
        m_config.app.runtime.orbMinThFAST = remote.orbMinThFAST;
        m_config.app.runtime.xfeatTopK = remote.xfeatTopK;
        m_config.app.runtime.xfeatMaxPoints = remote.xfeatMaxPoints;
        m_config.app.runtime.xfeatInputMaxWidth = remote.xfeatInputMaxWidth;
        m_config.app.runtime.xfeatInputMaxHeight = remote.xfeatInputMaxHeight;
        m_config.app.runtime.lkXFeatSeeding = remote.lkXFeatSeeding;
        m_config.app.runtime.lkPerFrameAcceleration = remote.lkPerFrameAcceleration;
        m_config.app.sensorMode = remote.sensorMode;
        m_config.app.settings = ResolveSettingsForSensorMode(remote.sensorMode, m_config.app.settings);
        m_config.app.udp.ip = remote.udpIp;
        m_config.app.udp.enable = remote.udpEnabled;
        m_config.app.udp.sendImage = remote.sendImage;
        m_config.app.udp.sendFeature = remote.sendFeature;
        m_config.app.udp.sendMap = remote.sendMap;
        m_config.app.runtime.useCustomTbc = remote.useCustomTbc;
        if (remote.useCustomTbc) {
            m_config.app.runtime.tbcTx = remote.tbcTx;
            m_config.app.runtime.tbcTy = remote.tbcTy;
            m_config.app.runtime.tbcTz = remote.tbcTz;
            m_config.app.runtime.tbcRollDeg = remote.tbcRollDeg;
            m_config.app.runtime.tbcPitchDeg = remote.tbcPitchDeg;
            m_config.app.runtime.tbcYawDeg = remote.tbcYawDeg;
        } else {
            const auto extrinsics = LoadStereoBodyExtrinsics(m_config.app.settings);
            if (extrinsics.loaded) {
                const Eigen::Vector3f t = extrinsics.Tbc.translation();
                m_config.app.runtime.tbcTx = t.x();
                m_config.app.runtime.tbcTy = t.y();
                m_config.app.runtime.tbcTz = t.z();
                // Runtime override now applies a dynamic pitch delta on top of
                // calibrated T_b_c1, so the neutral UI state is zero delta.
                m_config.app.runtime.tbcRollDeg = 0.0f;
                m_config.app.runtime.tbcPitchDeg = 0.0f;
                m_config.app.runtime.tbcYawDeg = 0.0f;
            } else {
                m_config.app.runtime.tbcTx = remote.tbcTx;
                m_config.app.runtime.tbcTy = remote.tbcTy;
                m_config.app.runtime.tbcTz = remote.tbcTz;
                m_config.app.runtime.tbcRollDeg = remote.tbcRollDeg;
                m_config.app.runtime.tbcPitchDeg = remote.tbcPitchDeg;
                m_config.app.runtime.tbcYawDeg = remote.tbcYawDeg;
            }
        }
        effectiveTbcTx = m_config.app.runtime.tbcTx;
        effectiveTbcTy = m_config.app.runtime.tbcTy;
        effectiveTbcTz = m_config.app.runtime.tbcTz;
        effectiveTbcRollDeg = m_config.app.runtime.tbcRollDeg;
        effectiveTbcPitchDeg = m_config.app.runtime.tbcPitchDeg;
        effectiveTbcYawDeg = m_config.app.runtime.tbcYawDeg;
        restartNeeded =
            sensorModeChanged || frontendChanged || aeModeChanged || uvcConfigChanged || orbChanged || xfeatChanged ||
            lkSeedChanged || lkPerFrameAccelChanged;
    }

    m_tuning.slamInputFps.store(remote.slamInputFps, std::memory_order_relaxed);
    m_tuning.slamOperationMode.store(static_cast<uint8_t>(remote.slamOperationMode), std::memory_order_relaxed);
    m_tuning.featureFrontend.store(static_cast<uint8_t>(remote.featureFrontend), std::memory_order_relaxed);
    m_tuning.sendImage.store(remote.sendImage, std::memory_order_relaxed);
    m_tuning.sendFeature.store(remote.sendFeature, std::memory_order_relaxed);
    m_tuning.sendMap.store(remote.sendMap, std::memory_order_relaxed);
    m_tuning.useCustomTbc.store(remote.useCustomTbc, std::memory_order_relaxed);
    m_tuning.tbcTx.store(effectiveTbcTx, std::memory_order_relaxed);
    m_tuning.tbcTy.store(effectiveTbcTy, std::memory_order_relaxed);
    m_tuning.tbcTz.store(effectiveTbcTz, std::memory_order_relaxed);
    m_tuning.tbcRollDeg.store(effectiveTbcRollDeg, std::memory_order_relaxed);
    m_tuning.tbcPitchDeg.store(effectiveTbcPitchDeg, std::memory_order_relaxed);
    m_tuning.tbcYawDeg.store(effectiveTbcYawDeg, std::memory_order_relaxed);

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
        } else if (key == ConfigRegistry::kCameraAutoExposure) {
            if (const auto *v = std::get_if<bool>(&value)) {
                remote.autoExposureEnabled = *v;
            } else {
                return {false, "camera.auto_exposure type mismatch"};
            }
        } else if (key == ConfigRegistry::kCameraPairWindowMs) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.pairMs = static_cast<int>(*v);
            } else {
                return {false, "camera.pair_window_ms type mismatch"};
            }
        } else if (key == ConfigRegistry::kCameraUvcDeviceIndex) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.uvcDeviceIndex = static_cast<int>(*v);
            } else {
                return {false, "camera.uvc_device_index type mismatch"};
            }
        } else if (key == ConfigRegistry::kCameraUvcEyeWidth) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.uvcEyeWidth = static_cast<int>(*v);
            } else {
                return {false, "camera.uvc_eye_width type mismatch"};
            }
        } else if (key == ConfigRegistry::kCameraUvcEyeHeight) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.uvcEyeHeight = static_cast<int>(*v);
            } else {
                return {false, "camera.uvc_eye_height type mismatch"};
            }
        } else if (key == ConfigRegistry::kCameraUvcPackedStereo) {
            if (const auto *v = std::get_if<bool>(&value)) {
                remote.uvcPackedStereo = *v;
            } else {
                return {false, "camera.uvc_packed_stereo type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamInputFps) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.slamInputFps = static_cast<int>(*v);
            } else {
                return {false, "slam.input_fps type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamFeatureFrontend) {
            if (const auto *v = std::get_if<std::string>(&value)) {
                remote.featureFrontend = ParseFeatureFrontendText(*v);
            } else {
                return {false, "slam.feature_frontend type mismatch"};
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
        } else if (key == ConfigRegistry::kSlamUseCustomTbc) {
            if (const auto *v = std::get_if<bool>(&value)) {
                remote.useCustomTbc = *v;
            } else {
                return {false, "slam.tbc_override_enabled type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamTbcTx) {
            if (const auto *v = std::get_if<double>(&value)) {
                remote.tbcTx = static_cast<float>(*v);
            } else if (const auto *vInt = std::get_if<int64_t>(&value)) {
                remote.tbcTx = static_cast<float>(*vInt);
            } else {
                return {false, "slam.tbc_tx_m type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamTbcTy) {
            if (const auto *v = std::get_if<double>(&value)) {
                remote.tbcTy = static_cast<float>(*v);
            } else if (const auto *vInt = std::get_if<int64_t>(&value)) {
                remote.tbcTy = static_cast<float>(*vInt);
            } else {
                return {false, "slam.tbc_ty_m type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamTbcTz) {
            if (const auto *v = std::get_if<double>(&value)) {
                remote.tbcTz = static_cast<float>(*v);
            } else if (const auto *vInt = std::get_if<int64_t>(&value)) {
                remote.tbcTz = static_cast<float>(*vInt);
            } else {
                return {false, "slam.tbc_tz_m type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamTbcRollDeg) {
            if (const auto *v = std::get_if<double>(&value)) {
                remote.tbcRollDeg = static_cast<float>(*v);
            } else if (const auto *vInt = std::get_if<int64_t>(&value)) {
                remote.tbcRollDeg = static_cast<float>(*vInt);
            } else {
                return {false, "slam.tbc_roll_deg type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamTbcPitchDeg) {
            if (const auto *v = std::get_if<double>(&value)) {
                remote.tbcPitchDeg = static_cast<float>(*v);
            } else if (const auto *vInt = std::get_if<int64_t>(&value)) {
                remote.tbcPitchDeg = static_cast<float>(*vInt);
            } else {
                return {false, "slam.tbc_pitch_deg type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamTbcYawDeg) {
            if (const auto *v = std::get_if<double>(&value)) {
                remote.tbcYawDeg = static_cast<float>(*v);
            } else if (const auto *vInt = std::get_if<int64_t>(&value)) {
                remote.tbcYawDeg = static_cast<float>(*vInt);
            } else {
                return {false, "slam.tbc_yaw_deg type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamOrbNFeatures) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.orbNFeatures = static_cast<int>(*v);
            } else if (const auto *vFloat = std::get_if<double>(&value)) {
                remote.orbNFeatures = static_cast<int>(*vFloat);
            } else {
                return {false, "slam.orb_nfeatures type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamOrbScaleFactor) {
            if (const auto *v = std::get_if<double>(&value)) {
                remote.orbScaleFactor = static_cast<float>(*v);
            } else if (const auto *vInt = std::get_if<int64_t>(&value)) {
                remote.orbScaleFactor = static_cast<float>(*vInt);
            } else {
                return {false, "slam.orb_scale_factor type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamOrbNLevels) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.orbNLevels = static_cast<int>(*v);
            } else if (const auto *vFloat = std::get_if<double>(&value)) {
                remote.orbNLevels = static_cast<int>(*vFloat);
            } else {
                return {false, "slam.orb_nlevels type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamOrbIniThFast) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.orbIniThFAST = static_cast<int>(*v);
            } else if (const auto *vFloat = std::get_if<double>(&value)) {
                remote.orbIniThFAST = static_cast<int>(*vFloat);
            } else {
                return {false, "slam.orb_ini_th_fast type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamOrbMinThFast) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.orbMinThFAST = static_cast<int>(*v);
            } else if (const auto *vFloat = std::get_if<double>(&value)) {
                remote.orbMinThFAST = static_cast<int>(*vFloat);
            } else {
                return {false, "slam.orb_min_th_fast type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamXFeatTopK) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.xfeatTopK = static_cast<int>(*v);
            } else if (const auto *vFloat = std::get_if<double>(&value)) {
                remote.xfeatTopK = static_cast<int>(*vFloat);
            } else {
                return {false, "slam.xfeat_top_k type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamXFeatMaxPoints) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.xfeatMaxPoints = static_cast<int>(*v);
            } else if (const auto *vFloat = std::get_if<double>(&value)) {
                remote.xfeatMaxPoints = static_cast<int>(*vFloat);
            } else {
                return {false, "slam.xfeat_max_points type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamXFeatInputMaxWidth) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.xfeatInputMaxWidth = static_cast<int>(*v);
            } else if (const auto *vFloat = std::get_if<double>(&value)) {
                remote.xfeatInputMaxWidth = static_cast<int>(*vFloat);
            } else {
                return {false, "slam.xfeat_input_max_width type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamXFeatInputMaxHeight) {
            if (const auto *v = std::get_if<int64_t>(&value)) {
                remote.xfeatInputMaxHeight = static_cast<int>(*v);
            } else if (const auto *vFloat = std::get_if<double>(&value)) {
                remote.xfeatInputMaxHeight = static_cast<int>(*vFloat);
            } else {
                return {false, "slam.xfeat_input_max_height type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamLkXFeatSeeding) {
            if (const auto *v = std::get_if<bool>(&value)) {
                remote.lkXFeatSeeding = *v;
            } else {
                return {false, "slam.lk_xfeat_seeding type mismatch"};
            }
        } else if (key == ConfigRegistry::kSlamLkPerFrameAcceleration) {
            if (const auto *v = std::get_if<std::string>(&value)) {
                remote.lkPerFrameAcceleration = *v;
            } else {
                return {false, "slam.lk_per_frame_accel type mismatch"};
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
    std::string message = "runtime cfg updated sensor=" + std::string(ToSensorModeText(remote.sensorMode)) +
                          " frontend=" + std::string(ToFeatureFrontendText(remote.featureFrontend)) +
                          " slam_mode=" + std::string(smartdrone::core::domain::ToString(remote.slamOperationMode)) +
                          " slam_fps=" + std::to_string(clampedSlamFps) + " pair_ms=" +
                          std::to_string(remote.pairMs) + " provider_specific xfeat_top_k=" +
                          std::to_string(remote.xfeatTopK) + " xfeat_max_points=" +
                          std::to_string(remote.xfeatMaxPoints) + " xfeat_input_max=" +
                          std::to_string(remote.xfeatInputMaxWidth) + "x" +
                          std::to_string(remote.xfeatInputMaxHeight) + " lk_seed=" +
                          (remote.lkXFeatSeeding ? "xfeat" : "gftt") +
                          " lk_accel=" + remote.lkPerFrameAcceleration;
    return {true, message};
}

RemoteRuntimeConfig RuntimeConfigService::BuildRemoteConfig(const UnifiedConfig &currentConfig)
{
    RemoteRuntimeConfig remote{};
    remote.exposureUs = currentConfig.app.camera.exposureUs;
    remote.gain = currentConfig.app.camera.gain;
    remote.autoExposureEnabled = !currentConfig.app.camera.aeDisable;
    remote.pairMs = currentConfig.app.camera.pairMs > 0 ? currentConfig.app.camera.pairMs : 1;
    remote.uvcDeviceIndex = currentConfig.app.camera.uvcDeviceIndex;
    remote.uvcEyeWidth = currentConfig.app.camera.uvcEyeWidth;
    remote.uvcEyeHeight = currentConfig.app.camera.uvcEyeHeight;
    remote.uvcPackedStereo = currentConfig.app.camera.uvcPackedStereo;
    remote.slamInputFps = currentConfig.app.runtime.slamInputFps;
    remote.slamOperationMode = currentConfig.app.runtime.slamOperationMode;
    remote.featureFrontend = currentConfig.app.runtime.featureFrontend == FeatureFrontend::XFeat
                                 ? FeatureFrontend::Orb
                                 : currentConfig.app.runtime.featureFrontend;
    remote.sensorMode = currentConfig.app.sensorMode;
    remote.udpIp = currentConfig.app.udp.ip;
    remote.udpEnabled = currentConfig.app.udp.enable;
    remote.sendImage = currentConfig.app.udp.sendImage;
    remote.sendFeature = currentConfig.app.udp.sendFeature;
    remote.sendMap = currentConfig.app.udp.sendMap;
    remote.useCustomTbc = currentConfig.app.runtime.useCustomTbc;
    remote.tbcTx = currentConfig.app.runtime.tbcTx;
    remote.tbcTy = currentConfig.app.runtime.tbcTy;
    remote.tbcTz = currentConfig.app.runtime.tbcTz;
    remote.tbcRollDeg = currentConfig.app.runtime.tbcRollDeg;
    remote.tbcPitchDeg = currentConfig.app.runtime.tbcPitchDeg;
    remote.tbcYawDeg = currentConfig.app.runtime.tbcYawDeg;
    remote.orbNFeatures = currentConfig.app.runtime.orbNFeatures;
    remote.orbScaleFactor = currentConfig.app.runtime.orbScaleFactor;
    remote.orbNLevels = currentConfig.app.runtime.orbNLevels;
    remote.orbIniThFAST = currentConfig.app.runtime.orbIniThFAST;
    remote.orbMinThFAST = currentConfig.app.runtime.orbMinThFAST;
    remote.xfeatTopK = currentConfig.app.runtime.xfeatTopK;
    remote.xfeatMaxPoints = currentConfig.app.runtime.xfeatMaxPoints;
    remote.xfeatInputMaxWidth = currentConfig.app.runtime.xfeatInputMaxWidth;
    remote.xfeatInputMaxHeight = currentConfig.app.runtime.xfeatInputMaxHeight;
    remote.lkXFeatSeeding = currentConfig.app.runtime.lkXFeatSeeding;
    remote.lkPerFrameAcceleration = currentConfig.app.runtime.lkPerFrameAcceleration;
    return remote;
}

} // namespace smartdrone::core::application
