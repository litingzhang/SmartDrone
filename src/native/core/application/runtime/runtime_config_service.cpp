#include "core/application/runtime/runtime_config_service.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <variant>

#include "core/application/config/app_args.h"
#include "core/application/config/config_registry.h"
#include "core/application/config/orb_acceleration_config.h"
#include "core/application/session/slam/slam_settings_loader.h"

namespace SmartDrone::Core::Application {
namespace {

bool ValidateOrbExtractorConfig(const RemoteRuntimeConfig &remote,
                                std::string *err)
{
    if (remote.orbNFeatures <= 0 || !(remote.orbScaleFactor > 0.0f) ||
        remote.orbNLevels <= 0 || remote.orbIniThFAST <= 0 ||
        remote.orbMinThFAST <= 0) {
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
    if (remote.orbNFeatures < 100 || remote.orbNFeatures > 5000 ||
        remote.orbScaleFactor < 1.01f || remote.orbScaleFactor > 3.0f ||
        remote.orbNLevels < 1 || remote.orbNLevels > 16 ||
        remote.orbIniThFAST > 100 || remote.orbMinThFAST > 100) {
        if (err) {
            *err = "orb extractor config out of range";
        }
        return false;
    }
    return true;
}

void SetDefaultOrbExtractorConfig(RemoteRuntimeConfig &remote)
{
    remote.orbNFeatures = 1200;
    remote.orbScaleFactor = 1.2f;
    remote.orbNLevels = 8;
    remote.orbIniThFAST = 16;
    remote.orbMinThFAST = 6;
}

struct RuntimeTbcValues {
    float tx{0.0f};
    float ty{0.0f};
    float tz{0.0f};
    float rollDeg{0.0f};
    float pitchDeg{0.0f};
    float yawDeg{0.0f};
};

struct AppliedRuntimeConfig {
    RuntimeTbcValues tbc{};
    bool restartNeeded{false};
};

CommandResult OkResult()
{
    return {true, ""};
}

CommandResult TypeMismatchResult(const char *key)
{
    return {false, std::string(key) + " type mismatch"};
}

CommandResult AssignStrictInt(const ConfigValue &value, int &out,
                              const char *key)
{
    if (const auto *typedValue = std::get_if<int64_t>(&value)) {
        out = static_cast<int>(*typedValue);
        return OkResult();
    }
    return TypeMismatchResult(key);
}

CommandResult AssignNumericInt(const ConfigValue &value, int &out,
                               const char *key)
{
    if (const auto *typedValue = std::get_if<int64_t>(&value)) {
        out = static_cast<int>(*typedValue);
        return OkResult();
    }
    if (const auto *typedValue = std::get_if<double>(&value)) {
        out = static_cast<int>(*typedValue);
        return OkResult();
    }
    return TypeMismatchResult(key);
}

CommandResult AssignFloat(const ConfigValue &value, float &out,
                          const char *key)
{
    if (const auto *typedValue = std::get_if<double>(&value)) {
        out = static_cast<float>(*typedValue);
        return OkResult();
    }
    if (const auto *typedValue = std::get_if<int64_t>(&value)) {
        out = static_cast<float>(*typedValue);
        return OkResult();
    }
    return TypeMismatchResult(key);
}

CommandResult AssignBool(const ConfigValue &value, bool &out,
                         const char *key)
{
    if (const auto *typedValue = std::get_if<bool>(&value)) {
        out = *typedValue;
        return OkResult();
    }
    return TypeMismatchResult(key);
}

CommandResult AssignString(const ConfigValue &value, std::string &out,
                           const char *key)
{
    if (const auto *typedValue = std::get_if<std::string>(&value)) {
        out = *typedValue;
        return OkResult();
    }
    return TypeMismatchResult(key);
}

void NormalizeRemoteRuntimeConfig(RemoteRuntimeConfig &remote)
{
    remote.orbAcceleration = NormalizeOrbAcceleration(remote.orbAcceleration);
    remote.slamBackend = NormalizeSlamBackendForBuild(remote.slamBackend);
    if (remote.slamBackend == SlamBackend::DpvoTensorRt) {
        remote.featureFrontend = FeatureFrontend::LkGfttPerFrame;
        remote.lkPerFrameAcceleration = "cpu";
    }
    if (remote.slamBackend == SlamBackend::Klt &&
        remote.featureFrontend != FeatureFrontend::LK &&
        remote.featureFrontend != FeatureFrontend::LkGfttPerFrame) {
        remote.featureFrontend = FeatureFrontend::LkGfttPerFrame;
    }
    if (remote.slamBackend != SlamBackend::OrbSlam3 ||
        remote.featureFrontend != FeatureFrontend::Orb) {
        remote.orbAcceleration = "cpu";
    }
}

bool SetError(std::string *err, const char *message)
{
    if (err) {
        *err = message;
    }
    return false;
}

bool ValidateBasicRuntimeConfig(const RemoteRuntimeConfig &remote,
                                std::string *err)
{
    if (remote.exposureUs <= 0 || !std::isfinite(remote.gain) ||
        remote.gain < 0.0f || remote.pairMs <= 0 || remote.slamInputFps < 0 ||
        remote.uvcDeviceIndex < 0 || remote.uvcEyeWidth <= 0 ||
        remote.uvcEyeHeight <= 0) {
        return SetError(err, "bad runtime config");
    }
    return true;
}

bool ValidateTbcConfig(const RemoteRuntimeConfig &remote, std::string *err)
{
    if (!std::isfinite(remote.tbcTx) || !std::isfinite(remote.tbcTy) ||
        !std::isfinite(remote.tbcTz) || !std::isfinite(remote.tbcRollDeg) ||
        !std::isfinite(remote.tbcPitchDeg) || !std::isfinite(remote.tbcYawDeg)) {
        return SetError(err, "bad tbc override config");
    }
    return true;
}

bool ValidateVisualFeatureConfig(const RemoteRuntimeConfig &remote,
                                 std::string *err)
{
    if (remote.visualFeatureTopK < 1 || remote.visualFeatureTopK > 4096 ||
        remote.visualFeatureMaxPoints < 1 ||
        remote.visualFeatureMaxPoints > 4096 ||
        remote.visualFeatureMaxPoints > remote.visualFeatureTopK) {
        return SetError(err, "visual feature config out of range");
    }
    if (remote.visualFeatureInputMaxWidth < 0 ||
        remote.visualFeatureInputMaxWidth > 4096 ||
        remote.visualFeatureInputMaxHeight < 0 ||
        remote.visualFeatureInputMaxHeight > 4096) {
        return SetError(err, "visual feature input size config out of range");
    }
    return true;
}

bool ValidateAccelerationConfig(const RemoteRuntimeConfig &remote,
                                std::string *err)
{
    if (remote.lkPerFrameAcceleration != "cpu" &&
        remote.lkPerFrameAcceleration != "vpi-cuda" &&
        remote.lkPerFrameAcceleration != "auto") {
        return SetError(err, "bad lk per-frame acceleration");
    }
    if (remote.orbAcceleration != "cpu" && remote.orbAcceleration != "cuda" &&
        remote.orbAcceleration != "vpi-remap") {
        return SetError(err, "bad orb acceleration");
    }
    return true;
}

bool ValidateRemoteRuntimeConfig(RemoteRuntimeConfig &remote,
                                 std::string *err)
{
    const bool usesOrbBackend = remote.slamBackend == SlamBackend::OrbSlam3;
    if (!ValidateBasicRuntimeConfig(remote, err) ||
        !ValidateTbcConfig(remote, err)) {
        return false;
    }
    if (usesOrbBackend && !ValidateOrbExtractorConfig(remote, err)) {
        return false;
    }
    if (!usesOrbBackend && !ValidateOrbExtractorConfig(remote, nullptr)) {
        SetDefaultOrbExtractorConfig(remote);
    }
    return ValidateVisualFeatureConfig(remote, err) &&
           ValidateAccelerationConfig(remote, err);
}

RuntimeTbcValues ReadRuntimeTbc(const RuntimeConfig &runtime)
{
    return {runtime.tbcTx, runtime.tbcTy, runtime.tbcTz,
            runtime.tbcRollDeg, runtime.tbcPitchDeg, runtime.tbcYawDeg};
}

bool OrbParamsAffectPipeline(const AppConfig &app,
                             const RemoteRuntimeConfig &remote)
{
    return app.runtime.slamBackend == SlamBackend::OrbSlam3 ||
           remote.slamBackend == SlamBackend::OrbSlam3;
}

bool OrbConfigChanged(const AppConfig &app, const RemoteRuntimeConfig &remote)
{
    if (!OrbParamsAffectPipeline(app, remote)) {
        return false;
    }
    return app.runtime.orbNFeatures != remote.orbNFeatures ||
           std::abs(app.runtime.orbScaleFactor - remote.orbScaleFactor) > 1e-6f ||
           app.runtime.orbNLevels != remote.orbNLevels ||
           app.runtime.orbIniThFAST != remote.orbIniThFAST ||
           app.runtime.orbMinThFAST != remote.orbMinThFAST;
}

bool VisualFeatureConfigChanged(const AppConfig &app,
                                const RemoteRuntimeConfig &remote)
{
    return app.runtime.visualFeatureTopK != remote.visualFeatureTopK ||
           app.runtime.visualFeatureMaxPoints != remote.visualFeatureMaxPoints ||
           app.runtime.visualFeatureInputMaxWidth !=
               remote.visualFeatureInputMaxWidth ||
           app.runtime.visualFeatureInputMaxHeight !=
               remote.visualFeatureInputMaxHeight;
}

bool RuntimeRestartNeeded(const AppConfig &app,
                          const RemoteRuntimeConfig &remote)
{
    const CameraConfig &cam = app.camera;
    const bool uvcConfigChanged = cam.uvcDeviceIndex != remote.uvcDeviceIndex ||
                                  cam.uvcEyeWidth != remote.uvcEyeWidth ||
                                  cam.uvcEyeHeight != remote.uvcEyeHeight ||
                                  cam.uvcPackedStereo != remote.uvcPackedStereo;
    return app.sensorMode != remote.sensorMode ||
           app.runtime.slamBackend != remote.slamBackend ||
           app.runtime.featureFrontend != remote.featureFrontend ||
           cam.aeDisable != (!remote.autoExposureEnabled) || uvcConfigChanged ||
           OrbConfigChanged(app, remote) ||
           VisualFeatureConfigChanged(app, remote) ||
           app.runtime.lkPerFrameAcceleration != remote.lkPerFrameAcceleration ||
           (OrbParamsAffectPipeline(app, remote) &&
            app.runtime.orbAcceleration != remote.orbAcceleration);
}

void ApplyCameraConfig(CameraConfig &camera, const RemoteRuntimeConfig &remote)
{
    camera.exposureUs = remote.exposureUs;
    camera.gain = remote.gain;
    camera.aeDisable = !remote.autoExposureEnabled;
    camera.pairMs = remote.pairMs;
    camera.uvcDeviceIndex = remote.uvcDeviceIndex;
    camera.uvcEyeWidth = remote.uvcEyeWidth;
    camera.uvcEyeHeight = remote.uvcEyeHeight;
    camera.uvcPackedStereo = remote.uvcPackedStereo;
}

void ApplyRuntimeConfig(RuntimeConfig &runtime,
                        const RemoteRuntimeConfig &remote)
{
    runtime.slamInputFps = remote.slamInputFps;
    runtime.slamOperationMode = remote.slamOperationMode;
    runtime.slamBackend = remote.slamBackend;
    runtime.featureFrontend = remote.featureFrontend;
    runtime.orbNFeatures = remote.orbNFeatures;
    runtime.orbScaleFactor = remote.orbScaleFactor;
    runtime.orbNLevels = remote.orbNLevels;
    runtime.orbIniThFAST = remote.orbIniThFAST;
    runtime.orbMinThFAST = remote.orbMinThFAST;
    runtime.visualFeatureTopK = remote.visualFeatureTopK;
    runtime.visualFeatureMaxPoints = remote.visualFeatureMaxPoints;
    runtime.visualFeatureInputMaxWidth = remote.visualFeatureInputMaxWidth;
    runtime.visualFeatureInputMaxHeight = remote.visualFeatureInputMaxHeight;
    runtime.lkPerFrameAcceleration = remote.lkPerFrameAcceleration;
    runtime.orbAcceleration = remote.orbAcceleration;
    runtime.useCustomTbc = remote.useCustomTbc;
}

void ApplyConfiguredTbc(RuntimeConfig &runtime,
                        const RemoteRuntimeConfig &remote)
{
    runtime.tbcTx = remote.tbcTx;
    runtime.tbcTy = remote.tbcTy;
    runtime.tbcTz = remote.tbcTz;
    runtime.tbcRollDeg = remote.tbcRollDeg;
    runtime.tbcPitchDeg = remote.tbcPitchDeg;
    runtime.tbcYawDeg = remote.tbcYawDeg;
}

void ApplyCalibratedTbc(RuntimeConfig &runtime,
                        const StereoBodyExtrinsics &extrinsics)
{
    const Eigen::Vector3f t = extrinsics.Tbc.translation();
    runtime.tbcTx = t.x();
    runtime.tbcTy = t.y();
    runtime.tbcTz = t.z();
    runtime.tbcRollDeg = 0.0f;
    runtime.tbcPitchDeg = 0.0f;
    runtime.tbcYawDeg = 0.0f;
}

void ApplyTbcConfig(AppConfig &app, const RemoteRuntimeConfig &remote)
{
    if (remote.useCustomTbc) {
        ApplyConfiguredTbc(app.runtime, remote);
        return;
    }
    const auto extrinsics = LoadStereoBodyExtrinsics(app.settings);
    if (extrinsics.loaded) {
        ApplyCalibratedTbc(app.runtime, extrinsics);
        return;
    }
    ApplyConfiguredTbc(app.runtime, remote);
}

void ApplyUdpConfig(UdpConfig &udp, const RemoteRuntimeConfig &remote)
{
    udp.ip = remote.udpIp;
    udp.enable = remote.udpEnabled;
    udp.sendImage = remote.sendImage;
    udp.sendFeature = remote.sendFeature;
    udp.sendMap = remote.sendMap;
}

AppliedRuntimeConfig ApplyRemoteRuntimeConfig(UnifiedConfig &config,
                                              const RemoteRuntimeConfig &remote)
{
    AppliedRuntimeConfig applied{};
    applied.restartNeeded = RuntimeRestartNeeded(config.app, remote);
    ApplyCameraConfig(config.app.camera, remote);
    ApplyRuntimeConfig(config.app.runtime, remote);
    config.app.sensorMode = remote.sensorMode;
    config.app.settings =
        ResolveSettingsForSensorMode(remote.sensorMode, config.app.settings);
    ApplyUdpConfig(config.app.udp, remote);
    ApplyTbcConfig(config.app, remote);
    applied.tbc = ReadRuntimeTbc(config.app.runtime);
    return applied;
}

void SyncRuntimeTuning(LiveRuntimeTuning &tuning,
                       const RemoteRuntimeConfig &remote,
                       const RuntimeTbcValues &tbc)
{
    tuning.slamInputFps.store(remote.slamInputFps, std::memory_order_relaxed);
    tuning.slamOperationMode.store(static_cast<uint8_t>(remote.slamOperationMode),
                                   std::memory_order_relaxed);
    tuning.featureFrontend.store(static_cast<uint8_t>(remote.featureFrontend),
                                 std::memory_order_relaxed);
    tuning.sendImage.store(remote.sendImage, std::memory_order_relaxed);
    tuning.sendFeature.store(remote.sendFeature, std::memory_order_relaxed);
    tuning.sendMap.store(remote.sendMap, std::memory_order_relaxed);
    tuning.useCustomTbc.store(remote.useCustomTbc, std::memory_order_relaxed);
    tuning.tbcTx.store(tbc.tx, std::memory_order_relaxed);
    tuning.tbcTy.store(tbc.ty, std::memory_order_relaxed);
    tuning.tbcTz.store(tbc.tz, std::memory_order_relaxed);
    tuning.tbcRollDeg.store(tbc.rollDeg, std::memory_order_relaxed);
    tuning.tbcPitchDeg.store(tbc.pitchDeg, std::memory_order_relaxed);
    tuning.tbcYawDeg.store(tbc.yawDeg, std::memory_order_relaxed);
}

CommandResult ApplyCameraConfigValue(const std::string &key,
                                     const ConfigValue &value,
                                     RemoteRuntimeConfig &remote,
                                     bool &handled)
{
    handled = true;
    if (key == ConfigRegistry::CAMERA_EXPOSURE_US) {
        return AssignStrictInt(value, remote.exposureUs, "camera.exposure_us");
    }
    if (key == ConfigRegistry::CAMERA_GAIN) {
        return AssignFloat(value, remote.gain, "camera.gain");
    }
    if (key == ConfigRegistry::CAMERA_AUTO_EXPOSURE) {
        return AssignBool(value, remote.autoExposureEnabled, "camera.auto_exposure");
    }
    if (key == ConfigRegistry::CAMERA_PAIR_WINDOW_MS) {
        return AssignStrictInt(value, remote.pairMs, "camera.pair_window_ms");
    }
    if (key == ConfigRegistry::CAMERA_UVC_DEVICE_INDEX) {
        return AssignStrictInt(value, remote.uvcDeviceIndex,
                               "camera.uvc_device_index");
    }
    if (key == ConfigRegistry::CAMERA_UVC_EYE_WIDTH) {
        return AssignStrictInt(value, remote.uvcEyeWidth, "camera.uvc_eye_width");
    }
    if (key == ConfigRegistry::CAMERA_UVC_EYE_HEIGHT) {
        return AssignStrictInt(value, remote.uvcEyeHeight, "camera.uvc_eye_height");
    }
    if (key == ConfigRegistry::CAMERA_UVC_PACKED_STEREO) {
        return AssignBool(value, remote.uvcPackedStereo, "camera.uvc_packed_stereo");
    }
    handled = false;
    return OkResult();
}

CommandResult ApplySlamModeConfigValue(const std::string &key,
                                       const ConfigValue &value,
                                       RemoteRuntimeConfig &remote,
                                       bool &handled)
{
    handled = true;
    if (key == ConfigRegistry::SLAM_INPUT_FPS) {
        return AssignStrictInt(value, remote.slamInputFps, "slam.input_fps");
    }
    if (key == ConfigRegistry::SLAM_BACKEND) {
        if (const auto *text = std::get_if<std::string>(&value)) {
            remote.slamBackend =
                NormalizeSlamBackendForBuild(ParseSlamBackendText(*text));
            return OkResult();
        }
        return TypeMismatchResult("slam.backend");
    }
    if (key == ConfigRegistry::SLAM_FEATURE_FRONTEND) {
        if (const auto *text = std::get_if<std::string>(&value)) {
            remote.featureFrontend = ParseFeatureFrontendText(*text);
            return OkResult();
        }
        return TypeMismatchResult("slam.feature_frontend");
    }
    if (key == ConfigRegistry::SLAM_OPERATION_MODE) {
        if (const auto *text = std::get_if<std::string>(&value)) {
            remote.slamOperationMode = ParseSlamOperationModeText(*text);
            return OkResult();
        }
        return TypeMismatchResult("slam.operation_mode");
    }
    if (key == ConfigRegistry::SLAM_PERCEPTION_MODE) {
        if (const auto *text = std::get_if<std::string>(&value)) {
            remote.sensorMode = ParseSensorModeText(*text);
            return OkResult();
        }
        return TypeMismatchResult("slam.perception_mode");
    }
    handled = false;
    return OkResult();
}

CommandResult ApplyStreamConfigValue(const std::string &key,
                                     const ConfigValue &value,
                                     RemoteRuntimeConfig &remote,
                                     bool &handled)
{
    handled = true;
    if (key == ConfigRegistry::STREAM_UDP_ENABLED) {
        return AssignBool(value, remote.udpEnabled, "stream.udp_enabled");
    }
    if (key == ConfigRegistry::STREAM_UDP_IP) {
        return AssignString(value, remote.udpIp, "stream.udp_ip");
    }
    if (key == ConfigRegistry::STREAM_SEND_IMAGE) {
        return AssignBool(value, remote.sendImage, "stream.send_image");
    }
    if (key == ConfigRegistry::STREAM_SEND_FEATURE) {
        return AssignBool(value, remote.sendFeature, "stream.send_feature");
    }
    if (key == ConfigRegistry::STREAM_SEND_MAP) {
        return AssignBool(value, remote.sendMap, "stream.send_map");
    }
    handled = false;
    return OkResult();
}

CommandResult ApplyTbcConfigValue(const std::string &key,
                                  const ConfigValue &value,
                                  RemoteRuntimeConfig &remote,
                                  bool &handled)
{
    handled = true;
    if (key == ConfigRegistry::SLAM_USE_CUSTOM_TBC) {
        return AssignBool(value, remote.useCustomTbc, "slam.tbc_override_enabled");
    }
    if (key == ConfigRegistry::SLAM_TBC_TX) {
        return AssignFloat(value, remote.tbcTx, "slam.tbc_tx_m");
    }
    if (key == ConfigRegistry::SLAM_TBC_TY) {
        return AssignFloat(value, remote.tbcTy, "slam.tbc_ty_m");
    }
    if (key == ConfigRegistry::SLAM_TBC_TZ) {
        return AssignFloat(value, remote.tbcTz, "slam.tbc_tz_m");
    }
    if (key == ConfigRegistry::SLAM_TBC_ROLL_DEG) {
        return AssignFloat(value, remote.tbcRollDeg, "slam.tbc_roll_deg");
    }
    if (key == ConfigRegistry::SLAM_TBC_PITCH_DEG) {
        return AssignFloat(value, remote.tbcPitchDeg, "slam.tbc_pitch_deg");
    }
    if (key == ConfigRegistry::SLAM_TBC_YAW_DEG) {
        return AssignFloat(value, remote.tbcYawDeg, "slam.tbc_yaw_deg");
    }
    handled = false;
    return OkResult();
}

CommandResult ApplyOrbConfigValue(const std::string &key,
                                  const ConfigValue &value,
                                  RemoteRuntimeConfig &remote,
                                  bool &handled)
{
    handled = true;
    if (key == ConfigRegistry::SLAM_ORB_N_FEATURES) {
        return AssignNumericInt(value, remote.orbNFeatures, "slam.orb_nfeatures");
    }
    if (key == ConfigRegistry::SLAM_ORB_SCALE_FACTOR) {
        return AssignFloat(value, remote.orbScaleFactor, "slam.orb_scale_factor");
    }
    if (key == ConfigRegistry::SLAM_ORB_N_LEVELS) {
        return AssignNumericInt(value, remote.orbNLevels, "slam.orb_nlevels");
    }
    if (key == ConfigRegistry::SLAM_ORB_INI_TH_FAST) {
        return AssignNumericInt(value, remote.orbIniThFAST, "slam.orb_ini_th_fast");
    }
    if (key == ConfigRegistry::SLAM_ORB_MIN_TH_FAST) {
        return AssignNumericInt(value, remote.orbMinThFAST, "slam.orb_min_th_fast");
    }
    handled = false;
    return OkResult();
}

CommandResult ApplyVisualFeatureConfigValue(const std::string &key,
                                            const ConfigValue &value,
                                            RemoteRuntimeConfig &remote,
                                            bool &handled)
{
    handled = true;
    if (key == ConfigRegistry::SLAM_VISUAL_FEATURE_TOP_K ||
        key == ConfigRegistry::SLAM_SUPER_POINT_TOP_K) {
        return AssignNumericInt(value, remote.visualFeatureTopK,
                                "slam.visual_feature_top_k");
    }
    if (key == ConfigRegistry::SLAM_VISUAL_FEATURE_MAX_POINTS ||
        key == ConfigRegistry::SLAM_SUPER_POINT_MAX_POINTS) {
        return AssignNumericInt(value, remote.visualFeatureMaxPoints,
                                "slam.visual_feature_max_points");
    }
    if (key == ConfigRegistry::SLAM_VISUAL_FEATURE_INPUT_MAX_WIDTH ||
        key == ConfigRegistry::SLAM_SUPER_POINT_INPUT_MAX_WIDTH) {
        return AssignNumericInt(value, remote.visualFeatureInputMaxWidth,
                                "slam.visual_feature_input_max_width");
    }
    if (key == ConfigRegistry::SLAM_VISUAL_FEATURE_INPUT_MAX_HEIGHT ||
        key == ConfigRegistry::SLAM_SUPER_POINT_INPUT_MAX_HEIGHT) {
        return AssignNumericInt(value, remote.visualFeatureInputMaxHeight,
                                "slam.visual_feature_input_max_height");
    }
    handled = false;
    return OkResult();
}

CommandResult ApplyAccelerationConfigValue(const std::string &key,
                                           const ConfigValue &value,
                                           RemoteRuntimeConfig &remote,
                                           bool &handled)
{
    handled = true;
    if (key == ConfigRegistry::SLAM_LK_SUPER_POINT_SEEDING) {
        if (std::holds_alternative<bool>(value)) {
            return OkResult();
        }
        return TypeMismatchResult("slam.lk_superpoint_seeding");
    }
    if (key == ConfigRegistry::SLAM_LK_PER_FRAME_ACCELERATION) {
        return AssignString(value, remote.lkPerFrameAcceleration,
                            "slam.lk_per_frame_accel");
    }
    if (key == ConfigRegistry::SLAM_ORB_ACCELERATION) {
        return AssignString(value, remote.orbAcceleration, "slam.orb_accel");
    }
    handled = false;
    return OkResult();
}

CommandResult ApplyConfigValue(const std::string &key,
                               const ConfigValue &value,
                               RemoteRuntimeConfig &remote)
{
    bool handled = false;
    CommandResult result = ApplyCameraConfigValue(key, value, remote, handled);
    if (handled || !result.ok) {
        return result;
    }
    result = ApplySlamModeConfigValue(key, value, remote, handled);
    if (handled || !result.ok) {
        return result;
    }
    result = ApplyStreamConfigValue(key, value, remote, handled);
    if (handled || !result.ok) {
        return result;
    }
    result = ApplyTbcConfigValue(key, value, remote, handled);
    if (handled || !result.ok) {
        return result;
    }
    result = ApplyOrbConfigValue(key, value, remote, handled);
    if (handled || !result.ok) {
        return result;
    }
    result = ApplyVisualFeatureConfigValue(key, value, remote, handled);
    if (handled || !result.ok) {
        return result;
    }
    result = ApplyAccelerationConfigValue(key, value, remote, handled);
    if (handled || !result.ok) {
        return result;
    }
    return {false, "unsupported config key: " + key};
}

std::string BuildRuntimeConfigMessage(const RemoteRuntimeConfig &remote,
                                      const UnifiedConfig &currentConfig)
{
    const int cameraFps =
        currentConfig.app.camera.fps > 0 ? currentConfig.app.camera.fps : 1;
    const int clampedSlamFps = remote.slamInputFps <= 0
                                   ? cameraFps
                                   : std::min(cameraFps, remote.slamInputFps);
    return "runtime cfg updated sensor=" +
           std::string(ToSensorModeText(remote.sensorMode)) +
           " backend=" + std::string(ToSlamBackendText(remote.slamBackend)) +
           " frontend=" +
           std::string(ToFeatureFrontendText(remote.featureFrontend)) +
           " slam_mode=" +
           std::string(Domain::ToString(remote.slamOperationMode)) +
           " slam_fps=" + std::to_string(clampedSlamFps) +
           " pair_ms=" + std::to_string(remote.pairMs) +
           " provider_specific visual_feature_top_k=" +
           std::to_string(remote.visualFeatureTopK) +
           " visual_feature_max_points=" +
           std::to_string(remote.visualFeatureMaxPoints) +
           " visual_feature_input_max=" +
           std::to_string(remote.visualFeatureInputMaxWidth) + "x" +
           std::to_string(remote.visualFeatureInputMaxHeight) +
           " lk_seed=gftt lk_accel=" + remote.lkPerFrameAcceleration +
           " orb_accel=" + remote.orbAcceleration;
}

} // namespace

RuntimeConfigService::RuntimeConfigService(std::shared_ptr<const UnifiedConfig> &config,
                                           LiveRuntimeTuning &tuning,
                                           RestartFn requestRestart)
    : m_config(config), m_tuning(tuning),
      m_requestRestart(std::move(requestRestart))
{
}

bool RuntimeConfigService::UpdateRemoteConfig(RemoteRuntimeConfig remote,
                                              std::string *err)
{
    NormalizeRemoteRuntimeConfig(remote);
    if (!ValidateRemoteRuntimeConfig(remote, err)) {
        return false;
    }

    AppliedRuntimeConfig applied{};
    while (true) {
        std::shared_ptr<const UnifiedConfig> current = LoadConfig();
        UnifiedConfig nextConfig = current ? *current : UnifiedConfig{};
        applied = ApplyRemoteRuntimeConfig(nextConfig, remote);
        auto next = std::make_shared<const UnifiedConfig>(std::move(nextConfig));
        if (ReplaceConfig(current, std::move(next))) {
            break;
        }
    }

    if (remote.slamBackend == SlamBackend::OrbSlam3) {
        ApplyOrbAccelerationEnvironment(remote.orbAcceleration);
    }

    SyncRuntimeTuning(m_tuning, remote, applied.tbc);

    if (applied.restartNeeded && m_requestRestart) {
        m_requestRestart();
    }
    return true;
}

CommandResult
RuntimeConfigService::ApplyConfig(const ConfigUpdate &update,
                                  const UnifiedConfig &currentConfig)
{
    RemoteRuntimeConfig remote = BuildRemoteConfig(currentConfig);

    for (const auto &[key, value] : update.values) {
        const CommandResult result = ApplyConfigValue(key, value, remote);
        if (!result.ok) {
            return result;
        }
    }

    std::string err;
    if (!UpdateRemoteConfig(remote, &err)) {
        return {false, err.empty() ? "runtime cfg failed" : err};
    }

    return {true, BuildRuntimeConfigMessage(remote, currentConfig)};
}

RemoteRuntimeConfig
RuntimeConfigService::BuildRemoteConfig(const UnifiedConfig &currentConfig)
{
    RemoteRuntimeConfig remote{};
    remote.exposureUs = currentConfig.app.camera.exposureUs;
    remote.gain = currentConfig.app.camera.gain;
    remote.autoExposureEnabled = !currentConfig.app.camera.aeDisable;
    remote.pairMs =
        currentConfig.app.camera.pairMs > 0 ? currentConfig.app.camera.pairMs : 1;
    remote.uvcDeviceIndex = currentConfig.app.camera.uvcDeviceIndex;
    remote.uvcEyeWidth = currentConfig.app.camera.uvcEyeWidth;
    remote.uvcEyeHeight = currentConfig.app.camera.uvcEyeHeight;
    remote.uvcPackedStereo = currentConfig.app.camera.uvcPackedStereo;
    remote.slamInputFps = currentConfig.app.runtime.slamInputFps;
    remote.slamOperationMode = currentConfig.app.runtime.slamOperationMode;
    remote.slamBackend = currentConfig.app.runtime.slamBackend;
    remote.featureFrontend = currentConfig.app.runtime.featureFrontend;
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
    remote.visualFeatureTopK = currentConfig.app.runtime.visualFeatureTopK;
    remote.visualFeatureMaxPoints =
        currentConfig.app.runtime.visualFeatureMaxPoints;
    remote.visualFeatureInputMaxWidth =
        currentConfig.app.runtime.visualFeatureInputMaxWidth;
    remote.visualFeatureInputMaxHeight =
        currentConfig.app.runtime.visualFeatureInputMaxHeight;
    remote.lkPerFrameAcceleration =
        currentConfig.app.runtime.lkPerFrameAcceleration;
    remote.orbAcceleration = currentConfig.app.runtime.orbAcceleration;
    return remote;
}

std::shared_ptr<const UnifiedConfig> RuntimeConfigService::LoadConfig() const
{
    return std::atomic_load_explicit(&m_config, std::memory_order_acquire);
}

bool RuntimeConfigService::ReplaceConfig(
    std::shared_ptr<const UnifiedConfig> &expected,
    std::shared_ptr<const UnifiedConfig> next)
{
    return std::atomic_compare_exchange_weak_explicit(&m_config, &expected, std::move(next),
                                                      std::memory_order_acq_rel,
                                                      std::memory_order_acquire);
}

} // namespace SmartDrone::Core::Application
