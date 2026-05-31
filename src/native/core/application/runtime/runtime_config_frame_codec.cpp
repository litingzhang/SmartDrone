#include "core/application/runtime/runtime_config_frame_codec.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "common/tlv/tlv_pack.h"
#include "common/tlv/tlv_protocol.h"
#include "core/application/config/slam_backend_availability.h"

namespace SmartDrone::Core::Application {
namespace {

SensorMode ParseRuntimeSensorMode(std::uint8_t value)
{
    switch (value) {
    case RUNTIME_SENSOR_STEREO_IMU:
        return SensorMode::StereoImu;
    case RUNTIME_SENSOR_MONO:
        return SensorMode::Mono;
    case RUNTIME_SENSOR_MONO_IMU:
        return SensorMode::MonoImu;
    case RUNTIME_SENSOR_STEREO:
    default:
        return SensorMode::Stereo;
    }
}

FeatureFrontend ParseRuntimeFeatureFrontend(std::uint8_t value)
{
    switch (value) {
    case RUNTIME_FEATURE_FRONTEND_XFEAT_LIGHTGLUE:
        return FeatureFrontend::XFeatLightGlue;
    case RUNTIME_FEATURE_FRONTEND_SUPERPOINT_LIGHTGLUE:
        return FeatureFrontend::SuperPointLightGlue;
    case RUNTIME_FEATURE_FRONTEND_LK_GFTT_PER_FRAME:
        return FeatureFrontend::LkGfttPerFrame;
    case RUNTIME_FEATURE_FRONTEND_LK:
        return FeatureFrontend::LK;
    case RUNTIME_FEATURE_FRONTEND_ORB:
        return FeatureFrontend::Orb;
    case RUNTIME_FEATURE_FRONTEND_RESERVED_SUPERPOINT:
    case RUNTIME_FEATURE_FRONTEND_RESERVED_LEGACY:
    default:
        return FeatureFrontend::LkGfttPerFrame;
    }
}

std::string ParseRuntimeLkPerFrameAcceleration(std::uint8_t value)
{
    switch (value) {
    case RUNTIME_LK_PER_FRAME_ACCEL_VPI_CUDA:
        return "vpi-cuda";
    case RUNTIME_LK_PER_FRAME_ACCEL_CPU:
    default:
        return "cpu";
    }
}

std::string ParseRuntimeOrbAcceleration(std::uint8_t value)
{
    switch (value) {
    case RUNTIME_ORB_ACCEL_OPENCV_CUDA:
        return "cuda";
    case RUNTIME_ORB_ACCEL_VPI_REMAP:
        return "vpi-remap";
    case RUNTIME_ORB_ACCEL_CPU:
    default:
        return "cpu";
    }
}

SlamBackend ParseRuntimeSlamBackend(std::uint8_t value)
{
    switch (value) {
    case RUNTIME_SLAM_BACKEND_KLT:
        return SlamBackend::Klt;
    case RUNTIME_SLAM_BACKEND_DPVO_TENSORRT:
        return SlamBackend::DpvoTensorRt;
    case RUNTIME_SLAM_BACKEND_ORBSLAM3:
        return OrbSlam3BackendAvailable() ? SlamBackend::OrbSlam3
                                          : SlamBackend::Klt;
    default:
        return SlamBackend::Klt;
    }
}

SmartDrone::Core::Domain::SlamOperationMode
ParseRuntimeSlamMode(std::uint8_t value)
{
    using SmartDrone::Core::Domain::SlamOperationMode;
    switch (value) {
    case RUNTIME_SLAM_MODE_LOCALIZATION:
        return SlamOperationMode::Localization;
    case RUNTIME_SLAM_MODE_RELOCALIZATION:
        return SlamOperationMode::Relocalization;
    case RUNTIME_SLAM_MODE_TRACKING_ONLY:
        return SlamOperationMode::TrackingOnly;
    case RUNTIME_SLAM_MODE_AUTO:
        return SlamOperationMode::Auto;
    case RUNTIME_SLAM_MODE_MAPPING:
    default:
        return SlamOperationMode::Mapping;
    }
}

void ApplyAvoidanceDefaults(RemoteRuntimeConfig &remote,
                            const RuntimeConfig &runtime)
{
    remote.avoidanceEnabled = runtime.avoidanceEnabled;
    remote.avoidanceHoldOnStaleCloud = runtime.avoidanceHoldOnStaleCloud;
    remote.avoidanceRadiusM = runtime.avoidanceRadiusM;
    remote.avoidanceLookaheadM = runtime.avoidanceLookaheadM;
    remote.avoidanceSpeedLookaheadS = runtime.avoidanceSpeedLookaheadS;
    remote.avoidanceNearFieldRadiusM = runtime.avoidanceNearFieldRadiusM;
    remote.avoidanceMaxPointCloudAgeMs = runtime.avoidanceMaxPointCloudAgeMs;
    remote.avoidanceMinCloudPoints = runtime.avoidanceMinCloudPoints;
    remote.avoidanceMinBlockingPoints = runtime.avoidanceMinBlockingPoints;
}

RemoteRuntimeConfig RuntimeConfigDefaults(const UnifiedConfig &currentCfg)
{
    RemoteRuntimeConfig remote{};
    remote.pairMs =
        currentCfg.app.camera.pairMs > 0 ? currentCfg.app.camera.pairMs : 1;
    remote.autoExposureEnabled = !currentCfg.app.camera.aeDisable;
    remote.slamInputFps = currentCfg.app.runtime.slamInputFps;
    remote.slamOperationMode = currentCfg.app.runtime.slamOperationMode;
    remote.slamBackend = currentCfg.app.runtime.slamBackend;
    remote.featureFrontend = currentCfg.app.runtime.featureFrontend;
    remote.useCustomTbc = currentCfg.app.runtime.useCustomTbc;
    remote.tbcTx = currentCfg.app.runtime.tbcTx;
    remote.tbcTy = currentCfg.app.runtime.tbcTy;
    remote.tbcTz = currentCfg.app.runtime.tbcTz;
    remote.tbcRollDeg = currentCfg.app.runtime.tbcRollDeg;
    remote.tbcPitchDeg = currentCfg.app.runtime.tbcPitchDeg;
    remote.tbcYawDeg = currentCfg.app.runtime.tbcYawDeg;
    remote.orbNFeatures = currentCfg.app.runtime.orbNFeatures;
    remote.orbScaleFactor = currentCfg.app.runtime.orbScaleFactor;
    remote.orbNLevels = currentCfg.app.runtime.orbNLevels;
    remote.orbIniThFAST = currentCfg.app.runtime.orbIniThFAST;
    remote.orbMinThFAST = currentCfg.app.runtime.orbMinThFAST;
    remote.visualFeatureTopK = currentCfg.app.runtime.visualFeatureTopK;
    remote.visualFeatureMaxPoints =
        currentCfg.app.runtime.visualFeatureMaxPoints;
    remote.visualFeatureInputMaxWidth =
        currentCfg.app.runtime.visualFeatureInputMaxWidth;
    remote.visualFeatureInputMaxHeight =
        currentCfg.app.runtime.visualFeatureInputMaxHeight;
    remote.lkPerFrameAcceleration =
        currentCfg.app.runtime.lkPerFrameAcceleration;
    remote.orbAcceleration = currentCfg.app.runtime.orbAcceleration;
    ApplyAvoidanceDefaults(remote, currentCfg.app.runtime);
    return remote;
}

void ApplyRuntimeStreamFlags(RemoteRuntimeConfig &remote,
                             std::uint8_t streamFlags)
{
    if (streamFlags == 0) {
        remote.sendImage = true;
        remote.sendFeature = false;
        remote.sendMap = false;
        return;
    }
    remote.sendImage = (streamFlags & RUNTIME_CFG_FLAG_SEND_IMAGE) != 0;
    remote.sendFeature = (streamFlags & RUNTIME_CFG_FLAG_SEND_FEATURE) != 0;
    remote.sendMap = (streamFlags & RUNTIME_CFG_FLAG_SEND_MAP) != 0;
}

void ApplyRuntimeConfigV2(RemoteRuntimeConfig &remote,
                          const std::uint8_t *payload)
{
    const int pairMs =
        static_cast<int>(ReadU16Le(&payload[RUNTIME_CONFIG_PAIR_MS_OFFSET]));
    if (pairMs > 0) {
        remote.pairMs = pairMs;
    }
    remote.slamInputFps =
        static_cast<int>(ReadU16Le(&payload[RUNTIME_CONFIG_SLAM_FPS_OFFSET]));
}

void ApplyRuntimeConfigV5(RemoteRuntimeConfig &remote,
                          const std::uint8_t *payload)
{
    remote.useCustomTbc =
        payload[RUNTIME_CONFIG_TBC_OVERRIDE_ENABLE_OFFSET] != 0;
    remote.tbcTx = ReadF32Le(&payload[RUNTIME_CONFIG_TBC_TX_OFFSET]);
    remote.tbcTy = ReadF32Le(&payload[RUNTIME_CONFIG_TBC_TY_OFFSET]);
    remote.tbcTz = ReadF32Le(&payload[RUNTIME_CONFIG_TBC_TZ_OFFSET]);
    remote.tbcPitchDeg =
        ReadF32Le(&payload[RUNTIME_CONFIG_TBC_PITCH_DEG_OFFSET]);
}

void ApplyRuntimeConfigV7(RemoteRuntimeConfig &remote,
                          const std::uint8_t *payload)
{
    remote.orbNFeatures = static_cast<int>(
        std::lround(ReadF32Le(&payload[RUNTIME_CONFIG_ORB_NFEATURES_OFFSET])));
    remote.orbScaleFactor =
        ReadF32Le(&payload[RUNTIME_CONFIG_ORB_SCALE_FACTOR_OFFSET]);
    remote.orbNLevels = static_cast<int>(
        std::lround(ReadF32Le(&payload[RUNTIME_CONFIG_ORB_NLEVELS_OFFSET])));
    remote.orbIniThFAST = static_cast<int>(std::lround(
        ReadF32Le(&payload[RUNTIME_CONFIG_ORB_INI_TH_FAST_OFFSET])));
    remote.orbMinThFAST = static_cast<int>(std::lround(
        ReadF32Le(&payload[RUNTIME_CONFIG_ORB_MIN_TH_FAST_OFFSET])));
}

void ApplyRuntimeConfigV9(RemoteRuntimeConfig &remote,
                          const std::uint8_t *payload)
{
    remote.visualFeatureTopK = static_cast<int>(std::lround(
        ReadF32Le(&payload[RUNTIME_CONFIG_SUPERPOINT_TOP_K_OFFSET])));
    remote.visualFeatureMaxPoints = static_cast<int>(std::lround(
        ReadF32Le(&payload[RUNTIME_CONFIG_SUPERPOINT_MAX_POINTS_OFFSET])));
}

void ApplyRuntimeConfigV10(RemoteRuntimeConfig &remote,
                           const std::uint8_t *payload)
{
    remote.visualFeatureInputMaxWidth = static_cast<int>(std::lround(
        ReadF32Le(&payload[RUNTIME_CONFIG_SUPERPOINT_INPUT_MAX_WIDTH_OFFSET])));
    remote.visualFeatureInputMaxHeight = static_cast<int>(std::lround(
        ReadF32Le(&payload[RUNTIME_CONFIG_SUPERPOINT_INPUT_MAX_HEIGHT_OFFSET])));
}

void ApplyRuntimeConfigV12(RemoteRuntimeConfig &remote,
                           const std::uint8_t *payload)
{
    remote.lkPerFrameAcceleration = ParseRuntimeLkPerFrameAcceleration(
        payload[RUNTIME_CONFIG_LK_PER_FRAME_ACCEL_OFFSET]);
}

void ApplyRuntimeConfigV13(RemoteRuntimeConfig &remote,
                           const std::uint8_t *payload)
{
    remote.orbAcceleration =
        ParseRuntimeOrbAcceleration(payload[RUNTIME_CONFIG_ORB_ACCEL_OFFSET]);
}

void ApplyRuntimeConfigV14(RemoteRuntimeConfig &remote,
                           const std::uint8_t *payload)
{
    remote.slamBackend =
        ParseRuntimeSlamBackend(payload[RUNTIME_CONFIG_SLAM_BACKEND_OFFSET]);
}

void ApplyRuntimeConfigV15(RemoteRuntimeConfig &remote,
                           const std::uint8_t *payload)
{
    remote.avoidanceEnabled =
        payload[RUNTIME_CONFIG_AVOIDANCE_ENABLE_OFFSET] != 0;
    remote.avoidanceHoldOnStaleCloud =
        payload[RUNTIME_CONFIG_AVOIDANCE_HOLD_ON_STALE_OFFSET] != 0;
    remote.avoidanceRadiusM =
        ReadF32Le(&payload[RUNTIME_CONFIG_AVOIDANCE_RADIUS_M_OFFSET]);
    remote.avoidanceLookaheadM =
        ReadF32Le(&payload[RUNTIME_CONFIG_AVOIDANCE_LOOKAHEAD_M_OFFSET]);
    remote.avoidanceSpeedLookaheadS =
        ReadF32Le(&payload[RUNTIME_CONFIG_AVOIDANCE_SPEED_LOOKAHEAD_S_OFFSET]);
    remote.avoidanceNearFieldRadiusM =
        ReadF32Le(&payload[RUNTIME_CONFIG_AVOIDANCE_NEAR_FIELD_RADIUS_M_OFFSET]);
    remote.avoidanceMaxPointCloudAgeMs = static_cast<int>(std::lround(
        ReadF32Le(&payload[RUNTIME_CONFIG_AVOIDANCE_MAX_POINT_AGE_MS_OFFSET])));
    remote.avoidanceMinCloudPoints = static_cast<int>(std::lround(
        ReadF32Le(&payload[RUNTIME_CONFIG_AVOIDANCE_MIN_CLOUD_POINTS_OFFSET])));
    remote.avoidanceMinBlockingPoints = static_cast<int>(std::lround(
        ReadF32Le(&payload[RUNTIME_CONFIG_AVOIDANCE_MIN_BLOCKING_POINTS_OFFSET])));
}

void ApplyRuntimeConfigBaseVersions(RemoteRuntimeConfig &remote,
                                    const TlvFrame &frame,
                                    const std::uint8_t *payload)
{
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN) {
        remote.autoExposureEnabled = payload[RUNTIME_CONFIG_AE_OFFSET] != 0;
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V3) {
        remote.slamOperationMode =
            ParseRuntimeSlamMode(payload[RUNTIME_CONFIG_SLAM_MODE_OFFSET]);
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V5) {
        ApplyRuntimeConfigV5(remote, payload);
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V6) {
        remote.tbcRollDeg =
            ReadF32Le(&payload[RUNTIME_CONFIG_TBC_ROLL_DEG_OFFSET]);
        remote.tbcYawDeg =
            ReadF32Le(&payload[RUNTIME_CONFIG_TBC_YAW_DEG_OFFSET]);
    }
}

void ApplyRuntimeConfigFeatureVersions(RemoteRuntimeConfig &remote,
                                       const TlvFrame &frame,
                                       const std::uint8_t *payload)
{
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V7) {
        ApplyRuntimeConfigV7(remote, payload);
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V8) {
        remote.featureFrontend = ParseRuntimeFeatureFrontend(
            payload[RUNTIME_CONFIG_FEATURE_FRONTEND_OFFSET]);
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V9) {
        ApplyRuntimeConfigV9(remote, payload);
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V10) {
        ApplyRuntimeConfigV10(remote, payload);
    }
}

void ApplyRuntimeConfigBackendVersions(RemoteRuntimeConfig &remote,
                                       const TlvFrame &frame,
                                       const std::uint8_t *payload)
{
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V12) {
        ApplyRuntimeConfigV12(remote, payload);
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V13) {
        ApplyRuntimeConfigV13(remote, payload);
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V14) {
        ApplyRuntimeConfigV14(remote, payload);
    }
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V15) {
        ApplyRuntimeConfigV15(remote, payload);
    }
}

std::size_t ApplyVersionedRuntimeConfig(RemoteRuntimeConfig &remote,
                                        const TlvFrame &frame)
{
    const std::uint8_t *payload = frame.payload.data();
    std::size_t ipOffset = 10;
    if (frame.len >= RUNTIME_CONFIG_PAYLOAD_LEN_V2) {
        ApplyRuntimeConfigV2(remote, payload);
        ipOffset = RUNTIME_CONFIG_IP_OFFSET;
    }
    ApplyRuntimeConfigBaseVersions(remote, frame, payload);
    ApplyRuntimeConfigFeatureVersions(remote, frame, payload);
    ApplyRuntimeConfigBackendVersions(remote, frame, payload);
    return ipOffset;
}

void ApplyRuntimeConfigIp(RemoteRuntimeConfig &remote,
                          const std::uint8_t *payload,
                          std::size_t ipOffset)
{
    const char *ipChars = reinterpret_cast<const char *>(&payload[ipOffset]);
    std::size_t ipLen = 0;
    while (ipLen < RUNTIME_CONFIG_IP_LEN && ipChars[ipLen] != '\0') {
        ++ipLen;
    }
    remote.udpIp.assign(ipChars, ipLen);
    remote.udpEnabled = !remote.udpIp.empty();
}

} // namespace

bool RuntimeConfigPayloadLengthValid(std::uint16_t len)
{
    return len == RUNTIME_CONFIG_PAYLOAD_LEN_V15 ||
           len == RUNTIME_CONFIG_PAYLOAD_LEN_V14 ||
           len == RUNTIME_CONFIG_PAYLOAD_LEN_V13 ||
           len == RUNTIME_CONFIG_PAYLOAD_LEN_V12 ||
           len == RUNTIME_CONFIG_PAYLOAD_LEN_V11 ||
           len == RUNTIME_CONFIG_PAYLOAD_LEN_V10 ||
           len == RUNTIME_CONFIG_PAYLOAD_LEN_V9 ||
           len == RUNTIME_CONFIG_PAYLOAD_LEN_V8 ||
           len == RUNTIME_CONFIG_PAYLOAD_LEN_V7 ||
           len == RUNTIME_CONFIG_PAYLOAD_LEN_V6 ||
           len == RUNTIME_CONFIG_PAYLOAD_LEN_V5 ||
           len == RUNTIME_CONFIG_PAYLOAD_LEN ||
           len == RUNTIME_CONFIG_PAYLOAD_LEN_V3 ||
           len == RUNTIME_CONFIG_PAYLOAD_LEN_V2 ||
           len == RUNTIME_CONFIG_PAYLOAD_LEN_LEGACY;
}

RemoteRuntimeConfig ParseRuntimeConfigFrame(const TlvFrame &frame,
                                            const UnifiedConfig &currentCfg)
{
    const std::uint8_t *payload = frame.payload.data();
    RemoteRuntimeConfig remote = RuntimeConfigDefaults(currentCfg);
    remote.exposureUs = static_cast<int>(ReadU32Le(&payload[0]));
    remote.gain = ReadF32Le(&payload[4]);
    remote.sensorMode = ParseRuntimeSensorMode(payload[8]);
    ApplyRuntimeStreamFlags(remote, payload[9]);
    const std::size_t ipOffset = ApplyVersionedRuntimeConfig(remote, frame);
    ApplyRuntimeConfigIp(remote, payload, ipOffset);
    return remote;
}

void ApplyConfigPeerIp(RemoteRuntimeConfig &remote,
                       const std::string &peerIp)
{
    if (peerIp.empty()) {
        return;
    }
    remote.udpIp = peerIp;
    remote.udpEnabled = true;
}

std::string BuildRuntimeConfigAckMessage(const std::string &message,
                                         const RemoteRuntimeConfig &remote)
{
    return message + " udp=" + remote.udpIp + " settings=" +
           std::string(DefaultSettingsForSlamBackend(
               NormalizeSensorModeForSlamBackend(remote.sensorMode,
                                                 remote.slamBackend),
               remote.slamBackend)) +
           " backend=" + std::string(ToSlamBackendText(remote.slamBackend)) +
           " frontend=" +
           std::string(ToFeatureFrontendText(remote.featureFrontend)) +
           " slam_mode=" + std::string(SmartDrone::Core::Domain::ToString(remote.slamOperationMode)) +
           " img=" + (remote.sendImage ? "on" : "off") +
           " feat=" + (remote.sendFeature ? "on" : "off") +
           " map=" + (remote.sendMap ? "on" : "off") +
           " ae=" + (remote.autoExposureEnabled ? "on" : "off") +
           " tbc_override=" + (remote.useCustomTbc ? "on" : "off") +
           " lk_seed=gftt" + " lk_accel=" +
           remote.lkPerFrameAcceleration +
           " orb_accel=" + remote.orbAcceleration +
           " avoid=" + (remote.avoidanceEnabled ? "on" : "off");
}

} // namespace SmartDrone::Core::Application
