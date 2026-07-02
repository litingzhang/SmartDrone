#include "core/application/runtime/runtime_config_application.h"

#include <atomic>
#include <cmath>

#include "core/application/config/app_args.h"
#include "core/application/session/slam/slam_settings_loader.h"

namespace SmartDrone::Core::Application {
namespace {

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
    runtime.avoidanceEnabled = remote.avoidanceEnabled;
    runtime.avoidanceHoldOnStaleCloud =
        remote.avoidanceHoldOnStaleCloud;
    runtime.avoidanceRadiusM = remote.avoidanceRadiusM;
    runtime.avoidanceLookaheadM = remote.avoidanceLookaheadM;
    runtime.avoidanceSpeedLookaheadS = remote.avoidanceSpeedLookaheadS;
    runtime.avoidanceNearFieldRadiusM =
        remote.avoidanceNearFieldRadiusM;
    runtime.avoidanceMaxPointCloudAgeMs =
        remote.avoidanceMaxPointCloudAgeMs;
    runtime.avoidanceMinCloudPoints = remote.avoidanceMinCloudPoints;
    runtime.avoidanceMinBlockingPoints =
        remote.avoidanceMinBlockingPoints;
    runtime.px4PoseOutputMode = remote.px4PoseOutputMode;
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

} // namespace

AppliedRuntimeConfig ApplyRemoteRuntimeConfig(UnifiedConfig &config,
                                              const RemoteRuntimeConfig &remote)
{
    AppliedRuntimeConfig applied{};
    applied.restartNeeded = RuntimeRestartNeeded(config.app, remote);
    ApplyCameraConfig(config.app.camera, remote);
    ApplyRuntimeConfig(config.app.runtime, remote);
    config.app.sensorMode = NormalizeSensorModeForSlamBackend(
        remote.sensorMode, config.app.runtime.slamBackend);
    config.app.settings =
        ResolveSettingsForSlamBackend(config.app.sensorMode,
                                      config.app.runtime.slamBackend,
                                      config.app.settings);
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
    tuning.avoidanceEnabled.store(remote.avoidanceEnabled,
                                  std::memory_order_relaxed);
    tuning.avoidanceHoldOnStaleCloud.store(
        remote.avoidanceHoldOnStaleCloud, std::memory_order_relaxed);
    tuning.avoidanceRadiusM.store(remote.avoidanceRadiusM,
                                  std::memory_order_relaxed);
    tuning.avoidanceLookaheadM.store(remote.avoidanceLookaheadM,
                                     std::memory_order_relaxed);
    tuning.avoidanceSpeedLookaheadS.store(
        remote.avoidanceSpeedLookaheadS, std::memory_order_relaxed);
    tuning.avoidanceNearFieldRadiusM.store(
        remote.avoidanceNearFieldRadiusM, std::memory_order_relaxed);
    tuning.avoidanceMaxPointCloudAgeMs.store(
        remote.avoidanceMaxPointCloudAgeMs, std::memory_order_relaxed);
    tuning.avoidanceMinCloudPoints.store(
        remote.avoidanceMinCloudPoints, std::memory_order_relaxed);
    tuning.avoidanceMinBlockingPoints.store(
        remote.avoidanceMinBlockingPoints, std::memory_order_relaxed);
    tuning.px4PoseOutputMode.store(static_cast<uint8_t>(
        remote.px4PoseOutputMode), std::memory_order_relaxed);
}

} // namespace SmartDrone::Core::Application
