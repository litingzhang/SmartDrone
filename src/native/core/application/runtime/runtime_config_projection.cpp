#include "core/application/runtime/runtime_config_projection.h"

namespace SmartDrone::Core::Application {
namespace {

void ProjectCameraConfig(RemoteRuntimeConfig &remote,
                         const AppConfig &app)
{
    remote.exposureUs = app.camera.exposureUs;
    remote.gain = app.camera.gain;
    remote.autoExposureEnabled = !app.camera.aeDisable;
    remote.pairMs = app.camera.pairMs > 0 ? app.camera.pairMs : 1;
    remote.uvcDeviceIndex = app.camera.uvcDeviceIndex;
    remote.uvcEyeWidth = app.camera.uvcEyeWidth;
    remote.uvcEyeHeight = app.camera.uvcEyeHeight;
    remote.uvcPackedStereo = app.camera.uvcPackedStereo;
}

void ProjectRuntimeConfig(RemoteRuntimeConfig &remote,
                          const AppConfig &app)
{
    remote.slamInputFps = app.runtime.slamInputFps;
    remote.slamOperationMode = app.runtime.slamOperationMode;
    remote.slamBackend = app.runtime.slamBackend;
    remote.featureFrontend = app.runtime.featureFrontend;
    remote.sensorMode = app.sensorMode;
    remote.lkPerFrameAcceleration = app.runtime.lkPerFrameAcceleration;
    remote.orbAcceleration = app.runtime.orbAcceleration;
}

void ProjectTbcConfig(RemoteRuntimeConfig &remote,
                      const RuntimeConfig &runtime)
{
    remote.useCustomTbc = runtime.useCustomTbc;
    remote.tbcTx = runtime.tbcTx;
    remote.tbcTy = runtime.tbcTy;
    remote.tbcTz = runtime.tbcTz;
    remote.tbcRollDeg = runtime.tbcRollDeg;
    remote.tbcPitchDeg = runtime.tbcPitchDeg;
    remote.tbcYawDeg = runtime.tbcYawDeg;
}

void ProjectFeatureConfig(RemoteRuntimeConfig &remote,
                          const RuntimeConfig &runtime)
{
    remote.orbNFeatures = runtime.orbNFeatures;
    remote.orbScaleFactor = runtime.orbScaleFactor;
    remote.orbNLevels = runtime.orbNLevels;
    remote.orbIniThFAST = runtime.orbIniThFAST;
    remote.orbMinThFAST = runtime.orbMinThFAST;
    remote.visualFeatureTopK = runtime.visualFeatureTopK;
    remote.visualFeatureMaxPoints = runtime.visualFeatureMaxPoints;
    remote.visualFeatureInputMaxWidth = runtime.visualFeatureInputMaxWidth;
    remote.visualFeatureInputMaxHeight = runtime.visualFeatureInputMaxHeight;
}

void ProjectAvoidanceConfig(RemoteRuntimeConfig &remote,
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

void ProjectStreamConfig(RemoteRuntimeConfig &remote, const UdpConfig &udp)
{
    remote.udpIp = udp.ip;
    remote.udpEnabled = udp.enable;
    remote.sendImage = udp.sendImage;
    remote.sendFeature = udp.sendFeature;
    remote.sendMap = udp.sendMap;
}

} // namespace

RemoteRuntimeConfig BuildRemoteConfig(const UnifiedConfig &currentConfig)
{
    RemoteRuntimeConfig remote{};
    ProjectCameraConfig(remote, currentConfig.app);
    ProjectRuntimeConfig(remote, currentConfig.app);
    ProjectStreamConfig(remote, currentConfig.app.udp);
    ProjectTbcConfig(remote, currentConfig.app.runtime);
    ProjectFeatureConfig(remote, currentConfig.app.runtime);
    ProjectAvoidanceConfig(remote, currentConfig.app.runtime);
    return remote;
}

} // namespace SmartDrone::Core::Application
