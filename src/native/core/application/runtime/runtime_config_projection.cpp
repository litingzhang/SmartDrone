#include "core/application/runtime/runtime_config_projection.h"

namespace SmartDrone::Core::Application {

RemoteRuntimeConfig BuildRemoteConfig(const UnifiedConfig &currentConfig)
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

} // namespace SmartDrone::Core::Application
