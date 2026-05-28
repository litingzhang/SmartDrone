#include "core/application/runtime/runtime_config_validation.h"

#include <cmath>
#include <string>

#include "core/application/config/app_args.h"
#include "core/application/config/orb_acceleration_config.h"

namespace SmartDrone::Core::Application {
namespace {

bool SetError(std::string *err, const char *message)
{
    if (err) {
        *err = message;
    }
    return false;
}

bool ValidateOrbExtractorConfig(const RemoteRuntimeConfig &remote,
                                std::string *err)
{
    if (remote.orbNFeatures <= 0 || !(remote.orbScaleFactor > 0.0f) ||
        remote.orbNLevels <= 0 || remote.orbIniThFAST <= 0 ||
        remote.orbMinThFAST <= 0) {
        return SetError(err, "bad orb extractor config");
    }
    if (remote.orbMinThFAST > remote.orbIniThFAST) {
        return SetError(err, "orb minThFAST must be <= iniThFAST");
    }
    if (remote.orbNFeatures < 100 || remote.orbNFeatures > 5000 ||
        remote.orbScaleFactor < 1.01f || remote.orbScaleFactor > 3.0f ||
        remote.orbNLevels < 1 || remote.orbNLevels > 16 ||
        remote.orbIniThFAST > 100 || remote.orbMinThFAST > 100) {
        return SetError(err, "orb extractor config out of range");
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

} // namespace

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

} // namespace SmartDrone::Core::Application
