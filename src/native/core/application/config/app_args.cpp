#include "core/application/config/app_args.h"

#include <algorithm>
#include <cctype>
#include <vector>

#include "common/environment.h"
#include "common/numeric_parse.h"
#include "core/application/config/orb_acceleration_config.h"
#include "core/application/config/slam_backend_availability.h"
#include "core/application/runtime/obstacle_avoidance_config.h"

namespace {

namespace AppCore = SmartDrone::Core::Application;

std::string
ResolveFirstExistingRuntimePath(const std::vector<std::string> &candidates,
                                const char *argv0)
{
    std::string firstResolved;
    for (const std::string &candidate : candidates) {
        if (candidate.empty()) {
            continue;
        }
        const std::string resolved = ResolveRuntimePath(candidate, argv0);
        if (firstResolved.empty()) {
            firstResolved = resolved;
        }
        std::error_code ec;
        if (fs::exists(resolved, ec)) {
            return resolved;
        }
    }
    return firstResolved;
}

int GetIntWithLegacyFallback(const ArgReader &args, const char *name,
                             const char *legacyName, int defaultValue)
{
    if (args.HasOption(name)) {
        return args.GetInt(name, defaultValue);
    }
    return args.GetInt(legacyName, defaultValue);
}

std::string GetStringWithLegacyFallback(const ArgReader &args, const char *name,
                                        const char *legacyName,
                                        const char *defaultValue)
{
    if (args.HasOption(name)) {
        return args.GetString(name, defaultValue);
    }
    return args.GetString(legacyName, defaultValue);
}

fs::path ResolveDefaultSettingsFromCurrent(const fs::path &defaultSettings,
                                           const fs::path &currentPath)
{
    const fs::path parent = currentPath.parent_path();
    if (parent.filename() == "config") {
        return parent.parent_path() / defaultSettings;
    }
    if (parent.filename() == "openvins" &&
        parent.parent_path().filename() == "config") {
        return parent.parent_path().parent_path() / defaultSettings;
    }
    return parent / defaultSettings.filename();
}

} // namespace

const char *DefaultSettingsForSensorMode(SensorMode mode)
{
    switch (mode) {
    case SensorMode::StereoImu:
        return "config/stereo_inertial.yaml";
    case SensorMode::Mono:
        return "config/mono_right.yaml";
    case SensorMode::MonoImu:
        return "config/mono_inertial_right.yaml";
    case SensorMode::Stereo:
    default:
        return "config/stereo.yaml";
    }
}

SensorMode NormalizeSensorModeForSlamBackend(SensorMode mode,
                                             SlamBackend backend)
{
    if (backend != SlamBackend::OpenVins) {
        return mode;
    }
    return SensorMode::StereoImu;
}

const char *DefaultSettingsForSlamBackend(SensorMode mode, SlamBackend backend)
{
    if (backend == SlamBackend::OpenVins) {
        return "config/openvins/estimator_config.yaml";
    }
    return DefaultSettingsForSensorMode(mode);
}

SensorMode ParseSensorModeText(const std::string &text)
{
    std::string normalized = text;
    std::transform(
        normalized.begin(), normalized.end(), normalized.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized == "mono-imu" || normalized == "imu-mono" ||
        normalized == "mono_inertial") {
        return SensorMode::MonoImu;
    }
    if (normalized == "mono" || normalized == "monocular") {
        return SensorMode::Mono;
    }
    if (normalized == "stereo-imu" || normalized == "imu-stereo" ||
        normalized == "stereo_inertial") {
        return SensorMode::StereoImu;
    }
    return SensorMode::Stereo;
}

const char *ToSensorModeText(SensorMode mode)
{
    switch (mode) {
    case SensorMode::StereoImu:
        return "stereo-imu";
    case SensorMode::Mono:
        return "mono";
    case SensorMode::MonoImu:
        return "mono-imu";
    case SensorMode::Stereo:
    default:
        return "stereo";
    }
}

FeatureFrontend ParseFeatureFrontendText(const std::string &text)
{
    std::string normalized = text;
    std::transform(
        normalized.begin(), normalized.end(), normalized.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized == "superpoint-lightglue" ||
        normalized == "superpoint_lightglue" ||
        normalized == "superpoint+lightglue" || normalized == "sp-lightglue" ||
        normalized == "sp_lightglue" || normalized == "sp-lg" ||
        normalized == "splg") {
        return FeatureFrontend::SuperPointLightGlue;
    }
    if (normalized == "xfeat-lightglue" || normalized == "xfeat_lightglue" ||
        normalized == "xfeat+lightglue" || normalized == "xfeat-lg" ||
        normalized == "xfeat_lg" || normalized == "xfeatlg") {
        return FeatureFrontend::XFeatLightGlue;
    }
    if (normalized == "lk-gftt-per-frame" || normalized == "lk_gftt_per_frame" ||
        normalized == "lk-gftt-every-frame" ||
        normalized == "lk_gftt_every_frame" || normalized == "per-frame-gftt" ||
        normalized == "per_frame_gftt" || normalized == "klt-tracking" ||
        normalized == "klt_tracking") {
        return FeatureFrontend::LkGfttPerFrame;
    }
    if (normalized == "lk" || normalized == "klt" || normalized == "stereo-lk" ||
        normalized == "stereo_lk" || normalized == "optical-flow" ||
        normalized == "optical_flow") {
        return FeatureFrontend::LK;
    }
    return FeatureFrontend::Orb;
}

const char *ToFeatureFrontendText(FeatureFrontend frontend)
{
    switch (frontend) {
    case FeatureFrontend::LkGfttPerFrame:
        return "lk_gftt_per_frame";
    case FeatureFrontend::LK:
        return "lk";
    case FeatureFrontend::SuperPointLightGlue:
        return "superpoint_lightglue";
    case FeatureFrontend::XFeatLightGlue:
        return "xfeat_lightglue";
    case FeatureFrontend::Orb:
    default:
        return "orb";
    }
}

bool IsVisualFeatureLightGlueFrontend(FeatureFrontend frontend)
{
    return frontend == FeatureFrontend::SuperPointLightGlue ||
           frontend == FeatureFrontend::XFeatLightGlue;
}

SlamBackend ParseSlamBackendText(const std::string &text)
{
    std::string normalized = text;
    std::transform(
        normalized.begin(), normalized.end(), normalized.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized == "klt" || normalized == "lk" || normalized == "stereo-klt" ||
        normalized == "stereo_klt" || normalized == "klt_pnp" ||
        normalized == "lk_gftt_per_frame") {
        return SlamBackend::Klt;
    }
    if (normalized == "dpvo" || normalized == "dpvo_tensorrt" ||
        normalized == "dpvo-tensorrt" || normalized == "dpvo_trt" ||
        normalized == "dpvo-trt") {
        return SlamBackend::DpvoTensorRt;
    }
    if (normalized == "openvins" || normalized == "open-vins" ||
        normalized == "open_vins" || normalized == "ov_msckf" ||
        normalized == "ov-msckf" || normalized == "ov") {
        return SlamBackend::OpenVins;
    }
    if (normalized == "orbslam3" || normalized == "orb-slam3" ||
        normalized == "orb_slam3" || normalized == "orb") {
        return SlamBackend::OrbSlam3;
    }
    return SlamBackend::Klt;
}

SlamBackend NormalizeSlamBackendForBuild(SlamBackend backend)
{
    if (backend == SlamBackend::OrbSlam3 && !OrbSlam3BackendAvailable()) {
        return SlamBackend::Klt;
    }
    if (backend == SlamBackend::OpenVins && !OpenVinsBackendAvailable()) {
        return SlamBackend::Klt;
    }
    return backend;
}

const char *ToSlamBackendText(SlamBackend backend)
{
    switch (backend) {
    case SlamBackend::Klt:
        return "klt";
    case SlamBackend::DpvoTensorRt:
        return "dpvo_tensorrt";
    case SlamBackend::OpenVins:
        return "openvins";
    case SlamBackend::OrbSlam3:
    default:
        return "orbslam3";
    }
}

SmartDrone::Core::Domain::SlamOperationMode
ParseSlamOperationModeText(const std::string &text)
{
    using SmartDrone::Core::Domain::SlamOperationMode;

    std::string normalized = text;
    std::transform(
        normalized.begin(), normalized.end(), normalized.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (normalized == "localization" || normalized == "localisation") {
        return SlamOperationMode::Localization;
    }
    if (normalized == "relocalization" || normalized == "relocalisation") {
        return SlamOperationMode::Relocalization;
    }
    if (normalized == "tracking-only" || normalized == "tracking_only" ||
        normalized == "trackingonly") {
        return SlamOperationMode::TrackingOnly;
    }
    if (normalized == "auto" || normalized == "auto-switch" ||
        normalized == "auto_switch") {
        return SlamOperationMode::Auto;
    }
    return SlamOperationMode::Mapping;
}

SmartDrone::Core::Domain::Px4PoseOutputMode
ParsePx4PoseOutputModeText(const std::string &text)
{
    using SmartDrone::Core::Domain::Px4PoseOutputMode;

    std::string normalized = text;
    std::transform(
        normalized.begin(), normalized.end(), normalized.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (normalized == "none" || normalized == "off" ||
        normalized == "disabled") {
        return Px4PoseOutputMode::None;
    }
    if (normalized == "position" || normalized == "pose") {
        return Px4PoseOutputMode::Position;
    }
    if (normalized == "position_velocity" ||
        normalized == "position-velocity" || normalized == "pose_velocity" ||
        normalized == "pose-velocity" || normalized == "full" ||
        normalized == "default") {
        return Px4PoseOutputMode::PositionVelocity;
    }
    return Px4PoseOutputMode::PositionVelocity;
}

const char *ToPx4PoseOutputModeText(
    SmartDrone::Core::Domain::Px4PoseOutputMode mode)
{
    return SmartDrone::Core::Domain::ToString(mode);
}

std::string ResolveRuntimePath(const std::string &path, const char *argv0)
{
    if (path.empty()) {
        return path;
    }

    const fs::path rawPath(path);
    if (rawPath.is_absolute()) {
        return rawPath.lexically_normal().string();
    }

    std::vector<fs::path> candidates;
    candidates.push_back(fs::current_path() / rawPath);

    try {
        const fs::path exePath =
            fs::absolute(argv0 != nullptr ? fs::path(argv0) : fs::path{});
        const fs::path exeDir = exePath.parent_path();
        if (!exeDir.empty()) {
            candidates.push_back(exeDir / rawPath);
            const fs::path parent = exeDir.parent_path();
            if (!parent.empty()) {
                candidates.push_back(parent / rawPath);
                const fs::path grandParent = parent.parent_path();
                if (!grandParent.empty()) {
                    candidates.push_back(grandParent / rawPath);
                }
            }
        }
    } catch (...) {
    }

    for (const auto &candidate : candidates) {
        std::error_code ec;
        if (fs::exists(candidate, ec)) {
            return candidate.lexically_normal().string();
        }
    }

    if (!candidates.empty()) {
        return candidates.front().lexically_normal().string();
    }
    return rawPath.lexically_normal().string();
}

std::string
ResolveSettingsForSensorMode(SensorMode mode,
                             const std::string &currentSettingsPath)
{
    return ResolveSettingsForSlamBackend(mode, SlamBackend::Klt,
                                         currentSettingsPath);
}

std::string
ResolveSettingsForSlamBackend(SensorMode mode, SlamBackend backend,
                              const std::string &currentSettingsPath)
{
    const fs::path defaultSettings(DefaultSettingsForSlamBackend(mode, backend));
    if (!currentSettingsPath.empty()) {
        const fs::path currentPath(currentSettingsPath);
        if (currentPath.is_absolute()) {
            return ResolveDefaultSettingsFromCurrent(defaultSettings, currentPath)
                .lexically_normal()
                .string();
        }
    }
    return ResolveRuntimePath(defaultSettings.string(), nullptr);
}

ArgReader::ArgReader(int argc, char **argv)
    : m_argc(argc), m_argv(argv)
{
}

std::string ArgReader::GetString(const char *name,
                                 const char *defaultValue) const
{
    for (int i = 1; i + 1 < m_argc; ++i) {
        if (std::string(m_argv[i]) == name) {
            return m_argv[i + 1];
        }
    }
    return defaultValue;
}

int ArgReader::GetInt(const char *name, int defaultValue) const
{
    for (int i = 1; i + 1 < m_argc; ++i) {
        if (std::string(m_argv[i]) == name) {
            int parsed = defaultValue;
            if (!SmartDrone::Common::TryParseIntPrefix(
                    m_argv[i + 1], 10, parsed)) {
                throw std::invalid_argument(m_argv[i + 1]);
            }
            return parsed;
        }
    }
    return defaultValue;
}

int64_t ArgReader::GetInt64(const char *name, int64_t defaultValue) const
{
    for (int i = 1; i + 1 < m_argc; ++i) {
        if (std::string(m_argv[i]) == name) {
            int64_t parsed = defaultValue;
            if (!SmartDrone::Common::TryParseInt64Prefix(
                    m_argv[i + 1], 10, parsed)) {
                throw std::invalid_argument(m_argv[i + 1]);
            }
            return parsed;
        }
    }
    return defaultValue;
}

float ArgReader::GetFloat(const char *name, float defaultValue) const
{
    for (int i = 1; i + 1 < m_argc; ++i) {
        if (std::string(m_argv[i]) == name) {
            float parsed = defaultValue;
            if (!SmartDrone::Common::TryParseFloatPrefix(m_argv[i + 1],
                                                         parsed)) {
                throw std::invalid_argument(m_argv[i + 1]);
            }
            return parsed;
        }
    }
    return defaultValue;
}

bool ArgReader::HasOption(const char *name) const
{
    for (int i = 1; i + 1 < m_argc; ++i) {
        if (std::string(m_argv[i]) == name) {
            return true;
        }
    }
    return false;
}

bool ArgReader::HasFlag(const char *name) const
{
    for (int i = 1; i < m_argc; ++i) {
        if (std::string(m_argv[i]) == name) {
            return true;
        }
    }
    return false;
}

uint8_t ArgReader::GetUint8HexOrDec(const char *name, uint8_t defaultValue,
                                    const char *defaultText) const
{
    return ParseUint8HexOrDec(GetString(name, defaultText), defaultValue);
}

uint8_t ArgReader::ParseUint8HexOrDec(const std::string &text,
                                      uint8_t defaultValue)
{
    int base = 10;
    if (text.size() > 2 && text[0] == '0' &&
        (text[1] == 'x' || text[1] == 'X')) {
        base = 16;
    }
    int value = defaultValue;
    if (!SmartDrone::Common::TryParseIntFull(text.c_str(), base, value) ||
        value < 0 || value > 255) {
        return defaultValue;
    }
    return static_cast<uint8_t>(value);
}

namespace {

const char *GetArgv0(int argc, char **argv)
{
    if (argc > 0) {
        return argv[0];
    }
    return nullptr;
}

void ParseSensorSettings(const ArgReader &argReader, const char *argv0,
                         AppConfig &config)
{
    config.sensorMode =
        ParseSensorModeText(argReader.GetString("--sensor-mode", "stereo"));
}

void ParseCameraPackingFlags(const ArgReader &argReader,
                             CameraConfig &cameraConfig)
{
    cameraConfig.uvcPackedStereo = true;
    if (argReader.HasFlag("--no-uvc-packed-stereo")) {
        cameraConfig.uvcPackedStereo = false;
    }
    if (argReader.HasFlag("--uvc-packed-stereo")) {
        cameraConfig.uvcPackedStereo = true;
    }
}

void ParseCameraExposureFlags(const ArgReader &argReader,
                              CameraConfig &cameraConfig)
{
    cameraConfig.aeDisable = false;
    if (argReader.HasFlag("--no-ae")) {
        cameraConfig.aeDisable = true;
    }
    if (argReader.HasFlag("--ae")) {
        cameraConfig.aeDisable = false;
    }
}

CameraConfig ParseCameraConfig(const ArgReader &argReader)
{
    CameraConfig cameraConfig;
    cameraConfig.width = argReader.GetInt("--w", 640);
    cameraConfig.height = argReader.GetInt("--h", 400);
    cameraConfig.fps = argReader.GetInt("--fps", 60);
    cameraConfig.leftCamIndex = argReader.GetInt("--left-cam-index", 0);
    cameraConfig.rightCamIndex = argReader.GetInt("--right-cam-index", 1);
    cameraConfig.uvcDeviceIndex =
        argReader.GetInt("--uvc-device-index", cameraConfig.leftCamIndex);
    cameraConfig.uvcEyeWidth =
        argReader.GetInt("--uvc-eye-width", cameraConfig.width);
    cameraConfig.uvcEyeHeight =
        argReader.GetInt("--uvc-eye-height", cameraConfig.height);
    ParseCameraPackingFlags(argReader, cameraConfig);
    cameraConfig.uvcSwapEyes = argReader.HasFlag("--uvc-swap-eyes");
    if (argReader.HasFlag("--swap-cams")) {
        std::swap(cameraConfig.leftCamIndex, cameraConfig.rightCamIndex);
    }
    ParseCameraExposureFlags(argReader, cameraConfig);
    cameraConfig.exposureUs = argReader.GetInt("--exp-us", 5000);
    cameraConfig.gain = argReader.GetFloat("--gain", 2.0f);
    cameraConfig.requestY8 = !argReader.HasFlag("--no-y8");
    cameraConfig.r16Norm = argReader.HasFlag("--r16-norm");
    cameraConfig.pairMs = argReader.GetInt("--pair-ms", 2);
    cameraConfig.keepMs = argReader.GetInt("--keep-ms", 120);
    cameraConfig.pairQueue = argReader.GetInt("--pair-queue", 3);
    return cameraConfig;
}

UdpConfig ParseUdpConfig(const ArgReader &argReader)
{
    UdpConfig udpConfig;
    udpConfig.enable = argReader.HasFlag("--udp");
    udpConfig.ip = argReader.GetString("--udp-ip", "10.42.0.109");
    udpConfig.port = argReader.GetInt("--udp-port", 5000);
    udpConfig.cmdPort = argReader.GetInt("--cmd-port", 14550);
    udpConfig.jpegQ = argReader.GetInt("--udp-jpeg-q", 80);
    udpConfig.payload = argReader.GetInt("--udp-payload", 1200);
    udpConfig.queue = argReader.GetInt("--udp-queue", 4);
    return udpConfig;
}

ImuRuntimeConfig ParseImuRuntimeConfig(const ArgReader &argReader)
{
    ImuRuntimeConfig imuConfig;
    imuConfig.spiDev = argReader.GetString("--spi", "/dev/spidev0.0");
    imuConfig.spiSpeed =
        static_cast<uint32_t>(argReader.GetInt("--speed", 8000000));
    imuConfig.spiMode = static_cast<uint8_t>(argReader.GetInt("--mode", 0));
    imuConfig.spiBits = static_cast<uint8_t>(argReader.GetInt("--bits", 8));
    imuConfig.gpiochip = argReader.GetString("--gpiochip", "/dev/gpiochip0");
    imuConfig.drdyLine = static_cast<unsigned>(argReader.GetInt("--drdy", 24));
    imuConfig.imuHz = argReader.GetInt("--imu-hz", 500);
    imuConfig.accelFsG = argReader.GetInt("--accel-fs", 16);
    imuConfig.gyroFsDps = argReader.GetInt("--gyro-fs", 2000);
    imuConfig.imuStartReg =
        argReader.GetUint8HexOrDec("--imu-start-reg", 0x1F, "0x1F");
    imuConfig.rtImu = argReader.HasFlag("--rt-imu");
    imuConfig.rtPrio = argReader.GetInt("--rt-prio", 60);
    return imuConfig;
}

void ParseCoreRuntimeConfig(const ArgReader &argReader, const char *argv0,
                            AppConfig &config)
{
    config.runtime.offRejectNs =
        argReader.GetInt64("--off-reject-ns", 10'000'000);
    config.runtime.allowEmptyImu = argReader.HasFlag("--allow-empty-imu");
    config.runtime.slamInputFps = argReader.GetInt("--slam-fps", 30);
    config.runtime.slamOperationMode =
        ParseSlamOperationModeText(argReader.GetString("--slam-mode", "mapping"));
    config.runtime.slamBackend = NormalizeSlamBackendForBuild(
        ParseSlamBackendText(argReader.GetString("--slam-backend", "klt")));
    config.runtime.px4PoseOutputMode = ParsePx4PoseOutputModeText(
        argReader.GetString("--px4-pose-output-mode", "position_velocity"));
    const std::string vocabArg = argReader.GetString(
        "--vocab",
        config.runtime.slamBackend == SlamBackend::OrbSlam3 ? "ORBvoc.txt" : "");
    config.vocab = ResolveRuntimePath(vocabArg, argv0);
}

void ResolveSlamBackendSettings(const ArgReader &argReader, const char *argv0,
                                AppConfig &config)
{
    config.sensorMode = NormalizeSensorModeForSlamBackend(
        config.sensorMode, config.runtime.slamBackend);
    const char *defaultSettings = DefaultSettingsForSlamBackend(
        config.sensorMode, config.runtime.slamBackend);
    config.settings = ResolveRuntimePath(
        argReader.GetString("--settings", defaultSettings), argv0);
}

void ParseDpvoRuntimeConfig(const ArgReader &argReader, const char *argv0,
                            RuntimeConfig &runtimeConfig)
{
    runtimeConfig.dpvoRepo =
        ResolveRuntimePath(argReader.GetString("--dpvo-repo", ""), argv0);
    runtimeConfig.dpvoPatchEngine =
        ResolveRuntimePath(argReader.GetString("--dpvo-patch-engine", ""), argv0);
    runtimeConfig.dpvoUpdateEngine = ResolveRuntimePath(
        argReader.GetString("--dpvo-update-engine", ""), argv0);
    runtimeConfig.dpvoInputWidth = argReader.GetInt("--dpvo-input-width", 640);
    runtimeConfig.dpvoInputHeight = argReader.GetInt("--dpvo-input-height", 400);
    runtimeConfig.dpvoPatchesPerFrame =
        argReader.GetInt("--dpvo-patches-per-frame", 48);
    runtimeConfig.dpvoOptimizationWindow =
        argReader.GetInt("--dpvo-optimization-window", 7);
}

std::vector<std::string> BuildVisualFeatureRepoCandidates(
    const std::string &home)
{
    std::vector<std::string> repoCandidates;
    repoCandidates.emplace_back("LightGlue");
    repoCandidates.emplace_back("lightglue");
    repoCandidates.emplace_back("third_party/LightGlue");
    repoCandidates.emplace_back("third_party/lightglue");
    if (home.empty()) {
        return repoCandidates;
    }
    repoCandidates.push_back((fs::path(home) / "LightGlue").string());
    repoCandidates.push_back((fs::path(home) / "lightglue").string());
    repoCandidates.push_back(
        (fs::path(home) / "third_party" / "LightGlue").string());
    repoCandidates.push_back(
        (fs::path(home) / "third_party" / "lightglue").string());
    return repoCandidates;
}

std::string ResolveVisualFeatureRepo(const ArgReader &argReader,
                                     const char *argv0)
{
    const std::string explicitRepo = GetStringWithLegacyFallback(
        argReader, "--visual-feature-repo", "--superpoint-repo", "");
    if (!explicitRepo.empty()) {
        return ResolveRuntimePath(explicitRepo, argv0);
    }
    const std::string home = SmartDrone::Common::EnvStringValue("HOME", "");
    return ResolveFirstExistingRuntimePath(
        BuildVisualFeatureRepoCandidates(home), argv0);
}

void ParseVisualFeatureRuntimeConfig(const ArgReader &argReader,
                                     const char *argv0,
                                     RuntimeConfig &runtimeConfig)
{
    runtimeConfig.featureFrontend = ParseFeatureFrontendText(
        argReader.GetString("--feature-frontend", "lk_gftt_per_frame"));
    runtimeConfig.visualFeatureRepo = ResolveVisualFeatureRepo(argReader, argv0);
    runtimeConfig.visualFeatureDevice = GetStringWithLegacyFallback(
        argReader, "--visual-feature-device", "--superpoint-device", "auto");
    runtimeConfig.visualFeatureTopK = GetIntWithLegacyFallback(
        argReader, "--visual-feature-top-k", "--superpoint-top-k", 1024);
    runtimeConfig.visualFeatureMaxPoints = GetIntWithLegacyFallback(
        argReader, "--visual-feature-max-points", "--superpoint-max-points", 512);
    runtimeConfig.visualFeatureInputMaxWidth =
        GetIntWithLegacyFallback(argReader, "--visual-feature-input-max-width",
                                 "--superpoint-input-max-width", 640);
    runtimeConfig.visualFeatureInputMaxHeight = GetIntWithLegacyFallback(
        argReader, "--visual-feature-input-max-height",
        "--superpoint-input-max-height",
        runtimeConfig.featureFrontend == FeatureFrontend::SuperPointLightGlue
            ? 409
            : 400);
}

void ParseSlamFeatureTuningConfig(const ArgReader &argReader,
                                  RuntimeConfig &runtimeConfig)
{
    runtimeConfig.lkLoopClosure = argReader.HasFlag("--lk-loop-closure");
    runtimeConfig.lkLoopScale = argReader.GetFloat("--lk-loop-scale", 1.20f);
    runtimeConfig.lkLoopRelaxation = argReader.GetFloat("--lk-loop-relax", 1.40f);
    runtimeConfig.lkPerFrameAcceleration =
        argReader.GetString("--lk-per-frame-accel", "vpi-cuda");
    runtimeConfig.orbAcceleration =
        AppCore::NormalizeOrbAccelerationOrCpu(
            argReader.GetString("--orb-accel", "cpu"));
    if (runtimeConfig.slamBackend == SlamBackend::OrbSlam3) {
        AppCore::ApplyOrbAccelerationEnvironment(
            runtimeConfig.orbAcceleration);
    }
}

void ParseDiagnosticsConfig(const ArgReader &argReader,
                            RuntimeConfig &runtimeConfig)
{
    runtimeConfig.debugRightOnlyFeatures =
        argReader.HasFlag("--debug-right-only-features");
    runtimeConfig.slamLowLightEnhance =
        argReader.HasFlag("--slam-lowlight-enhance");
    runtimeConfig.jsonDiagnostics = argReader.HasFlag("--json-diagnostics");
}

void ParseAvoidanceRuntimeConfig(RuntimeConfig &runtimeConfig)
{
    const auto avoidanceConfig =
        AppCore::ReadObstacleAvoidanceConfig();
    runtimeConfig.avoidanceEnabled = avoidanceConfig.enabled;
    runtimeConfig.avoidanceHoldOnStaleCloud =
        avoidanceConfig.holdOnStaleCloud;
    runtimeConfig.avoidanceRadiusM = avoidanceConfig.radiusM;
    runtimeConfig.avoidanceLookaheadM = avoidanceConfig.lookaheadM;
    runtimeConfig.avoidanceSpeedLookaheadS =
        avoidanceConfig.speedLookaheadS;
    runtimeConfig.avoidanceNearFieldRadiusM =
        avoidanceConfig.nearFieldRadiusM;
    runtimeConfig.avoidanceMaxPointCloudAgeMs =
        avoidanceConfig.maxPointCloudAgeMs;
    runtimeConfig.avoidanceMinCloudPoints =
        avoidanceConfig.minCloudPoints;
    runtimeConfig.avoidanceMinBlockingPoints =
        avoidanceConfig.minBlockingPoints;
}

} // namespace

AppConfig ParseAppConfig(int argc, char **argv)
{
    const char *argv0 = GetArgv0(argc, argv);
    ArgReader argReader(argc, argv);
    AppConfig config;

    ParseSensorSettings(argReader, argv0, config);
    config.camera = ParseCameraConfig(argReader);
    config.udp = ParseUdpConfig(argReader);
    config.imu = ParseImuRuntimeConfig(argReader);
    ParseCoreRuntimeConfig(argReader, argv0, config);
    ResolveSlamBackendSettings(argReader, argv0, config);
    ParseDpvoRuntimeConfig(argReader, argv0, config.runtime);
    ParseVisualFeatureRuntimeConfig(argReader, argv0, config.runtime);
    ParseSlamFeatureTuningConfig(argReader, config.runtime);
    ParseDiagnosticsConfig(argReader, config.runtime);
    ParseAvoidanceRuntimeConfig(config.runtime);

    return config;
}
