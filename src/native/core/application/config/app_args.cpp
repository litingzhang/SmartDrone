#include "core/application/config/app_args.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <vector>

namespace {

std::string ResolveFirstExistingRuntimePath(const std::vector<std::string> &candidates, const char *argv0)
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

std::string NormalizeAccelerationText(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (value == "cuda" || value == "gpu" || value == "opencv_cuda" || value == "opencv-cuda") {
        return "cuda";
    }
    if (value == "vpi" || value == "vpi_remap" || value == "vpi-remap" || value == "vpi_cuda_remap" ||
        value == "vpi-cuda-remap") {
        return "vpi-remap";
    }
    return "cpu";
}

void ApplyOrbAccelerationEnvironment(const std::string &acceleration)
{
    const std::string normalized = NormalizeAccelerationText(acceleration);
#if defined(_WIN32)
    _putenv_s("SMART_DRONE_ORB_ACCEL", normalized == "cuda" ? "cuda" : "");
    _putenv_s("SMART_DRONE_ORB_VPI_REMAP", normalized == "vpi-remap" ? "1" : "");
    _putenv_s("SMART_DRONE_ORB_CUDA_PYRAMID", "");
#else
    if (normalized == "cuda") {
        setenv("SMART_DRONE_ORB_ACCEL", "cuda", 1);
    } else {
        unsetenv("SMART_DRONE_ORB_ACCEL");
    }
    if (normalized == "vpi-remap") {
        setenv("SMART_DRONE_ORB_VPI_REMAP", "1", 1);
    } else {
        unsetenv("SMART_DRONE_ORB_VPI_REMAP");
    }
    unsetenv("SMART_DRONE_ORB_CUDA_PYRAMID");
#endif
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

SensorMode ParseSensorModeText(const std::string &text)
{
    std::string normalized = text;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized == "mono-imu" || normalized == "imu-mono" || normalized == "mono_inertial") {
        return SensorMode::MonoImu;
    }
    if (normalized == "mono" || normalized == "monocular") {
        return SensorMode::Mono;
    }
    if (normalized == "stereo-imu" || normalized == "imu-stereo" || normalized == "stereo_inertial") {
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
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized == "droid" || normalized == "droid-light" || normalized == "droid_light" ||
        normalized == "droidlight") {
        return FeatureFrontend::DroidLight;
    }
    if (normalized == "superpoint-lightglue" || normalized == "superpoint_lightglue" ||
        normalized == "superpoint+lightglue" || normalized == "sp-lightglue" ||
        normalized == "sp_lightglue" || normalized == "sp-lg" || normalized == "splg") {
        return FeatureFrontend::SuperPointLightGlue;
    }
    if (normalized == "lk-gftt-per-frame" || normalized == "lk_gftt_per_frame" ||
        normalized == "lk-gftt-every-frame" || normalized == "lk_gftt_every_frame" ||
        normalized == "per-frame-gftt" || normalized == "per_frame_gftt") {
        return FeatureFrontend::LkGfttPerFrame;
    }
    if (normalized == "lk" || normalized == "klt" || normalized == "stereo-lk" || normalized == "stereo_lk" ||
        normalized == "optical-flow" || normalized == "optical_flow") {
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
    case FeatureFrontend::DroidLight:
        return "droid_light";
    case FeatureFrontend::SuperPointLightGlue:
        return "superpoint_lightglue";
    case FeatureFrontend::Orb:
    default:
        return "orb";
    }
}

smartdrone::core::domain::SlamOperationMode ParseSlamOperationModeText(const std::string &text)
{
    using smartdrone::core::domain::SlamOperationMode;

    std::string normalized = text;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (normalized == "localization" || normalized == "localisation") {
        return SlamOperationMode::Localization;
    }
    if (normalized == "relocalization" || normalized == "relocalisation") {
        return SlamOperationMode::Relocalization;
    }
    if (normalized == "tracking-only" || normalized == "tracking_only" || normalized == "trackingonly") {
        return SlamOperationMode::TrackingOnly;
    }
    if (normalized == "auto" || normalized == "auto-switch" || normalized == "auto_switch") {
        return SlamOperationMode::Auto;
    }
    return SlamOperationMode::Mapping;
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
        const fs::path exePath = fs::absolute(argv0 != nullptr ? fs::path(argv0) : fs::path{});
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

std::string ResolveSettingsForSensorMode(SensorMode mode, const std::string &currentSettingsPath)
{
    const fs::path targetName = fs::path(DefaultSettingsForSensorMode(mode)).filename();
    if (!currentSettingsPath.empty()) {
        const fs::path currentPath(currentSettingsPath);
        if (currentPath.is_absolute()) {
            return (currentPath.parent_path() / targetName).lexically_normal().string();
        }
    }
    return ResolveRuntimePath(DefaultSettingsForSensorMode(mode), nullptr);
}

ArgReader::ArgReader(int argc, char **argv) : m_argc(argc), m_argv(argv) {}

std::string ArgReader::GetString(const char *name, const char *defaultValue) const
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
            return std::stoi(m_argv[i + 1]);
        }
    }
    return defaultValue;
}

int64_t ArgReader::GetInt64(const char *name, int64_t defaultValue) const
{
    for (int i = 1; i + 1 < m_argc; ++i) {
        if (std::string(m_argv[i]) == name) {
            return std::stoll(m_argv[i + 1]);
        }
    }
    return defaultValue;
}

float ArgReader::GetFloat(const char *name, float defaultValue) const
{
    for (int i = 1; i + 1 < m_argc; ++i) {
        if (std::string(m_argv[i]) == name) {
            return std::stof(m_argv[i + 1]);
        }
    }
    return defaultValue;
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

uint8_t ArgReader::GetUint8HexOrDec(const char *name, uint8_t defaultValue, const char *defaultText) const
{
    return ParseUint8HexOrDec(GetString(name, defaultText), defaultValue);
}

uint8_t ArgReader::ParseUint8HexOrDec(const std::string &text, uint8_t defaultValue)
{
    try {
        int base = 10;
        if (text.size() > 2 && text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
            base = 16;
        }
        const int value = std::stoi(text, nullptr, base);
        if (value < 0 || value > 255) {
            return defaultValue;
        }
        return static_cast<uint8_t>(value);
    } catch (...) {
        return defaultValue;
    }
}

AppConfig ParseAppConfig(int argc, char **argv)
{
    ArgReader argReader(argc, argv);
    AppConfig config;

    config.vocab = ResolveRuntimePath(argReader.GetString("--vocab", "ORBvoc.txt"), argc > 0 ? argv[0] : nullptr);
    config.sensorMode = ParseSensorModeText(argReader.GetString("--sensor-mode", "stereo"));
    const char *defaultSettings = DefaultSettingsForSensorMode(config.sensorMode);
    config.settings =
        ResolveRuntimePath(argReader.GetString("--settings", defaultSettings), argc > 0 ? argv[0] : nullptr);

    config.camera.width = argReader.GetInt("--w", 640);
    config.camera.height = argReader.GetInt("--h", 400);
    config.camera.fps = argReader.GetInt("--fps", 60);
    config.camera.leftCamIndex = argReader.GetInt("--left-cam-index", 0);
    config.camera.rightCamIndex = argReader.GetInt("--right-cam-index", 1);
    config.camera.uvcDeviceIndex = argReader.GetInt("--uvc-device-index", config.camera.leftCamIndex);
    config.camera.uvcEyeWidth = argReader.GetInt("--uvc-eye-width", config.camera.width);
    config.camera.uvcEyeHeight = argReader.GetInt("--uvc-eye-height", config.camera.height);
    config.camera.uvcPackedStereo = true;
    if (argReader.HasFlag("--no-uvc-packed-stereo")) {
        config.camera.uvcPackedStereo = false;
    }
    if (argReader.HasFlag("--uvc-packed-stereo")) {
        config.camera.uvcPackedStereo = true;
    }
    config.camera.uvcSwapEyes = argReader.HasFlag("--uvc-swap-eyes");
    if (argReader.HasFlag("--swap-cams")) {
        std::swap(config.camera.leftCamIndex, config.camera.rightCamIndex);
    }
    config.camera.aeDisable = false;
    if (argReader.HasFlag("--no-ae")) {
        config.camera.aeDisable = true;
    }
    if (argReader.HasFlag("--ae")) {
        config.camera.aeDisable = false;
    }
    config.camera.exposureUs = argReader.GetInt("--exp-us", 5000);
    config.camera.gain = argReader.GetFloat("--gain", 2.0f);
    config.camera.requestY8 = !argReader.HasFlag("--no-y8");
    config.camera.r16Norm = argReader.HasFlag("--r16-norm");
    config.camera.pairMs = argReader.GetInt("--pair-ms", 2);
    config.camera.keepMs = argReader.GetInt("--keep-ms", 120);
    config.camera.pairQueue = argReader.GetInt("--pair-queue", 3);

    config.udp.enable = argReader.HasFlag("--udp");
    config.udp.ip = argReader.GetString("--udp-ip", "10.42.0.109");
    config.udp.port = argReader.GetInt("--udp-port", 5000);
    config.udp.cmdPort = argReader.GetInt("--cmd-port", 14550);
    config.udp.jpegQ = argReader.GetInt("--udp-jpeg-q", 80);
    config.udp.payload = argReader.GetInt("--udp-payload", 1200);
    config.udp.queue = argReader.GetInt("--udp-queue", 4);

    config.imu.spiDev = argReader.GetString("--spi", "/dev/spidev0.0");
    config.imu.spiSpeed = static_cast<uint32_t>(argReader.GetInt("--speed", 8000000));
    config.imu.spiMode = static_cast<uint8_t>(argReader.GetInt("--mode", 0));
    config.imu.spiBits = static_cast<uint8_t>(argReader.GetInt("--bits", 8));
    config.imu.gpiochip = argReader.GetString("--gpiochip", "/dev/gpiochip0");
    config.imu.drdyLine = static_cast<unsigned>(argReader.GetInt("--drdy", 24));
    config.imu.imuHz = argReader.GetInt("--imu-hz", 500);
    config.imu.accelFsG = argReader.GetInt("--accel-fs", 16);
    config.imu.gyroFsDps = argReader.GetInt("--gyro-fs", 2000);
    config.imu.imuStartReg = argReader.GetUint8HexOrDec("--imu-start-reg", 0x1F, "0x1F");
    config.imu.rtImu = argReader.HasFlag("--rt-imu");
    config.imu.rtPrio = argReader.GetInt("--rt-prio", 60);

    config.runtime.offRejectNs = argReader.GetInt64("--off-reject-ns", 10'000'000);
    config.runtime.allowEmptyImu = argReader.HasFlag("--allow-empty-imu");
    config.runtime.slamInputFps = argReader.GetInt("--slam-fps", 30);
    config.runtime.slamOperationMode = ParseSlamOperationModeText(argReader.GetString("--slam-mode", "mapping"));
    config.runtime.featureFrontend = ParseFeatureFrontendText(argReader.GetString("--feature-frontend", "orb"));
    {
        const char *home = std::getenv("HOME");
        const std::string explicitRepo = argReader.GetString("--xfeat-repo", "");
        if (!explicitRepo.empty()) {
            config.runtime.xfeatRepo = ResolveRuntimePath(explicitRepo, argc > 0 ? argv[0] : nullptr);
        } else {
            std::vector<std::string> repoCandidates;
            repoCandidates.emplace_back("LightGlue");
            repoCandidates.emplace_back("lightglue");
            repoCandidates.emplace_back("third_party/LightGlue");
            repoCandidates.emplace_back("third_party/lightglue");
            if (home != nullptr) {
                repoCandidates.push_back((fs::path(home) / "LightGlue").string());
                repoCandidates.push_back((fs::path(home) / "lightglue").string());
                repoCandidates.push_back((fs::path(home) / "third_party" / "LightGlue").string());
                repoCandidates.push_back((fs::path(home) / "third_party" / "lightglue").string());
            }
            config.runtime.xfeatRepo = ResolveFirstExistingRuntimePath(repoCandidates, argc > 0 ? argv[0] : nullptr);
        }
    }
    config.runtime.xfeatDevice = argReader.GetString("--xfeat-device", "auto");
    config.runtime.xfeatTopK = argReader.GetInt("--xfeat-top-k", 1024);
    config.runtime.xfeatMaxPoints = argReader.GetInt("--xfeat-max-points", 768);
    config.runtime.xfeatInputMaxWidth = argReader.GetInt("--xfeat-input-max-width", 640);
    config.runtime.xfeatInputMaxHeight = argReader.GetInt(
        "--xfeat-input-max-height",
        config.runtime.featureFrontend == FeatureFrontend::SuperPointLightGlue ? 480 : 400);
    config.runtime.lkXFeatSeeding = false;
    config.runtime.lkLoopClosure = argReader.HasFlag("--lk-loop-closure");
    config.runtime.lkLoopScale = argReader.GetFloat("--lk-loop-scale", 1.20f);
    config.runtime.lkLoopRelaxation = argReader.GetFloat("--lk-loop-relax", 1.40f);
    config.runtime.lkPerFrameAcceleration = argReader.GetString("--lk-per-frame-accel", "cpu");
    config.runtime.orbAcceleration = NormalizeAccelerationText(argReader.GetString("--orb-accel", "cpu"));
    ApplyOrbAccelerationEnvironment(config.runtime.orbAcceleration);
    config.runtime.debugRightOnlyFeatures = argReader.HasFlag("--debug-right-only-features");
    config.runtime.slamLowLightEnhance = argReader.HasFlag("--slam-lowlight-enhance");
    config.runtime.jsonDiagnostics = argReader.HasFlag("--json-diagnostics");

    return config;
}
