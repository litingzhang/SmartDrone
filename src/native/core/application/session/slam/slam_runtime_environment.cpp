#include "core/application/session/slam/slam_runtime_environment.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include "core/application/config/runtime_app_types.h"

namespace SmartDrone::core::application {
namespace {

namespace fs = std::filesystem;

std::string TrimCopy(const std::string &in)
{
    size_t begin = 0;
    while (begin < in.size() &&
           std::isspace(static_cast<unsigned char>(in[begin])) != 0) {
        ++begin;
    }

    size_t end = in.size();
    while (end > begin &&
           std::isspace(static_cast<unsigned char>(in[end - 1])) != 0) {
        --end;
    }
    return in.substr(begin, end - begin);
}

bool IsYamlKeyLine(const std::string &line, const std::string &key)
{
    const std::string trimmed = TrimCopy(line);
    if (trimmed.rfind(key, 0) != 0) {
        return false;
    }
    if (trimmed.size() <= key.size()) {
        return false;
    }
    return trimmed[key.size()] == ':';
}

void ReplaceOrInsertYamlScalar(std::string &text, const std::string &key,
                               const std::string &value)
{
    std::vector<std::string> lines;
    {
        std::istringstream input(text);
        std::string line;
        while (std::getline(input, line)) {
            lines.push_back(line);
        }
    }

    const std::string replacement = key + ": " + value;
    bool replaced = false;
    for (std::string &line : lines) {
        if (IsYamlKeyLine(line, key)) {
            line = replacement;
            replaced = true;
            break;
        }
    }

    if (!replaced) {
        size_t insertAt = lines.size();
        for (size_t i = 0; i < lines.size(); ++i) {
            if (TrimCopy(lines[i]) == "...") {
                insertAt = i;
                break;
            }
        }
        lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(insertAt),
                     replacement);
    }

    std::ostringstream output;
    for (const std::string &line : lines) {
        output << line << "\n";
    }
    text = output.str();
}

bool HasOrbExtractorRuntimeOverride(const RuntimeConfig &runtime)
{
    return runtime.orbNFeatures > 0 && runtime.orbScaleFactor > 0.0f &&
           runtime.orbNLevels > 0 && runtime.orbIniThFAST > 0 &&
           runtime.orbMinThFAST > 0;
}

bool ShouldWriteRuntimeOrbSettings(const AppConfig &app)
{
    return app.runtime.slamBackend == SlamBackend::OrbSlam3 &&
           HasOrbExtractorRuntimeOverride(app.runtime);
}

std::string ReadSettingsFile(const std::string &settingsPath, bool &ok)
{
    std::ifstream in(settingsPath, std::ios::in);
    if (!in.is_open()) {
        std::cerr << "[slam] warning: failed to open settings for ORB override: "
                  << settingsPath << "\n";
        ok = false;
        return {};
    }

    std::ostringstream buffer;
    buffer << in.rdbuf();
    ok = true;
    return buffer.str();
}

std::string FormatRuntimeFloat(float value)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6) << value;
    return ss.str();
}

void ApplyOrbExtractorRuntimeConfig(std::string &content,
                                    const RuntimeConfig &runtime)
{
    ReplaceOrInsertYamlScalar(content, "ORBextractor.nFeatures",
                              std::to_string(runtime.orbNFeatures));
    ReplaceOrInsertYamlScalar(content, "ORBextractor.scaleFactor",
                              FormatRuntimeFloat(runtime.orbScaleFactor));
    ReplaceOrInsertYamlScalar(content, "ORBextractor.nLevels",
                              std::to_string(runtime.orbNLevels));
    ReplaceOrInsertYamlScalar(content, "ORBextractor.iniThFAST",
                              std::to_string(runtime.orbIniThFAST));
    ReplaceOrInsertYamlScalar(content, "ORBextractor.minThFAST",
                              std::to_string(runtime.orbMinThFAST));
}

fs::path MakeRuntimeOrbSettingsPath(const std::string &settingsPath)
{
    const fs::path sourcePath(settingsPath);
    const fs::path sourceDir =
        sourcePath.has_parent_path() ? sourcePath.parent_path() : fs::path(".");
    return sourceDir / (sourcePath.stem().string() + ".runtime_orb.yaml");
}

bool WriteRuntimeSettingsFile(const fs::path &targetPath,
                              const std::string &content)
{
    std::ofstream out(targetPath.string(), std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        std::cerr << "[slam] warning: failed to write runtime ORB settings file: "
                  << targetPath.string() << "\n";
        return false;
    }
    out << content;
    out.close();
    if (!out) {
        std::cerr << "[slam] warning: failed to flush runtime ORB settings file: "
                  << targetPath.string() << "\n";
        return false;
    }
    return true;
}

void LogRuntimeOrbSettings(const RuntimeConfig &runtime,
                           const fs::path &targetPath)
{
    std::cerr << "[slam] ORB settings override: nFeatures="
              << runtime.orbNFeatures << " scaleFactor="
              << runtime.orbScaleFactor << " nLevels=" << runtime.orbNLevels
              << " iniThFAST=" << runtime.orbIniThFAST
              << " minThFAST=" << runtime.orbMinThFAST
              << " file=" << targetPath.string() << "\n";
}

void ResetOrbAccelerationEnvironment()
{
#if defined(_WIN32)
    _putenv_s("SMART_DRONE_ORB_ACCEL", "");
    _putenv_s("SMART_DRONE_ORB_VPI_REMAP", "");
    _putenv_s("SMART_DRONE_ORB_CUDA_PYRAMID", "");
#else
    unsetenv("SMART_DRONE_ORB_ACCEL");
    unsetenv("SMART_DRONE_ORB_VPI_REMAP");
    unsetenv("SMART_DRONE_ORB_CUDA_PYRAMID");
#endif
}

} // namespace

std::string BuildEffectiveSlamSettingsPath(const UnifiedConfig &cfg)
{
    if (!ShouldWriteRuntimeOrbSettings(cfg.app)) {
        return cfg.app.settings;
    }

    bool readOk = false;
    std::string content = ReadSettingsFile(cfg.app.settings, readOk);
    if (!readOk) {
        return cfg.app.settings;
    }
    if (content.empty()) {
        std::cerr << "[slam] warning: empty settings file for ORB override: "
                  << cfg.app.settings << "\n";
        return cfg.app.settings;
    }

    ApplyOrbExtractorRuntimeConfig(content, cfg.app.runtime);
    const fs::path targetPath = MakeRuntimeOrbSettingsPath(cfg.app.settings);
    if (!WriteRuntimeSettingsFile(targetPath, content)) {
        return cfg.app.settings;
    }

    LogRuntimeOrbSettings(cfg.app.runtime, targetPath);
    return targetPath.string();
}

void ApplyOrbAccelerationEnvironment(const std::string &acceleration)
{
    std::string normalized = acceleration;
    std::transform(
        normalized.begin(), normalized.end(), normalized.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized == "cuda" || normalized == "gpu" ||
        normalized == "opencv_cuda" || normalized == "opencv-cuda") {
#if defined(_WIN32)
        _putenv_s("SMART_DRONE_ORB_ACCEL", "cuda");
        _putenv_s("SMART_DRONE_ORB_VPI_REMAP", "");
        _putenv_s("SMART_DRONE_ORB_CUDA_PYRAMID", "");
#else
        setenv("SMART_DRONE_ORB_ACCEL", "cuda", 1);
        unsetenv("SMART_DRONE_ORB_VPI_REMAP");
        unsetenv("SMART_DRONE_ORB_CUDA_PYRAMID");
#endif
        return;
    }

    if (normalized == "vpi" || normalized == "vpi_remap" ||
        normalized == "vpi-remap" || normalized == "vpi_cuda_remap" ||
        normalized == "vpi-cuda-remap") {
#if defined(_WIN32)
        _putenv_s("SMART_DRONE_ORB_ACCEL", "");
        _putenv_s("SMART_DRONE_ORB_VPI_REMAP", "1");
        _putenv_s("SMART_DRONE_ORB_CUDA_PYRAMID", "");
#else
        unsetenv("SMART_DRONE_ORB_ACCEL");
        setenv("SMART_DRONE_ORB_VPI_REMAP", "1", 1);
        unsetenv("SMART_DRONE_ORB_CUDA_PYRAMID");
#endif
        return;
    }

    ResetOrbAccelerationEnvironment();
}

} // namespace SmartDrone::core::application
