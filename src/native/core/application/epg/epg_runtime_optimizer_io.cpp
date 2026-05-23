#include "core/application/epg/epg_runtime_optimizer_io.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace SmartDrone::Core::Application {
namespace {

std::string StripGeneratedMetadata(std::string text)
{
    const std::vector<std::string> fields = {
        "generatedAtMs",
        "sourceTimestampMs",
    };
    for (const auto &field : fields) {
        const auto key = "\"" + field + "\":";
        auto pos = text.find(key);
        if (pos == std::string::npos) {
            continue;
        }
        auto end = text.find('\n', pos);
        if (end == std::string::npos) {
            text.erase(pos);
            continue;
        }
        text.erase(pos, end - pos + 1);
    }
    return text;
}

void EnsureArtifactDirectory(const std::string &path)
{
    const std::filesystem::path artifactPath(path);
    const auto parent = artifactPath.parent_path();
    if (parent.empty()) {
        return;
    }
    std::error_code error;
    if (std::filesystem::exists(parent, error) &&
        !std::filesystem::is_directory(parent, error)) {
        throw std::runtime_error("EPG artifact directory is not a directory: " +
                                 parent.string());
    }
    std::filesystem::create_directories(parent, error);
    if (error) {
        throw std::runtime_error("failed to create EPG artifact directory: " +
                                 parent.string());
    }
}

} // namespace

std::string ReadEpgOptimizerFile(const std::string &path)
{
    std::ifstream input(path);
    if (!input) {
        return {};
    }
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
}

bool EpgOptimizedConfigChanged(const std::string &oldJson,
                               const std::string &newJson)
{
    return StripGeneratedMetadata(oldJson) != StripGeneratedMetadata(newJson);
}

void WriteRequiredEpgOptimizerArtifactFile(const std::string &path,
                                           const std::string &text)
{
    EnsureArtifactDirectory(path);
    const std::string tempPath = path + ".tmp";
    {
        std::ofstream output(tempPath, std::ios::out | std::ios::trunc);
        if (!output) {
            throw std::runtime_error("failed to open EPG artifact: " +
                                     tempPath);
        }
        output << text;
        if (!output) {
            throw std::runtime_error("failed to write EPG artifact: " +
                                     tempPath);
        }
    }
    if (std::rename(tempPath.c_str(), path.c_str()) != 0) {
        (void)std::remove(tempPath.c_str());
        throw std::runtime_error("failed to publish EPG artifact: " + path);
    }
}

} // namespace SmartDrone::Core::Application
