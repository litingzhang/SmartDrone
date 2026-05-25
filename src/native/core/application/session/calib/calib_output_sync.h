#pragma once

#include <cstdio>
#include <filesystem>
#include <vector>

namespace SmartDrone::Core::Application {

struct CalibOutputFlushRequest {
    FILE *cam0File;
    FILE *cam1File;
    FILE *imuFile;
    const std::vector<std::filesystem::path> &imagePaths;
    const std::filesystem::path &root;
    const std::filesystem::path &cam0Dir;
    const std::filesystem::path &cam1Dir;
};

bool FlushAndSyncFile(FILE *file, const char *label);
bool SyncPathFile(const std::filesystem::path &path);
bool SyncDirPath(const std::filesystem::path &path);
bool FlushCalibOutputs(const CalibOutputFlushRequest &request);

} // namespace SmartDrone::Core::Application
