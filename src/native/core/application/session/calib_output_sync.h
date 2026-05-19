#pragma once

#include <cstdio>
#include <filesystem>
#include <vector>

namespace smartdrone::core::application {

struct CalibOutputFlushRequest {
    FILE *cam0File;
    FILE *cam1File;
    FILE *imuFile;
    const std::vector<std::filesystem::path> &imagePaths;
    const std::filesystem::path &root;
    const std::filesystem::path &cam0Dir;
    const std::filesystem::path &cam1Dir;
};

void FlushAndSyncFile(FILE *file, const char *label);
void SyncPathFile(const std::filesystem::path &path);
void SyncDirPath(const std::filesystem::path &path);
void FlushCalibOutputs(const CalibOutputFlushRequest &request);

} // namespace smartdrone::core::application
