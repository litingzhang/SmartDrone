#include "core/application/session/calib/calib_output_sync.h"

#include <cerrno>
#include <cstdio>
#include <iostream>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

namespace smartdrone::core::application {

void FlushAndSyncFile(FILE *file, const char *label)
{
    if (!file) {
        return;
    }
    std::fflush(file);
    const int fd = ::fileno(file);
    if (fd >= 0 && ::fsync(fd) != 0) {
        std::cerr << "[calib-sync] fsync failed label=" << label << " errno=" << errno << "\n";
    }
}

void SyncPathFile(const std::filesystem::path &path)
{
    const int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) {
        std::cerr << "[calib-sync] open failed path=" << path.string() << " errno=" << errno << "\n";
        return;
    }
    if (::fsync(fd) != 0) {
        std::cerr << "[calib-sync] fsync failed path=" << path.string() << " errno=" << errno << "\n";
    }
    ::close(fd);
}

void SyncDirPath(const std::filesystem::path &path)
{
    const int fd = ::open(path.c_str(), O_RDONLY | O_DIRECTORY);
    if (fd < 0) {
        std::cerr << "[calib-sync] open dir failed path=" << path.string() << " errno=" << errno << "\n";
        return;
    }
    if (::fsync(fd) != 0) {
        std::cerr << "[calib-sync] fsync dir failed path=" << path.string() << " errno=" << errno << "\n";
    }
    ::close(fd);
}

void FlushCalibOutputs(const CalibOutputFlushRequest &request)
{
    FlushAndSyncFile(request.cam0File, "cam0.csv");
    FlushAndSyncFile(request.cam1File, "cam1.csv");
    FlushAndSyncFile(request.imuFile, "imu.csv");

    for (const auto &imagePath : request.imagePaths) {
        SyncPathFile(imagePath);
    }

    SyncDirPath(request.cam0Dir);
    SyncDirPath(request.cam1Dir);
    SyncDirPath(request.root);
}

} // namespace smartdrone::core::application
