#include "core/application/session/calib_session_service.h"

#include <cerrno>
#include <cstdio>
#include <iostream>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include "core/application/session/calib_session_graph_service.h"
#include "core/application/session/calib_storage_helpers.h"

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

void SyncPathFile(const fs::path &path)
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

void SyncDirPath(const fs::path &path)
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

void FlushCalibOutputs(FILE *fCam0, FILE *fCam1, FILE *fImu, const std::vector<fs::path> &imagePaths,
                       const fs::path &root, const fs::path &cam0Dir, const fs::path &cam1Dir)
{
    FlushAndSyncFile(fCam0, "cam0.csv");
    FlushAndSyncFile(fCam1, "cam1.csv");
    FlushAndSyncFile(fImu, "imu.csv");

    for (const auto &imagePath : imagePaths) {
        SyncPathFile(imagePath);
    }

    SyncDirPath(cam0Dir);
    SyncDirPath(cam1Dir);
    SyncDirPath(root);
}

bool RunCalibSession(const UnifiedConfig &cfg, std::atomic<bool> &stop, LivePoseState &livePose,
                     std::atomic<bool> &runningFlag)
{
    return RunCalibSessionGraph(cfg, stop, livePose, runningFlag);
}

} // namespace smartdrone::core::application
