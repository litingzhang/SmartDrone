#include "core/application/session/calib/calib_output_sync.h"

#include <cerrno>
#include <cstdio>
#include <iostream>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

namespace SmartDrone::Core::Application {

bool FlushAndSyncFile(FILE *file, const char *label)
{
    if (!file) {
        return true;
    }
    std::fflush(file);
    const int fd = ::fileno(file);
    if (fd >= 0 && ::fsync(fd) != 0) {
        std::cerr << "[calib-sync] fsync failed label=" << label << " errno=" << errno << "\n";
        return false;
    }
    return true;
}

bool SyncPathFile(const std::filesystem::path &path)
{
    const int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) {
        std::cerr << "[calib-sync] open failed path=" << path.string() << " errno=" << errno << "\n";
        return false;
    }
    const bool synced = ::fsync(fd) == 0;
    if (!synced) {
        std::cerr << "[calib-sync] fsync failed path=" << path.string() << " errno=" << errno << "\n";
    }
    ::close(fd);
    return synced;
}

bool SyncDirPath(const std::filesystem::path &path)
{
    const int fd = ::open(path.c_str(), O_RDONLY | O_DIRECTORY);
    if (fd < 0) {
        std::cerr << "[calib-sync] open dir failed path=" << path.string() << " errno=" << errno << "\n";
        return false;
    }
    const bool synced = ::fsync(fd) == 0;
    if (!synced) {
        std::cerr << "[calib-sync] fsync dir failed path=" << path.string() << " errno=" << errno << "\n";
    }
    ::close(fd);
    return synced;
}

bool FlushCalibOutputs(const CalibOutputFlushRequest &request)
{
    bool ok = true;
    ok = FlushAndSyncFile(request.cam0File, "cam0.csv") && ok;
    ok = FlushAndSyncFile(request.cam1File, "cam1.csv") && ok;
    ok = FlushAndSyncFile(request.imuFile, "imu.csv") && ok;

    for (const auto &imagePath : request.imagePaths) {
        ok = SyncPathFile(imagePath) && ok;
    }

    ok = SyncDirPath(request.cam0Dir) && ok;
    ok = SyncDirPath(request.cam1Dir) && ok;
    ok = SyncDirPath(request.root) && ok;
    return ok;
}

} // namespace SmartDrone::Core::Application
