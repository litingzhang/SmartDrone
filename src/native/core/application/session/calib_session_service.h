#pragma once

#include <atomic>
#include <cstdio>
#include <vector>

#include "core/application/config/runtime_app_types.h"
#include "core/application/state/live_pose_state.h"

namespace smartdrone::core::application {

void FlushAndSyncFile(FILE *file, const char *label);
void SyncPathFile(const fs::path &path);
void SyncDirPath(const fs::path &path);
void FlushCalibOutputs(FILE *fCam0, FILE *fCam1, FILE *fImu, const std::vector<fs::path> &imagePaths,
                       const fs::path &root, const fs::path &cam0Dir, const fs::path &cam1Dir);
bool RunCalibSession(const UnifiedConfig &cfg, std::atomic<bool> &stop, LivePoseState &livePose,
                     std::atomic<bool> &runningFlag);

} // namespace smartdrone::core::application
