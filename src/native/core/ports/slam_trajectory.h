#pragma once

#include <string>

#include <sophus/se3.hpp>

namespace SmartDrone::Core::Ports {

class ISlamTrajectoryProvider {
  public:
    virtual ~ISlamTrajectoryProvider() = default;

    virtual bool
    GetLatestFrameTrajectoryPoseEuRoC(Sophus::SE3f &twc,
                                      double *timestamp = nullptr,
                                      bool *lost = nullptr) const = 0;
};

class ISlamTrajectorySaver {
  public:
    virtual ~ISlamTrajectorySaver() = default;

    virtual bool ShutdownAndSaveTrajectoryEuRoC(const std::string &path) = 0;
};

} // namespace SmartDrone::Core::Ports
