#pragma once

#include <memory>
#include <string>

#include <sophus/se3.hpp>

#include "core/ports/slam_backend.h"

namespace SmartDrone::Adapters::Slam {

class OpenVinsRuntimeDriver {
  public:
    virtual ~OpenVinsRuntimeDriver() = default;

    virtual bool Available() const = 0;
    virtual void Shutdown() = 0;
    virtual Sophus::SE3f TrackRaw(
        const Core::Ports::SlamTrackRequest &request) = 0;
    virtual bool Initialized() const = 0;
};

std::unique_ptr<OpenVinsRuntimeDriver>
CreateOpenVinsRuntimeDriver(const std::string &settingsPath);

} // namespace SmartDrone::Adapters::Slam
