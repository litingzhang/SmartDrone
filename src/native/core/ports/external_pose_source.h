#pragma once

#include "core/ports/pose_publisher.h"

namespace SmartDrone::Core::Ports {

class IExternalPoseSource {
  public:
    virtual ~IExternalPoseSource() = default;

    virtual bool TryRead(PosePublishRequest &out) = 0;
    virtual bool Healthy() const = 0;
};

} // namespace SmartDrone::Core::Ports
