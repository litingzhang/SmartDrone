#pragma once

#include <cstdint>

#include "slam_engine.h"

namespace SmartDrone::Core::Ports {

struct VelocityEstimate {
    float vx{0.0f};
    float vy{0.0f};
    float vz{0.0f};
    bool valid{false};
};

enum class PoseQuality : uint8_t {
    Good = 0,
    Weak = 1,
    Lost = 2,
};

class IPosePublisher {
  public:
    virtual ~IPosePublisher() = default;

    virtual void PublishPose(uint64_t frameId, const PoseEstimate &pose, const VelocityEstimate &velocity,
                             uint8_t resetCounter, uint16_t resetMapCount, int trackingState, PoseQuality quality) = 0;
};

} // namespace SmartDrone::Core::Ports
