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

enum class PoseReferenceFrame : uint8_t {
    LocalNed = 0,
    LocalFrd = 1,
};

struct PosePublishRequest {
    uint64_t frameId{0};
    PoseEstimate pose{};
    VelocityEstimate velocity{};
    PoseReferenceFrame referenceFrame{PoseReferenceFrame::LocalNed};
    uint8_t resetCounter{0};
    uint16_t resetMapCount{0};
    int trackingState{0};
    PoseQuality quality{PoseQuality::Lost};
};

class IPosePublisher {
  public:
    virtual ~IPosePublisher() = default;

    virtual void PublishPose(const PosePublishRequest &request) = 0;
};

} // namespace SmartDrone::Core::Ports
