#pragma once

#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "core/ports/pose_publisher.h"

namespace SmartDrone::adapters::telemetry {

class MavlinkPosePublisher final : public core::ports::IPosePublisher {
  public:
    explicit MavlinkPosePublisher(Px4MavlinkGateway &serial);

    void PublishPose(uint64_t frameId, const core::ports::PoseEstimate &pose,
                     const core::ports::VelocityEstimate &velocity, uint8_t resetCounter, uint16_t resetMapCount,
                     int trackingState, core::ports::PoseQuality quality) override;

  private:
    Px4MavlinkGateway &m_serial;
};

} // namespace SmartDrone::adapters::telemetry
