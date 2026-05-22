#pragma once

#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "core/ports/pose_publisher.h"

namespace SmartDrone::Adapters::Telemetry {

class MavlinkPosePublisher final : public Core::Ports::IPosePublisher {
  public:
    explicit MavlinkPosePublisher(Px4MavlinkGateway &serial);

    void PublishPose(uint64_t frameId, const Core::Ports::PoseEstimate &pose,
                     const Core::Ports::VelocityEstimate &velocity, uint8_t resetCounter, uint16_t resetMapCount,
                     int trackingState, Core::Ports::PoseQuality quality) override;

  private:
    Px4MavlinkGateway &m_serial;
};

} // namespace SmartDrone::Adapters::Telemetry
