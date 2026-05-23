#pragma once

#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "core/ports/pose_publisher.h"

namespace SmartDrone::Adapters::Telemetry {

class MavlinkPosePublisher final : public Core::Ports::IPosePublisher {
  public:
    explicit MavlinkPosePublisher(Px4MavlinkGateway &serial);

    void PublishPose(const Core::Ports::PosePublishRequest &request) override;

  private:
    Px4MavlinkGateway &m_serial;
};

} // namespace SmartDrone::Adapters::Telemetry
