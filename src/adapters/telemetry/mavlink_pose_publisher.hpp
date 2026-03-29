#pragma once

#include "adapters/telemetry/px4_mavlink_gateway.hpp"
#include "core/ports/pose_publisher.hpp"

namespace smartdrone::adapters::telemetry {

class MavlinkPosePublisher final : public core::ports::IPosePublisher {
public:
    explicit MavlinkPosePublisher(Px4MavlinkGateway& serial) : m_serial(serial) {}

    void PublishPose(uint64_t timestampUs,
                     const core::ports::PoseEstimate& pose,
                     const core::ports::VelocityEstimate& velocity,
                     uint8_t resetCounter,
                     uint16_t,
                     int,
                     core::ports::PoseQuality quality) override
    {
        Px4MavlinkGateway::Pose mavPose{};
        mavPose.x = pose.x;
        mavPose.y = pose.y;
        mavPose.z = pose.z;
        mavPose.qw = pose.qw;
        mavPose.qx = pose.qx;
        mavPose.qy = pose.qy;
        mavPose.qz = pose.qz;

        Px4MavlinkGateway::LinearVelocityNed mavVelocity{};
        if (velocity.valid) {
            mavVelocity.x = velocity.vx;
            mavVelocity.y = velocity.vy;
            mavVelocity.z = velocity.vz;
        }

        OdomQualityMode odomQuality = OdomQualityMode::GOOD;
        if (quality == core::ports::PoseQuality::Weak) {
            odomQuality = OdomQualityMode::WEAK;
        } else if (quality == core::ports::PoseQuality::Lost) {
            odomQuality = OdomQualityMode::LOST;
        }

        m_serial.SendOdometry(
            timestampUs,
            mavPose,
            mavVelocity,
            MAV_FRAME_LOCAL_NED,
            MAV_FRAME_BODY_FRD,
            resetCounter,
            odomQuality);
    }

private:
    Px4MavlinkGateway& m_serial;
};

}  // namespace smartdrone::adapters::telemetry
