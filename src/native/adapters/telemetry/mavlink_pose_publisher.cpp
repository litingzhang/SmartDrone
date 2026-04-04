#include "adapters/telemetry/mavlink_pose_publisher.h"

namespace smartdrone::adapters::telemetry {

MavlinkPosePublisher::MavlinkPosePublisher(Px4MavlinkGateway &serial) : m_serial(serial) {}

void MavlinkPosePublisher::PublishPose(uint64_t frameId, const core::ports::PoseEstimate &pose,
                                       const core::ports::VelocityEstimate &velocity, uint8_t resetCounter, uint16_t,
                                       int, core::ports::PoseQuality quality)
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

    m_serial.SendOdometry(frameId, mavPose, mavVelocity, MAV_FRAME_LOCAL_NED, MAV_FRAME_BODY_FRD, resetCounter,
                          odomQuality);
}

} // namespace smartdrone::adapters::telemetry
