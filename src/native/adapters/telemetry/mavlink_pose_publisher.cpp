#include "adapters/telemetry/mavlink_pose_publisher.h"

namespace SmartDrone::Adapters::Telemetry {

MavlinkPosePublisher::MavlinkPosePublisher(Px4MavlinkGateway &serial)
    : m_serial(serial)
{
}

void MavlinkPosePublisher::PublishPose(uint64_t frameId, const Core::Ports::PoseEstimate &pose,
                                       const Core::Ports::VelocityEstimate &velocity, uint8_t resetCounter, uint16_t,
                                       int, Core::Ports::PoseQuality quality)
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
    if (quality == Core::Ports::PoseQuality::Weak) {
        odomQuality = OdomQualityMode::WEAK;
    } else if (quality == Core::Ports::PoseQuality::Lost) {
        odomQuality = OdomQualityMode::LOST;
    }

    Px4MavlinkGateway::OdometryRequest request{};
    request.frameId = frameId;
    request.poseNed = mavPose;
    request.velocityNed = mavVelocity;
    request.mavFrameId = MAV_FRAME_LOCAL_FRD;
    request.childFrameId = MAV_FRAME_BODY_FRD;
    request.resetCounter = resetCounter;
    request.qualityMode = odomQuality;
    m_serial.SendOdometry(request);
}

} // namespace SmartDrone::Adapters::Telemetry
