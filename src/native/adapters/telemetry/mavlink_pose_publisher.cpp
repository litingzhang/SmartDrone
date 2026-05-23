#include "adapters/telemetry/mavlink_pose_publisher.h"

namespace SmartDrone::Adapters::Telemetry {

MavlinkPosePublisher::MavlinkPosePublisher(Px4MavlinkGateway &serial)
    : m_serial(serial)
{
}

void MavlinkPosePublisher::PublishPose(
    const Core::Ports::PosePublishRequest &request)
{
    Px4MavlinkGateway::Pose mavPose{};
    mavPose.x = request.pose.x;
    mavPose.y = request.pose.y;
    mavPose.z = request.pose.z;
    mavPose.qw = request.pose.qw;
    mavPose.qx = request.pose.qx;
    mavPose.qy = request.pose.qy;
    mavPose.qz = request.pose.qz;

    Px4MavlinkGateway::LinearVelocityNed mavVelocity{};
    if (request.velocity.valid) {
        mavVelocity.x = request.velocity.vx;
        mavVelocity.y = request.velocity.vy;
        mavVelocity.z = request.velocity.vz;
    }

    OdomQualityMode odomQuality = OdomQualityMode::GOOD;
    if (request.quality == Core::Ports::PoseQuality::Weak) {
        odomQuality = OdomQualityMode::WEAK;
    } else if (request.quality == Core::Ports::PoseQuality::Lost) {
        odomQuality = OdomQualityMode::LOST;
    }

    Px4MavlinkGateway::OdometryRequest odometry{};
    odometry.frameId = request.frameId;
    odometry.poseNed = mavPose;
    odometry.velocityNed = mavVelocity;
    odometry.mavFrameId = MAV_FRAME_LOCAL_FRD;
    odometry.childFrameId = MAV_FRAME_BODY_FRD;
    odometry.resetCounter = request.resetCounter;
    odometry.qualityMode = odomQuality;
    m_serial.SendOdometry(odometry);
}

} // namespace SmartDrone::Adapters::Telemetry
