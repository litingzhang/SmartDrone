#include "adapters/telemetry/mavlink_pose_publisher.h"

#include <Eigen/Geometry>

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
        const Eigen::Quaternionf localFromBody(
            request.pose.qw, request.pose.qx, request.pose.qy, request.pose.qz);
        const Eigen::Vector3f localVelocity(
            request.velocity.vx, request.velocity.vy, request.velocity.vz);
        const Eigen::Vector3f bodyVelocity =
            localFromBody.normalized().conjugate() * localVelocity;
        mavVelocity.x = bodyVelocity.x();
        mavVelocity.y = bodyVelocity.y();
        mavVelocity.z = bodyVelocity.z();
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
    odometry.mavFrameId =
        request.referenceFrame == Core::Ports::PoseReferenceFrame::LocalNed
            ? MAV_FRAME_LOCAL_NED
            : MAV_FRAME_LOCAL_FRD;
    odometry.childFrameId = MAV_FRAME_BODY_FRD;
    odometry.resetCounter = request.resetCounter;
    odometry.qualityMode = odomQuality;
    m_serial.SendOdometry(odometry);
}

} // namespace SmartDrone::Adapters::Telemetry
