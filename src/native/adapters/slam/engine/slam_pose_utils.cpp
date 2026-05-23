#include "adapters/slam/engine/slam_pose_utils.h"

#include <algorithm>
#include <cmath>

#include "core/ports/slam_tracking_state.h"

namespace SmartDrone::Adapters::Slam {

bool TrackingStateCanPublishPose(int trackingState)
{
    return trackingState == Core::Ports::kSlamTrackingOk ||
           trackingState == Core::Ports::kSlamTrackingRecentlyLost ||
           trackingState == Core::Ports::kSlamTrackingOkKlt;
}

bool IsSuperPointTrackingStateSafe(int trackingState)
{
    switch (trackingState) {
    case Core::Ports::kSlamTrackingOk:
    case Core::Ports::kSlamTrackingRecentlyLost:
    case Core::Ports::kSlamTrackingLost:
    case Core::Ports::kSlamTrackingOkKlt:
        return true;
    default:
        return false;
    }
}

bool IsOrbBootstrapState(int trackingState)
{
    return trackingState == Core::Ports::kSlamTrackingNoImagesYet ||
           trackingState == Core::Ports::kSlamTrackingNotInitialized;
}

std::string DescribeTrackingState(int trackingState)
{
    switch (trackingState) {
    case Core::Ports::kSlamTrackingSystemNotReady:
        return "system_not_ready";
    case Core::Ports::kSlamTrackingNoImagesYet:
        return "no_images_yet";
    case Core::Ports::kSlamTrackingNotInitialized:
        return "not_initialized";
    case Core::Ports::kSlamTrackingOk:
        return "ok";
    case Core::Ports::kSlamTrackingRecentlyLost:
        return "recently_lost";
    case Core::Ports::kSlamTrackingLost:
        return "lost";
    case Core::Ports::kSlamTrackingOkKlt:
        return "ok_klt";
    default:
        return "unknown";
    }
}

bool IsIdentityPose(const Core::Ports::PoseEstimate &pose)
{
    return pose.valid && HasIdentityPoseValues(pose);
}

bool HasIdentityPoseValues(const Core::Ports::PoseEstimate &pose)
{
    return pose.x == 0.0f && pose.y == 0.0f && pose.z == 0.0f && pose.qw == 1.0f &&
           pose.qx == 0.0f && pose.qy == 0.0f && pose.qz == 0.0f;
}

bool IsFinitePose(const Core::Ports::PoseEstimate &pose)
{
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) &&
           std::isfinite(pose.qw) && std::isfinite(pose.qx) && std::isfinite(pose.qy) &&
           std::isfinite(pose.qz);
}

void NormalizePoseQuaternion(Core::Ports::PoseEstimate &pose)
{
    const float qNorm =
        std::sqrt(pose.qw * pose.qw + pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz);
    if (qNorm > 1.0e-6f && std::isfinite(qNorm)) {
        pose.qw /= qNorm;
        pose.qx /= qNorm;
        pose.qy /= qNorm;
        pose.qz /= qNorm;
    }
}

float PoseTranslationDistance(const Core::Ports::PoseEstimate &a, const Core::Ports::PoseEstimate &b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Eigen::Quaternionf PoseQuaternion(const Core::Ports::PoseEstimate &pose)
{
    Eigen::Quaternionf q(pose.qw, pose.qx, pose.qy, pose.qz);
    q.normalize();
    return q;
}

float QuaternionAngleDeg(const Eigen::Quaternionf &a, const Eigen::Quaternionf &b)
{
    const float dot = std::min(1.0f, std::max(-1.0f, std::abs(a.dot(b))));
    return 2.0f * std::acos(dot) * 180.0f / static_cast<float>(M_PI);
}

void LimitPoseRotationStep(const Core::Ports::PoseEstimate &reference,
                           Core::Ports::PoseEstimate &pose,
                           float maxRotationStepDeg)
{
    const Eigen::Quaternionf qa = PoseQuaternion(reference);
    Eigen::Quaternionf qb = PoseQuaternion(pose);
    if (qa.dot(qb) < 0.0f) {
        qb.coeffs() *= -1.0f;
    }

    const float angleDeg = QuaternionAngleDeg(qa, qb);
    if (angleDeg > maxRotationStepDeg && angleDeg > 1.0e-6f) {
        const float t = maxRotationStepDeg / angleDeg;
        qb = qa.slerp(t, qb);
    }

    qb.normalize();
    pose.qw = qb.w();
    pose.qx = qb.x();
    pose.qy = qb.y();
    pose.qz = qb.z();
}

void ClampVelocityVector(float &vx, float &vy, float &vz, float maxSpeed)
{
    const float speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (speed > maxSpeed && speed > 1.0e-6f) {
        const float scale = maxSpeed / speed;
        vx *= scale;
        vy *= scale;
        vz *= scale;
    }
}

Sophus::SE3f PoseEstimateToSe3(const Core::Ports::PoseEstimate &pose)
{
    Eigen::Quaternionf q(pose.qw, pose.qx, pose.qy, pose.qz);
    q.normalize();
    return Sophus::SE3f(Sophus::SO3f(q), Eigen::Vector3f(pose.x, pose.y, pose.z));
}

Core::Ports::PoseEstimate Se3ToPoseEstimate(const Sophus::SE3f &pose)
{
    const Eigen::Vector3f t = pose.translation();
    const Eigen::Quaternionf q(pose.so3().unit_quaternion());
    Core::Ports::PoseEstimate out{};
    out.valid = true;
    out.x = t.x();
    out.y = t.y();
    out.z = t.z();
    out.qw = q.w();
    out.qx = q.x();
    out.qy = q.y();
    out.qz = q.z();
    return out;
}

Core::Ports::PoseEstimate PoseFromTwc(const Sophus::SE3f &twc)
{
    Core::Ports::PoseEstimate pose{};
    const Eigen::Vector3f t = twc.translation();
    const Eigen::Quaternionf q(twc.so3().unit_quaternion());
    pose.valid = std::isfinite(t.x()) && std::isfinite(t.y()) && std::isfinite(t.z()) &&
                 std::isfinite(q.w()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z());
    pose.x = t.x();
    pose.y = t.y();
    pose.z = t.z();
    pose.qw = q.w();
    pose.qx = q.x();
    pose.qy = q.y();
    pose.qz = q.z();
    return pose;
}

} // namespace SmartDrone::Adapters::Slam
