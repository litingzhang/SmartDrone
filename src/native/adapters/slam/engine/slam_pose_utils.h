#pragma once

#include <string>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "core/ports/slam_engine.h"

namespace SmartDrone::Adapters::Slam {

constexpr double POSE_STABILIZER_DEFAULT_DT_SEC = 1.0 / 20.0;
constexpr double POSE_STABILIZER_MIN_DT_SEC = 1.0 / 120.0;
constexpr double POSE_STABILIZER_MAX_DT_SEC = 0.25;
constexpr float POSE_STABILIZER_MAX_SPEED_MPS = 3.0f;
constexpr float POSE_STABILIZER_MAX_STEP_METERS = 0.055f;
constexpr float POSE_STABILIZER_MAX_ROT_STEP_DEG = 3.0f;
constexpr float POSE_STABILIZER_VELOCITY_ALPHA = 0.35f;
constexpr float POSE_STABILIZER_PREDICTED_VELOCITY_DECAY = 0.985f;

bool TrackingStateCanPublishPose(int trackingState);
bool IsSuperPointTrackingStateSafe(int trackingState);
bool IsOrbBootstrapState(int trackingState);
std::string DescribeTrackingState(int trackingState);

bool IsIdentityPose(const Core::Ports::PoseEstimate &pose);
bool HasIdentityPoseValues(const Core::Ports::PoseEstimate &pose);
bool IsFinitePose(const Core::Ports::PoseEstimate &pose);
void NormalizePoseQuaternion(Core::Ports::PoseEstimate &pose);
float PoseTranslationDistance(const Core::Ports::PoseEstimate &a, const Core::Ports::PoseEstimate &b);
Eigen::Quaternionf PoseQuaternion(const Core::Ports::PoseEstimate &pose);
float QuaternionAngleDeg(const Eigen::Quaternionf &a, const Eigen::Quaternionf &b);
void LimitPoseRotationStep(const Core::Ports::PoseEstimate &reference, Core::Ports::PoseEstimate &pose,
                           float maxRotationStepDeg);
void ClampVelocityVector(float &vx, float &vy, float &vz, float maxSpeed = POSE_STABILIZER_MAX_SPEED_MPS);

Sophus::SE3f PoseEstimateToSe3(const Core::Ports::PoseEstimate &pose);
Core::Ports::PoseEstimate Se3ToPoseEstimate(const Sophus::SE3f &pose);
Core::Ports::PoseEstimate PoseFromTwc(const Sophus::SE3f &twc);

} // namespace SmartDrone::Adapters::Slam
