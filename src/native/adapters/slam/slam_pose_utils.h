#pragma once

#include <string>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "core/ports/slam_engine.h"

namespace SmartDrone::Adapters::Slam {

constexpr double kPoseStabilizerDefaultDtSec = 1.0 / 20.0;
constexpr double kPoseStabilizerMinDtSec = 1.0 / 120.0;
constexpr double kPoseStabilizerMaxDtSec = 0.25;
constexpr float kPoseStabilizerMaxSpeedMps = 3.0f;
constexpr float kPoseStabilizerMaxStepMeters = 0.055f;
constexpr float kPoseStabilizerMaxRotStepDeg = 3.0f;
constexpr float kPoseStabilizerVelocityAlpha = 0.35f;
constexpr float kPoseStabilizerPredictedVelocityDecay = 0.985f;

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
void ClampVelocityVector(float &vx, float &vy, float &vz, float maxSpeed = kPoseStabilizerMaxSpeedMps);

Sophus::SE3f PoseEstimateToSe3(const Core::Ports::PoseEstimate &pose);
Core::Ports::PoseEstimate Se3ToPoseEstimate(const Sophus::SE3f &pose);
Core::Ports::PoseEstimate PoseFromTwc(const Sophus::SE3f &twc);

} // namespace SmartDrone::Adapters::Slam
