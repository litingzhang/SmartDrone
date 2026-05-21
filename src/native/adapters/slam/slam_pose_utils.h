#pragma once

#include <string>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "core/ports/slam_engine.h"

namespace smartdrone::adapters::slam {

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

bool IsIdentityPose(const core::ports::PoseEstimate &pose);
bool HasIdentityPoseValues(const core::ports::PoseEstimate &pose);
bool IsFinitePose(const core::ports::PoseEstimate &pose);
void NormalizePoseQuaternion(core::ports::PoseEstimate &pose);
float PoseTranslationDistance(const core::ports::PoseEstimate &a, const core::ports::PoseEstimate &b);
Eigen::Quaternionf PoseQuaternion(const core::ports::PoseEstimate &pose);
float QuaternionAngleDeg(const Eigen::Quaternionf &a, const Eigen::Quaternionf &b);
void LimitPoseRotationStep(const core::ports::PoseEstimate &reference, core::ports::PoseEstimate &pose,
                           float maxRotationStepDeg);
void ClampVelocityVector(float &vx, float &vy, float &vz, float maxSpeed = kPoseStabilizerMaxSpeedMps);

Sophus::SE3f PoseEstimateToSe3(const core::ports::PoseEstimate &pose);
core::ports::PoseEstimate Se3ToPoseEstimate(const Sophus::SE3f &pose);
core::ports::PoseEstimate PoseFromTwc(const Sophus::SE3f &twc);

} // namespace smartdrone::adapters::slam
