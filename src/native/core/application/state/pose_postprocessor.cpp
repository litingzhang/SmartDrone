#include "core/application/state/pose_postprocessor.h"

#include <cmath>

#include "common/time_utils.h"

namespace smartdrone::core::application {

Sophus::SE3f PosePostprocessor::ContinuityMapper::MapPose(unsigned long mapId, bool trackingUsable,
                                                          const Sophus::SE3f &rawPoseWc)
{
    if (mapId != kInvalidMapId && (!haveMapId || mapId != lastMapId)) {
        pendingReset = haveMapId;
        haveMapId = true;
        lastMapId = mapId;
    }

    if (!trackingUsable) {
        return bridgeRawToContinuous * rawPoseWc;
    }

    if (!bridgeInitialized) {
        bridgeRawToContinuous = Sophus::SE3f();
        bridgeInitialized = true;
    } else if (pendingReset) {
        if (haveLastContinuousPose) {
            bridgeRawToContinuous = lastContinuousPose * rawPoseWc.inverse();
        } else {
            bridgeRawToContinuous = Sophus::SE3f();
        }
        pendingReset = false;
        ++resetCounter;
        ++resetMapCount;
    }

    const Sophus::SE3f continuousPose = bridgeRawToContinuous * rawPoseWc;
    lastContinuousPose = continuousPose;
    haveLastContinuousPose = true;
    return continuousPose;
}

uint8_t PosePostprocessor::ContinuityMapper::GetResetCounter() const
{
    return static_cast<uint8_t>(resetCounter & 0xFFu);
}

uint16_t PosePostprocessor::ContinuityMapper::GetResetMapCount() const
{
    return static_cast<uint16_t>(resetMapCount & 0xFFFFu);
}

Px4MavlinkGateway::Pose PosePostprocessor::StartupAligner::AlignPose(const Px4MavlinkGateway::Pose &poseNed,
                                                                     bool trackingUsable, Px4MavlinkGateway &mavlink,
                                                                     PoseQuality &outQuality)
{
    const uint64_t nowUs = MonoTimeUs();
    RefreshRangeSensor(mavlink);

    if (!trackingUsable) {
        Px4MavlinkGateway::Pose out = ComputeLostPose(nowUs);
        outQuality = havePublishedPose ? PoseQuality::Weak : PoseQuality::Lost;
        trackingUsablePrev = false;
        return out;
    }

    if (!haveZOffset) {
        const float worldZ = havePublishedPose ? holdPose.z : 0.0f;
        zOffset = worldZ - poseNed.z;
        haveZOffset = true;
        weakUntilUs = nowUs + kWeakHoldUs;
    }

    Px4MavlinkGateway::Pose out = poseNed;
    out.z += zOffset;

    trackingUsablePrev = true;
    lossActive = false;
    holdPose = out;
    havePublishedPose = true;
    outQuality = (nowUs < weakUntilUs) ? PoseQuality::Weak : PoseQuality::Good;
    return out;
}

void PosePostprocessor::StartupAligner::RefreshRangeSensor(Px4MavlinkGateway &mavlink)
{
    Px4MavlinkGateway::DownwardDistanceSensor rng{};
    if (mavlink.GetDownwardDistanceSensor(rng, kRangeSensorMaxAgeUs)) {
        latestRange = rng;
        haveLatestRange = true;
    }
}

bool PosePostprocessor::StartupAligner::HasFreshRange() const
{
    return haveLatestRange && std::isfinite(latestRange.currentDistance);
}

Px4MavlinkGateway::Pose PosePostprocessor::StartupAligner::ComputeLostPose(uint64_t nowUs)
{
    Px4MavlinkGateway::Pose out = holdPose;
    if (!lossActive) {
        lossActive = true;
        if (havePublishedPose) {
            holdPose = out;
        }
    }
    ApplyRangeProtection(out, nowUs);
    holdPose = out;
    return out;
}

void PosePostprocessor::StartupAligner::ApplyRangeProtection(Px4MavlinkGateway::Pose &, uint64_t)
{
    if (!HasFreshRange()) {
        return;
    }
    if (latestRange.signalQuality == 0 || !std::isfinite(latestRange.currentDistance)) {
        return;
    }
    if (latestRange.currentDistance >= kRangeHardFloorM) {
        return;
    }
}

smartdrone::core::ports::VelocityEstimate
PosePostprocessor::VelocityTracker::Update(const Px4MavlinkGateway::Pose &pose, int64_t frameNs, PoseQuality quality,
                                           uint16_t resetMapCount)
{
    smartdrone::core::ports::VelocityEstimate out{};

    if (quality != PoseQuality::Good || !haveLastPose || lastQuality != PoseQuality::Good ||
        resetMapCount != lastResetMapCount || frameNs <= lastFrameNs) {
        ResetState(pose, frameNs, quality, resetMapCount);
        return out;
    }

    const float dt = static_cast<float>(frameNs - lastFrameNs) * 1e-9f;
    if (!(dt >= 0.005f) || !(dt <= 0.2f)) {
        ResetState(pose, frameNs, quality, resetMapCount);
        return out;
    }

    const float rawVx = (pose.x - lastPose.x) / dt;
    const float rawVy = (pose.y - lastPose.y) / dt;
    const float rawVz = (pose.z - lastPose.z) / dt;

    if (!std::isfinite(rawVx) || !std::isfinite(rawVy) || !std::isfinite(rawVz) ||
        std::fabs(rawVx) > kMaxHorizontalSpeedMps || std::fabs(rawVy) > kMaxHorizontalSpeedMps ||
        std::fabs(rawVz) > kMaxVerticalSpeedMps) {
        ResetState(pose, frameNs, quality, resetMapCount);
        return out;
    }

    const float alphaXY = dt / (kHorizontalTauSec + dt);
    const float alphaZ = dt / (kVerticalTauSec + dt);

    if (!haveFilteredVelocity) {
        filteredVelocity.vx = rawVx;
        filteredVelocity.vy = rawVy;
        filteredVelocity.vz = rawVz;
        filteredVelocity.valid = true;
        haveFilteredVelocity = true;
    } else {
        filteredVelocity.vx += alphaXY * (rawVx - filteredVelocity.vx);
        filteredVelocity.vy += alphaXY * (rawVy - filteredVelocity.vy);
        filteredVelocity.vz += alphaZ * (rawVz - filteredVelocity.vz);
    }

    out = filteredVelocity;
    lastPose = pose;
    lastFrameNs = frameNs;
    lastQuality = quality;
    lastResetMapCount = resetMapCount;
    haveLastPose = true;
    return out;
}

void PosePostprocessor::VelocityTracker::ResetState(const Px4MavlinkGateway::Pose &pose, int64_t frameNs,
                                                    PoseQuality quality, uint16_t resetMapCount)
{
    lastPose = pose;
    lastFrameNs = frameNs;
    lastQuality = quality;
    lastResetMapCount = resetMapCount;
    filteredVelocity = {};
    haveFilteredVelocity = false;
    haveLastPose = true;
}

PosePostprocessor::Result PosePostprocessor::ProcessPose(const Sophus::SE3f &twcRaw, bool useImu, bool trackingUsable,
                                                         int, unsigned long mapId, bool stereoExtrinsicsLoaded,
                                                         const Sophus::SE3f &stereoBodyExtrinsics,
                                                         bool &stereoReferencePoseSet,
                                                         Sophus::SE3f &stereoReferencePose, int64_t frameNs,
                                                         Px4MavlinkGateway &mavlink)
{
    Sophus::SE3f twc = twcRaw;
    Result out{};
    out.debug.cameraX = twcRaw.translation().x();
    out.debug.cameraY = twcRaw.translation().y();
    out.debug.cameraZ = twcRaw.translation().z();

    if (!useImu && stereoExtrinsicsLoaded) {
        // Pure stereo/LK estimates the left optical camera pose. Convert it to body FRD using T_b_c1
        // before the startup reference is applied, so published local motion is +X forward, +Y right, +Z down.
        twc = twc * stereoBodyExtrinsics.inverse();
        out.debug.stereoExtrinsicsApplied = true;
    }
    out.debug.bodyX = twc.translation().x();
    out.debug.bodyY = twc.translation().y();
    out.debug.bodyZ = twc.translation().z();

    twc = m_continuity.MapPose(mapId, trackingUsable, twc);
    out.resetCounter = m_continuity.GetResetCounter();
    out.resetMapCount = m_continuity.GetResetMapCount();

    if (!useImu && trackingUsable) {
        if (!stereoReferencePoseSet) {
            stereoReferencePose = twc;
            stereoReferencePoseSet = true;
        }
        twc = stereoReferencePose.inverse() * twc;
        out.debug.referenceApplied = true;
    }
    out.debug.localX = twc.translation().x();
    out.debug.localY = twc.translation().y();
    out.debug.localZ = twc.translation().z();

    const Eigen::Vector3f t = twc.translation();
    const Eigen::Quaternionf q(twc.so3().unit_quaternion());

    Px4MavlinkGateway::Pose pSlam{};
    pSlam.x = t.x();
    pSlam.y = t.y();
    pSlam.z = t.z();
    pSlam.qw = q.w();
    pSlam.qx = q.x();
    pSlam.qy = q.y();
    pSlam.qz = q.z();
    Px4MavlinkGateway::NormalizeQuat(pSlam.qw, pSlam.qx, pSlam.qy, pSlam.qz);

    Px4MavlinkGateway::Pose pRaw = useImu ? Px4MavlinkGateway::EnuToNed(pSlam) : pSlam;
    PoseQuality quality = PoseQuality::Lost;
    const Px4MavlinkGateway::Pose aligned = m_aligner.AlignPose(pRaw, trackingUsable, mavlink, quality);
    const auto velocity = m_velocity.Update(aligned, frameNs, quality, out.resetMapCount);

    out.alignedPose = aligned;
    out.poseEstimate.valid = true;
    out.poseEstimate.x = aligned.x;
    out.poseEstimate.y = aligned.y;
    out.poseEstimate.z = aligned.z;
    out.poseEstimate.qw = aligned.qw;
    out.poseEstimate.qx = aligned.qx;
    out.poseEstimate.qy = aligned.qy;
    out.poseEstimate.qz = aligned.qz;
    out.velocityEstimate = velocity;
    out.quality = quality;
    return out;
}

} // namespace smartdrone::core::application
