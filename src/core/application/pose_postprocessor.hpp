#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

#include <sophus/se3.hpp>

#include "adapters/telemetry/px4_mavlink_gateway.hpp"
#include "common/time_utils.hpp"
#include "core/ports/pose_publisher.hpp"

namespace smartdrone::core::application {

class PosePostprocessor {
public:
    using PoseQuality = smartdrone::core::ports::PoseQuality;

    struct Result {
        Px4MavlinkGateway::Pose alignedPose{};
        smartdrone::core::ports::PoseEstimate poseEstimate{};
        smartdrone::core::ports::VelocityEstimate velocityEstimate{};
        PoseQuality quality{PoseQuality::Lost};
        uint8_t resetCounter{0};
        uint16_t resetMapCount{0};
    };

    struct ContinuityMapper {
        Sophus::SE3f MapPose(unsigned long mapId, bool trackingUsable, const Sophus::SE3f& rawPoseWc)
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

        uint8_t GetResetCounter() const { return static_cast<uint8_t>(resetCounter & 0xFFu); }
        uint16_t GetResetMapCount() const { return static_cast<uint16_t>(resetMapCount & 0xFFFFu); }

        static constexpr unsigned long kInvalidMapId = std::numeric_limits<unsigned long>::max();

        Sophus::SE3f bridgeRawToContinuous{Sophus::SE3f()};
        Sophus::SE3f lastContinuousPose{Sophus::SE3f()};
        bool bridgeInitialized{false};
        bool haveLastContinuousPose{false};
        bool haveMapId{false};
        bool pendingReset{false};
        unsigned long lastMapId{kInvalidMapId};
        uint32_t resetCounter{0};
        uint32_t resetMapCount{0};
    };

    struct StartupAligner {
        Px4MavlinkGateway::Pose AlignPose(const Px4MavlinkGateway::Pose& poseNed,
                                      bool trackingUsable,
                                      Px4MavlinkGateway& mavlink,
                                      PoseQuality& outQuality)
        {
            const uint64_t nowUs = MonoTimeUs();
            RefreshPx4LocalZ(mavlink);
            RefreshRangeSensor(mavlink);

            if (!trackingUsable) {
                Px4MavlinkGateway::Pose out = ComputeLostPose(nowUs);
                outQuality = havePublishedPose ? PoseQuality::Weak : PoseQuality::Lost;
                trackingUsablePrev = false;
                trackingReadySinceUs = 0;
                return out;
            }

            if (!trackingUsablePrev) {
                trackingReadySinceUs = nowUs;
            }
            const bool trackingRecovered = !trackingUsablePrev;
            if (!TryAlignZ(poseNed, nowUs, trackingRecovered)) {
                Px4MavlinkGateway::Pose out = havePublishedPose ? ComputeLostPose(nowUs) : poseNed;
                outQuality = havePublishedPose ? PoseQuality::Weak : PoseQuality::Lost;
                trackingUsablePrev = true;
                return out;
            }

            Px4MavlinkGateway::Pose out = poseNed;
            out.z += zOffset;

            trackingUsablePrev = true;
            lossActive = false;
            holdPose = out;
            havePublishedPose = true;
            outQuality = (!zOffsetFromPx4 || nowUs < weakUntilUs) ? PoseQuality::Weak : PoseQuality::Good;
            return out;
        }

    private:
        void RefreshPx4LocalZ(Px4MavlinkGateway& mavlink)
        {
            Px4MavlinkGateway::LocalPositionNed px4Local{};
            if (mavlink.GetLocalPositionNed(px4Local, kPx4LocalPositionMaxAgeUs)) {
                latestPx4Z = px4Local.z;
                latestPx4ZReceivedUs = px4Local.receivedUs;
                haveLatestPx4Z = true;
            }
        }

        void RefreshRangeSensor(Px4MavlinkGateway& mavlink)
        {
            Px4MavlinkGateway::DownwardDistanceSensor rng{};
            if (mavlink.GetDownwardDistanceSensor(rng, kRangeSensorMaxAgeUs)) {
                latestRange = rng;
                haveLatestRange = true;
            }
        }

        bool HasFreshPx4LocalZ(uint64_t nowUs) const
        {
            return haveLatestPx4Z && (nowUs - latestPx4ZReceivedUs) <= kPx4LocalPositionMaxAgeUs;
        }

        bool HasFreshRange() const
        {
            return haveLatestRange && std::isfinite(latestRange.currentDistance);
        }

        Px4MavlinkGateway::Pose ComputeLostPose(uint64_t nowUs)
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

        void ApplyRangeProtection(Px4MavlinkGateway::Pose&, uint64_t)
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

        bool TryAlignZ(const Px4MavlinkGateway::Pose& poseNed, uint64_t nowUs, bool trackingRecovered)
        {
            if (haveZOffset) {
                if (!zOffsetFromPx4 && HasFreshPx4LocalZ(nowUs)) {
                    zOffset = latestPx4Z - poseNed.z;
                    zOffsetFromPx4 = true;
                    weakUntilUs = nowUs + kWeakHoldUs;
                    return true;
                }
                if ((trackingRecovered || lossActive) && havePublishedPose) {
                    zOffset = holdPose.z - poseNed.z;
                    weakUntilUs = nowUs + kWeakHoldUs;
                }
                return true;
            }

            if (HasFreshPx4LocalZ(nowUs)) {
                zOffset = latestPx4Z - poseNed.z;
                haveZOffset = true;
                zOffsetFromPx4 = true;
                weakUntilUs = nowUs + kWeakHoldUs;
                return true;
            }

            if (trackingReadySinceUs != 0 && (nowUs - trackingReadySinceUs) >= kStartupFallbackAlignUs) {
                const float worldZ = havePublishedPose ? holdPose.z : 0.0f;
                zOffset = worldZ - poseNed.z;
                haveZOffset = true;
                zOffsetFromPx4 = false;
                weakUntilUs = nowUs + kWeakHoldUs;
                return true;
            }

            return false;
        }

        static constexpr uint64_t kPx4LocalPositionMaxAgeUs = 500000ULL;
        static constexpr uint64_t kRangeSensorMaxAgeUs = 200000ULL;
        static constexpr uint64_t kWeakHoldUs = 1500000ULL;
        static constexpr uint64_t kStartupFallbackAlignUs = 2000000ULL;
        static constexpr float kRangeHardFloorM = 0.35f;

        float zOffset{0.0f};
        float latestPx4Z{0.0f};
        uint64_t weakUntilUs{0};
        uint64_t latestPx4ZReceivedUs{0};
        uint64_t trackingReadySinceUs{0};
        bool haveZOffset{false};
        bool zOffsetFromPx4{false};
        bool haveLatestPx4Z{false};
        bool lossActive{false};
        bool trackingUsablePrev{false};
        bool havePublishedPose{false};
        bool haveLatestRange{false};
        Px4MavlinkGateway::Pose holdPose{};
        Px4MavlinkGateway::DownwardDistanceSensor latestRange{};
    };

    struct VelocityTracker {
        smartdrone::core::ports::VelocityEstimate Update(const Px4MavlinkGateway::Pose& pose,
                                                         int64_t frameNs,
                                                         PoseQuality quality,
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

    private:
        void ResetState(const Px4MavlinkGateway::Pose& pose,
                        int64_t frameNs,
                        PoseQuality quality,
                        uint16_t resetMapCount)
        {
            lastPose = pose;
            lastFrameNs = frameNs;
            lastQuality = quality;
            lastResetMapCount = resetMapCount;
            filteredVelocity = {};
            haveFilteredVelocity = false;
            haveLastPose = true;
        }

        static constexpr float kHorizontalTauSec = 0.18f;
        static constexpr float kVerticalTauSec = 0.30f;
        static constexpr float kMaxHorizontalSpeedMps = 8.0f;
        static constexpr float kMaxVerticalSpeedMps = 4.0f;

        Px4MavlinkGateway::Pose lastPose{};
        smartdrone::core::ports::VelocityEstimate filteredVelocity{};
        int64_t lastFrameNs{0};
        PoseQuality lastQuality{PoseQuality::Lost};
        uint16_t lastResetMapCount{0};
        bool haveLastPose{false};
        bool haveFilteredVelocity{false};
    };

    Result ProcessPose(const Sophus::SE3f& twcRaw,
                       bool useImu,
                       bool trackingUsable,
                       int trackingState,
                       unsigned long mapId,
                       bool stereoExtrinsicsLoaded,
                       const Sophus::SE3f& stereoBodyExtrinsics,
                       bool& stereoReferencePoseSet,
                       Sophus::SE3f& stereoReferencePose,
                       int64_t frameNs,
                       Px4MavlinkGateway& mavlink)
    {
        Sophus::SE3f twc = twcRaw;
        if (!useImu && stereoExtrinsicsLoaded) {
            twc = twc * stereoBodyExtrinsics.inverse();
        }
        if (!useImu && trackingUsable) {
            if (!stereoReferencePoseSet) {
                stereoReferencePose = twc;
                stereoReferencePoseSet = true;
            }
            twc = stereoReferencePose.inverse() * twc;
        }

        const Sophus::SE3f twcContinuous = m_continuity.MapPose(mapId, trackingUsable, twc);
        const Eigen::Vector3f t = twcContinuous.translation();
        const Eigen::Quaternionf q(twcContinuous.so3().unit_quaternion());

        Px4MavlinkGateway::Pose pSlam{};
        pSlam.x = t.x();
        pSlam.y = t.y();
        pSlam.z = t.z();
        pSlam.qw = q.w();
        pSlam.qx = q.x();
        pSlam.qy = q.y();
        pSlam.qz = q.z();
        Px4MavlinkGateway::NormalizeQuat(pSlam.qw, pSlam.qx, pSlam.qy, pSlam.qz);

        const Px4MavlinkGateway::Pose pRaw = useImu ? Px4MavlinkGateway::EnuToNed(pSlam) : pSlam;
        PoseQuality quality = PoseQuality::Lost;
        const Px4MavlinkGateway::Pose aligned = m_aligner.AlignPose(pRaw, trackingUsable, mavlink, quality);
        const auto velocity = m_velocity.Update(aligned, frameNs, quality, m_continuity.GetResetMapCount());

        Result out{};
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
        out.resetCounter = m_continuity.GetResetCounter();
        out.resetMapCount = m_continuity.GetResetMapCount();
        return out;
    }

private:
    ContinuityMapper m_continuity{};
    StartupAligner m_aligner{};
    VelocityTracker m_velocity{};
};

}  // namespace smartdrone::core::application
