#pragma once

#include <cstdint>
#include <limits>

#include <sophus/se3.hpp>

#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "core/ports/pose_publisher.h"

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
        Sophus::SE3f MapPose(unsigned long mapId, bool trackingUsable, const Sophus::SE3f &rawPoseWc);

        uint8_t GetResetCounter() const;
        uint16_t GetResetMapCount() const;

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
        Px4MavlinkGateway::Pose AlignPose(const Px4MavlinkGateway::Pose &poseNed, bool trackingUsable,
                                          Px4MavlinkGateway &mavlink, PoseQuality &outQuality);

      private:
        void RefreshRangeSensor(Px4MavlinkGateway &mavlink);
        bool HasFreshRange() const;
        Px4MavlinkGateway::Pose ComputeLostPose(uint64_t nowUs);
        void ApplyRangeProtection(Px4MavlinkGateway::Pose &pose, uint64_t nowUs);

        static constexpr uint64_t kRangeSensorMaxAgeUs = 200000ULL;
        static constexpr uint64_t kWeakHoldUs = 1500000ULL;
        static constexpr float kRangeHardFloorM = 0.35f;

        float zOffset{0.0f};
        uint64_t weakUntilUs{0};
        bool haveZOffset{false};
        bool lossActive{false};
        bool trackingUsablePrev{false};
        bool havePublishedPose{false};
        bool haveLatestRange{false};
        Px4MavlinkGateway::Pose holdPose{};
        Px4MavlinkGateway::DownwardDistanceSensor latestRange{};
    };

    struct VelocityTracker {
        smartdrone::core::ports::VelocityEstimate Update(const Px4MavlinkGateway::Pose &pose, int64_t frameNs,
                                                         PoseQuality quality, uint16_t resetMapCount);

      private:
        void ResetState(const Px4MavlinkGateway::Pose &pose, int64_t frameNs, PoseQuality quality,
                        uint16_t resetMapCount);

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

    Result ProcessPose(const Sophus::SE3f &twcRaw, bool useImu, bool trackingUsable, int trackingState,
                       unsigned long mapId, bool stereoExtrinsicsLoaded, const Sophus::SE3f &stereoBodyExtrinsics,
                       bool &stereoReferencePoseSet, Sophus::SE3f &stereoReferencePose, int64_t frameNs,
                       Px4MavlinkGateway &mavlink);

  private:
    ContinuityMapper m_continuity{};
    StartupAligner m_aligner{};
    VelocityTracker m_velocity{};
};

} // namespace smartdrone::core::application
