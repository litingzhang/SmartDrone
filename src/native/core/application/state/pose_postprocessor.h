#pragma once

#include <cstdint>
#include <functional>
#include <limits>

#include <sophus/se3.hpp>

#include "core/ports/pose_publisher.h"

namespace smartdrone::core::application {

class PosePostprocessor {
  public:
    using PoseQuality = smartdrone::core::ports::PoseQuality;

    struct RangeSensorSnapshot {
        float currentDistance{std::numeric_limits<float>::quiet_NaN()};
        uint8_t signalQuality{0};
    };

    using ReadRangeSensorFn = std::function<bool(RangeSensorSnapshot &)>;

    struct Result {
        struct DebugPose {
            float cameraX{0.0f};
            float cameraY{0.0f};
            float cameraZ{0.0f};
            float bodyX{0.0f};
            float bodyY{0.0f};
            float bodyZ{0.0f};
            float localX{0.0f};
            float localY{0.0f};
            float localZ{0.0f};
            float outputGuardRawStepM{0.0f};
            float outputGuardMaxStepM{0.0f};
            bool stereoExtrinsicsApplied{false};
            bool referenceApplied{false};
            bool outputGuardApplied{false};
        };

        smartdrone::core::ports::PoseEstimate poseEstimate{};
        smartdrone::core::ports::VelocityEstimate velocityEstimate{};
        PoseQuality quality{PoseQuality::Lost};
        uint8_t resetCounter{0};
        uint16_t resetMapCount{0};
        DebugPose debug{};
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
        smartdrone::core::ports::PoseEstimate AlignPose(const smartdrone::core::ports::PoseEstimate &poseNed,
                                                        bool trackingUsable, const ReadRangeSensorFn &readRangeSensor,
                                                        PoseQuality &outQuality);
        void SetPublishedPose(const smartdrone::core::ports::PoseEstimate &pose);

      private:
        void RefreshRangeSensor(const ReadRangeSensorFn &readRangeSensor);
        bool HasFreshRange() const;
        smartdrone::core::ports::PoseEstimate ComputeLostPose(uint64_t nowUs);
        void ApplyRangeProtection(smartdrone::core::ports::PoseEstimate &pose, uint64_t nowUs);

        static constexpr uint64_t kWeakHoldUs = 1500000ULL;
        static constexpr float kRangeHardFloorM = 0.35f;

        float zOffset{0.0f};
        uint64_t weakUntilUs{0};
        bool haveZOffset{false};
        bool lossActive{false};
        bool trackingUsablePrev{false};
        bool havePublishedPose{false};
        bool haveLatestRange{false};
        smartdrone::core::ports::PoseEstimate holdPose{};
        RangeSensorSnapshot latestRange{};
    };

    struct OutputGuard {
        struct GuardRequest {
            smartdrone::core::ports::PoseEstimate pose{};
            int64_t frameNs{0};
            bool trackingUsable{false};
            PoseQuality quality{PoseQuality::Lost};
        };

        struct GuardResult {
            smartdrone::core::ports::PoseEstimate pose{};
            PoseQuality quality{PoseQuality::Lost};
            bool guardApplied{false};
            float rawStepM{0.0f};
            float maxStepM{0.0f};
        };

        GuardResult GuardPose(const GuardRequest &request);

      private:
        float ComputeAllowedStep(int64_t frameNs) const;
        float ComputeAllowedRotation(int64_t frameNs) const;
        GuardResult BuildResult(const smartdrone::core::ports::PoseEstimate &pose, PoseQuality quality,
                                bool guardApplied, float rawStepM, float maxStepM) const;
        GuardResult HandleInvalidPose(const GuardRequest &request);
        GuardResult ClampPose(const GuardRequest &request, float rawStepM, float maxStepM, float rawRotRad,
                              float maxRotRad);
        void MaybeLogClamp(int64_t frameNs, float rawStepM, float maxStepM, float rawRotRad, float maxRotRad);
        void CommitPose(const smartdrone::core::ports::PoseEstimate &pose, int64_t frameNs);

        smartdrone::core::ports::PoseEstimate lastPose{};
        int64_t lastFrameNs{0};
        int64_t lastLogFrameNs{0};
        uint64_t guardHitCount{0};
        bool haveLastPose{false};
    };

    struct VelocityTracker {
        smartdrone::core::ports::VelocityEstimate Update(const smartdrone::core::ports::PoseEstimate &pose,
                                                         int64_t frameNs, PoseQuality quality,
                                                         uint16_t resetMapCount);

      private:
        struct RawVelocity {
            float vx{0.0f};
            float vy{0.0f};
            float vz{0.0f};
        };

        bool ShouldReset(int64_t frameNs, PoseQuality quality, uint16_t resetMapCount) const;
        RawVelocity ComputeRawVelocity(const smartdrone::core::ports::PoseEstimate &pose, float dt) const;
        bool RawVelocityUsable(const RawVelocity &velocity) const;
        void UpdateFilteredVelocity(const RawVelocity &velocity, float dt);
        void CommitPoseState(const smartdrone::core::ports::PoseEstimate &pose, int64_t frameNs, PoseQuality quality,
                             uint16_t resetMapCount);
        void ResetState(const smartdrone::core::ports::PoseEstimate &pose, int64_t frameNs, PoseQuality quality,
                        uint16_t resetMapCount);

        static constexpr float kHorizontalTauSec = 0.18f;
        static constexpr float kVerticalTauSec = 0.30f;
        static constexpr float kMaxHorizontalSpeedMps = 8.0f;
        static constexpr float kMaxVerticalSpeedMps = 4.0f;

        smartdrone::core::ports::PoseEstimate lastPose{};
        smartdrone::core::ports::VelocityEstimate filteredVelocity{};
        int64_t lastFrameNs{0};
        PoseQuality lastQuality{PoseQuality::Lost};
        uint16_t lastResetMapCount{0};
        bool haveLastPose{false};
        bool haveFilteredVelocity{false};
    };

    struct ProcessRequest {
        Sophus::SE3f twcRaw{Sophus::SE3f()};
        bool useImu{false};
        bool trackingUsable{false};
        unsigned long mapId{ContinuityMapper::kInvalidMapId};
        bool stereoExtrinsicsLoaded{false};
        Sophus::SE3f stereoBodyExtrinsics{Sophus::SE3f()};
        bool *stereoReferencePoseSet{nullptr};
        Sophus::SE3f *stereoReferencePose{nullptr};
        int64_t frameNs{0};
        ReadRangeSensorFn readRangeSensor;
    };

    Result ProcessPose(const ProcessRequest &request);

  private:
    struct PreparedPose {
        Sophus::SE3f twc{Sophus::SE3f()};
        Result result{};
    };

    PreparedPose PreparePose(const ProcessRequest &request);
    void ApplyStereoReference(const ProcessRequest &request, PreparedPose &prepared);
    void PopulateOutputPose(const ProcessRequest &request, PreparedPose &prepared);

    ContinuityMapper m_continuity{};
    StartupAligner m_aligner{};
    OutputGuard m_outputGuard{};
    VelocityTracker m_velocity{};
};

} // namespace smartdrone::core::application
