#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "core/application/session/slam/slam_frame_stage_context.h"
#include "core/application/session/slam/slam_frame_stage_data.h"
#include "core/application/session/slam/slam_frame_stage_state.h"
#include "core/application/session/slam/slam_frame_step_result.h"

namespace SmartDrone::core::application {

class SlamFramePosePostprocessPort final {
  public:
    SlamFramePosePostprocessPort(SlamFramePosePostprocessContext &context,
                                 SlamFramePosePostprocessState &state,
                                 SlamFrameSharedState &sharedState);

    SlamFrameStepResult PostprocessTrackedFrame(
        std::shared_ptr<SlamTrackedFrameData> tracked,
        SlamPublishedFrameData &published);

  private:
    struct TrackingContext {
        int state{SmartDrone::core::ports::kSlamTrackingNoImagesYet};
        bool usable{false};
        unsigned long mapId{PosePostprocessor::ContinuityMapper::kInvalidMapId};
    };

    struct StereoExtrinsicsContext {
        bool loaded{false};
        Sophus::SE3f bodyToCamera{Sophus::SE3f()};
    };

    struct PostprocessArtifacts {
        PosePostprocessor::Result poseResult{};
        std::chrono::steady_clock::time_point postStartTp;
        std::chrono::steady_clock::time_point postEndTp;
        int trackingState{SmartDrone::core::ports::kSlamTrackingNoImagesYet};
        bool trackingUsable{false};
        uint8_t effectiveResetCounter{0};
        uint16_t effectiveResetMapCount{0};
        size_t pointCount{0};
    };

    TrackingContext ResolveTrackingContext(const SlamTrackedFrameData &tracked);
    Sophus::SE3f ResolveRawPose(const SlamTrackedFrameData &tracked,
                                const TrackingContext &tracking);
    StereoExtrinsicsContext ResolveStereoExtrinsics() const;
    PosePostprocessor::ProcessRequest
    BuildPoseRequest(const Sophus::SE3f &twcRaw,
                     const TrackingContext &tracking,
                     int64_t captureTimestampNs,
                     const StereoExtrinsicsContext &extrinsics) const;
    void MaybeLogPoseAxis(const SlamTrackedFrameData &tracked,
                          const TrackingContext &tracking,
                          const Sophus::SE3f &twcRaw,
                          const PosePostprocessor::Result &poseResult) const;
    void UpdateAutoSlamMode(const SlamTrackedFrameData &tracked,
                            const TrackingContext &tracking,
                            const PosePostprocessor::Result &poseResult,
                            double frameGapMs);
    PostprocessArtifacts
    BuildPostprocessArtifacts(const SlamTrackedFrameData &tracked,
                              const TrackingContext &tracking,
                              const PosePostprocessor::Result &poseResult,
                              std::chrono::steady_clock::time_point postStartTp,
                              std::chrono::steady_clock::time_point postEndTp)
        const;
    void FillPublishedFrame(std::shared_ptr<SlamTrackedFrameData> tracked,
                            const PostprocessArtifacts &artifacts,
                            SlamPublishedFrameData &published) const;

    SlamFramePosePostprocessContext &m_ctx;
    SlamFramePosePostprocessState &m_state;
    SlamFrameSharedState &m_sharedState;
};

} // namespace SmartDrone::core::application
