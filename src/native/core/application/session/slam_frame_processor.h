#pragma once

#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "adapters/imu/icm42688_imu_provider.h"
#include "adapters/slam/slam_engine_adapter.h"
#include "adapters/slam/superpoint_lightglue_frontend_client.h"
#include "adapters/stream/udp_image_sender.h"
#include "adapters/telemetry/mavlink_pose_publisher.h"
#include "adapters/telemetry/px4_mavlink_gateway.h"
#include "common/tlv/tlv_protocol.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/session/runtime_session_common.h"
#include "core/application/session/slam_processing_support.h"
#include "core/application/state/frame_timing_tracker.h"
#include "core/application/state/live_pose_state.h"
#include "core/application/state/perception_pipeline.h"
#include "core/application/state/pose_postprocessor.h"
#include "core/ports/camera_provider.h"

namespace smartdrone::core::application {

class SlamFrameProcessor {
  public:
    struct Context {
        const MainRuntimeAliases &aliases;
        bool monoMode{false};
        bool useImu{false};
        LiveRuntimeTuning &tuning;
        LivePoseState &livePose;
        Px4MavlinkGateway &mav;
        smartdrone::adapters::slam::SlamEngineAdapter &slamEngine;
        smartdrone::adapters::slam::SuperPointLightGlueFrontendClient *superpointFrontendClient{nullptr};
        smartdrone::core::ports::ICameraProvider &cameraProvider;
        smartdrone::adapters::imu::Icm42688ImuProvider &imuProvider;
        smartdrone::adapters::telemetry::MavlinkPosePublisher &posePublisher;
        UdpImageSender &udpSender;
        FrameTimingTracker &frameTimingTracker;
        PerceptionPipeline &perceptionPipeline;
        PosePostprocessor &posePostprocessor;
        AutoSlamModeController &autoSlamModeController;
        const StereoBodyExtrinsics &stereoBodyExtrinsics;
    };

    struct State {
        int64_t lastFrameNs{0};
        int lastLoggedConfiguredSlamInputFps{-1};
        int lastLoggedEffectiveSlamInputFps{-1};
        uint64_t imuWarmupSamples{0};
        int64_t lastPointCloudUpdateNs{0};
        Sophus::SE3f stereoReferencePose{Sophus::SE3f()};
        bool stereoReferencePoseSet{false};
        unsigned long lastRawMapId{PosePostprocessor::ContinuityMapper::kInvalidMapId};
        Sophus::SE3f lastValidTwcRaw{Sophus::SE3f()};
        bool haveLastValidTwcRaw{false};
        uint64_t frameIndex{0};
        int64_t lastPublishedFrameNs{0};
        int64_t lastFrameGapWarnLogNs{0};
        uint64_t rateLimitedDrops{0};
        double smoothedAcquireMs{0.0};
        double smoothedSlamMs{0.0};
        double smoothedTotalMs{0.0};
        int adaptiveSlamInputFps{0};
        int superpointLoadSheddingLevel{0};
        int lastLoggedSuperPointLoadSheddingLevel{-1};
        uint8_t sessionResetCounterBase{0};
        uint16_t sessionResetMapCountBase{0};
        bool lastTrackingUsable{false};
        int lastTrackingState{ORB_SLAM3::Tracking::NO_IMAGES_YET};
        FeatureFrontend lastAppliedFeatureFrontend{FeatureFrontend::Orb};
        smartdrone::core::domain::SlamOperationMode requestedSlamMode{
            smartdrone::core::domain::SlamOperationMode::Mapping};
        smartdrone::core::domain::SlamOperationMode effectiveSlamMode{
            smartdrone::core::domain::SlamOperationMode::Mapping};
    };

    enum class StepResult : uint8_t {
        Continue,
        SessionAbort,
    };

    SlamFrameProcessor(Context &context, State &state);

    StepResult ProcessNextFrame(bool &sessionOk);

  private:
    Context &m_ctx;
    State &m_state;
};

} // namespace smartdrone::core::application
