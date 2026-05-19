#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "adapters/imu/icm42688_imu_provider.h"
#include "adapters/slam/slam_engine_control.h"
#include "adapters/stream/udp_image_sender.h"
#include "common/tlv/tlv_protocol.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/session/slam_settings_loader.h"
#include "core/application/session/slam_processing_support.h"
#include "core/application/state/frame_timing_tracker.h"
#include "core/application/state/live_pose_state.h"
#include "core/application/state/perception_pipeline.h"
#include "core/application/state/pose_postprocessor.h"
#include "core/ports/camera_provider.h"
#include "core/ports/pose_publisher.h"
#include "core/ports/slam_engine.h"
#include "core/ports/slam_tracking_state.h"
#include "core/ports/visual_feature_frontend.h"

namespace smartdrone::core::application {

class SlamFrameProcessor {
public:
  struct Context {
    const MainRuntimeAliases &aliases;
    bool monoMode{false};
    bool useImu{false};
    LiveRuntimeTuning &tuning;
    LivePoseState &livePose;
    PosePostprocessor::ReadRangeSensorFn readRangeSensor;
    smartdrone::core::ports::ISlamEngine &slamEngine;
    smartdrone::adapters::slam::ISlamRuntimeControl *slamControl{nullptr};
    smartdrone::core::ports::IVisualFeatureFrontend *visualFeatureFrontend{
        nullptr};
    smartdrone::core::ports::ICameraProvider &cameraProvider;
    smartdrone::adapters::imu::Icm42688ImuProvider &imuProvider;
    smartdrone::core::ports::IPosePublisher &posePublisher;
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
    unsigned long lastRawMapId{
        PosePostprocessor::ContinuityMapper::kInvalidMapId};
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
    int visualFeatureLoadSheddingLevel{0};
    int lastLoggedVisualFeatureLoadSheddingLevel{-1};
    uint8_t sessionResetCounterBase{0};
    uint16_t sessionResetMapCountBase{0};
    bool lastTrackingUsable{false};
    int lastTrackingState{smartdrone::core::ports::kSlamTrackingNoImagesYet};
    FeatureFrontend lastAppliedFeatureFrontend{FeatureFrontend::LkGfttPerFrame};
    smartdrone::core::domain::SlamOperationMode requestedSlamMode{
        smartdrone::core::domain::SlamOperationMode::Mapping};
    smartdrone::core::domain::SlamOperationMode effectiveSlamMode{
        smartdrone::core::domain::SlamOperationMode::Mapping};
  };

  enum class StepResult : uint8_t {
    Continue,
    SessionAbort,
  };

  struct PreparedFrame {
    std::chrono::steady_clock::time_point frameStartTp;
    std::chrono::steady_clock::time_point acquireStartTp;
    std::chrono::steady_clock::time_point acquireEndTp;
    std::chrono::steady_clock::time_point imuStartTp;
    std::chrono::steady_clock::time_point imuEndTp;
    StereoBatch stereoBatch;
    smartdrone::core::ports::SlamInputBatch slamInput;
    int configuredSlamInputFps{0};
    int effectiveSlamInputFps{0};
    bool sendImage{false};
    bool sendFeature{false};
    bool sendMap{false};
    int64_t pairDtMs{0};
    double rejectDtMs{0.0};
    uint64_t dropUnpairedL{0};
    uint64_t dropUnpairedR{0};
    size_t pendingL{0};
    size_t pendingR{0};
    int64_t captureTimestampNs{0};
    int64_t logicalFrameTimestampNs{0};
    double frameTime{0.0};
    double frameGapMs{0.0};
    double monoStepMs{0.0};
    double meanL{0.0};
    double stdL{0.0};
    double meanR{0.0};
    double stdR{0.0};
    double sharpL{0.0};
    double sharpR{0.0};
    bool debugRightOnlyFeatures{false};
    bool extractFeatures{false};
    bool updatePointCloud{false};
  };

  struct TrackedFrame {
    std::shared_ptr<PreparedFrame> frame;
    smartdrone::core::ports::SlamOutput slamOutput;
    std::chrono::steady_clock::time_point slamStartTp;
    std::chrono::steady_clock::time_point slamEndTp;
  };

  struct PublishedFrame {
    std::shared_ptr<TrackedFrame> frame;
    PosePostprocessor::Result poseResult;
    std::chrono::steady_clock::time_point cloudStartTp;
    std::chrono::steady_clock::time_point cloudEndTp;
    std::chrono::steady_clock::time_point udpStartTp;
    std::chrono::steady_clock::time_point udpEndTp;
    std::chrono::steady_clock::time_point postStartTp;
    std::chrono::steady_clock::time_point postEndTp;
    std::chrono::steady_clock::time_point livePoseStartTp;
    std::chrono::steady_clock::time_point livePoseEndTp;
    std::chrono::steady_clock::time_point publishStartTp;
    std::chrono::steady_clock::time_point publishEndTp;
    size_t pointCount{0};
    int trackingState{0};
    bool trackingUsable{false};
    uint8_t effectiveResetCounter{0};
    uint16_t effectiveResetMapCount{0};
  };

  SlamFrameProcessor(Context &context, State &state);

  StepResult StepBackend();
  StepResult AcquireAndPrepareFrame(bool &sessionOk, PreparedFrame &frame);
  StepResult TrackPreparedFrame(std::shared_ptr<PreparedFrame> frame,
                                TrackedFrame &tracked);
  StepResult PostprocessTrackedFrame(std::shared_ptr<TrackedFrame> tracked,
                                     PublishedFrame &published);
  StepResult PublishTrackedFrame(std::shared_ptr<TrackedFrame> tracked,
                                 PublishedFrame &published);
  StepResult EmitPointCloud(PublishedFrame &published);
  StepResult EmitLivePose(PublishedFrame &published);
  StepResult EmitMavlink(PublishedFrame &published);
  StepResult EmitUdp(PublishedFrame &published);
  StepResult EmitDfx(PublishedFrame &published);
  StepResult PublishTrackedFrame(TrackedFrame &tracked, bool &sessionOk);
  StepResult ProcessPreparedFrame(PreparedFrame &frame, bool &sessionOk);
  StepResult ProcessNextFrame(bool &sessionOk);

private:
  struct TrackingContext {
    int state{smartdrone::core::ports::kSlamTrackingNoImagesYet};
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
    int trackingState{smartdrone::core::ports::kSlamTrackingNoImagesYet};
    bool trackingUsable{false};
    uint8_t effectiveResetCounter{0};
    uint16_t effectiveResetMapCount{0};
    size_t pointCount{0};
  };

  struct RuntimeFrameConfig {
    int configuredSlamInputFps{0};
    int effectiveSlamInputFps{0};
    uint8_t configuredFrontendValue{0};
    FeatureFrontend effectiveFrontend{FeatureFrontend::LkGfttPerFrame};
    bool dpvoEpgPacing{false};
    int visualFeatureLoadSheddingLevel{0};
    int visualFeatureBudgetWidth{0};
    int visualFeatureBudgetHeight{0};
  };

  struct StereoAcquireResult {
    StepResult stepResult{StepResult::Continue};
    bool hasFrame{false};
    std::chrono::steady_clock::time_point acquireStartTp;
    std::chrono::steady_clock::time_point acquireEndTp;
    StereoBatch stereoBatch;
  };

  struct FrameMetadata {
    bool sendImage{false};
    bool sendFeature{false};
    bool sendMap{false};
    int64_t pairDtMs{0};
    double rejectDtMs{0.0};
    uint64_t dropUnpairedL{0};
    uint64_t dropUnpairedR{0};
    size_t pendingL{0};
    size_t pendingR{0};
    int64_t captureTimestampNs{0};
    int64_t logicalFrameTimestampNs{0};
    double frameTime{0.0};
    double frameGapMs{0.0};
    double monoStepMs{0.0};
    StereoFrameQuality stereoQuality{};
    bool debugRightOnlyFeatures{false};
    bool extractFeatures{false};
    bool updatePointCloud{false};
  };

  struct SlamInputPreparation {
    StepResult stepResult{StepResult::Continue};
    bool ready{false};
    std::chrono::steady_clock::time_point imuStartTp;
    std::chrono::steady_clock::time_point imuEndTp;
    smartdrone::core::ports::SlamInputBatch slamInput;
  };

  struct DfxTiming {
    double acquireMs{0.0};
    double imuMs{0.0};
    double slamMs{0.0};
    double cloudMs{0.0};
    double udpMs{0.0};
    double postMs{0.0};
    double livePoseMs{0.0};
    double publishMs{0.0};
    double totalMs{0.0};
  };

  struct DfxSample {
    PreparedFrame &frame;
    TrackedFrame &tracked;
    PublishedFrame &published;
    smartdrone::core::ports::SlamInputBatch &slamInput;
    smartdrone::core::ports::SlamOutput &slamOutput;
    const PosePostprocessor::Result &poseResult;
    DfxTiming timing;
    bool visualFeatureStereoWeak{false};
  };

  DfxSample MakeDfxSample(PublishedFrame &published) const;
  DfxTiming MakeDfxTiming(const PreparedFrame &frame, const TrackedFrame &tracked,
                          const PublishedFrame &published) const;
  bool ShouldLogDfx(const DfxSample &sample) const;
  void LogDfxLine(const DfxSample &sample) const;
  void UpdateDfxAverages(const DfxSample &sample);
  std::string BuildJsonDfxLine(const DfxSample &sample) const;
  std::string BuildTextDfxLine(const DfxSample &sample) const;
  void AppendDfxJsonCore(std::ostream &out, const DfxSample &sample) const;
  void AppendDfxJsonBackend(std::ostream &out, const DfxSample &sample) const;
  void AppendDfxJsonVisual(std::ostream &out, const DfxSample &sample) const;
  void AppendDfxJsonFrameStats(std::ostream &out, const DfxSample &sample) const;
  void AppendDfxJsonTiming(std::ostream &out, const DfxSample &sample) const;
  void AppendDfxTextCore(std::ostream &out, const DfxSample &sample) const;
  void AppendDfxTextBackend(std::ostream &out, const DfxSample &sample) const;
  void AppendDfxTextVisual(std::ostream &out, const DfxSample &sample) const;
  void AppendDfxTextFrameStats(std::ostream &out, const DfxSample &sample) const;
  void AppendDfxTextTiming(std::ostream &out, const DfxSample &sample) const;
  void SyncRequestedSlamMode();
  RuntimeFrameConfig ApplyRuntimeFrameConfig();
  void ApplySlamControlConfig(const RuntimeFrameConfig &config);
  void LogFrontendChange(const RuntimeFrameConfig &config);
  void LogInputFpsChange(const RuntimeFrameConfig &config);
  void LogVisualFeatureProfile(const RuntimeFrameConfig &config);
  StereoAcquireResult AcquireStereoBatch(const RuntimeFrameConfig &config,
                                         bool &sessionOk);
  FrameMetadata BuildFrameMetadata(const StereoBatch &stereoBatch,
                                   const RuntimeFrameConfig &config);
  void MaybeLogFrameGap(const StereoBatch &stereoBatch,
                        const RuntimeFrameConfig &config,
                        const FrameMetadata &metadata);
  SlamInputPreparation PrepareSlamInput(const StereoBatch &stereoBatch,
                                        const FrameMetadata &metadata);
  void FillPreparedFrame(PreparedFrame &frame, StereoAcquireResult &&acquire,
                         SlamInputPreparation &&input,
                         const RuntimeFrameConfig &config,
                         const FrameMetadata &metadata) const;
  TrackingContext ResolveTrackingContext(const TrackedFrame &tracked);
  Sophus::SE3f ResolveRawPose(const TrackedFrame &tracked,
                              const TrackingContext &tracking);
  StereoExtrinsicsContext ResolveStereoExtrinsics() const;
  PosePostprocessor::ProcessRequest
  BuildPoseRequest(const Sophus::SE3f &twcRaw,
                   const TrackingContext &tracking,
                   int64_t captureTimestampNs,
                   const StereoExtrinsicsContext &extrinsics) const;
  void MaybeLogPoseAxis(const TrackedFrame &tracked,
                        const TrackingContext &tracking,
                        const Sophus::SE3f &twcRaw,
                        const PosePostprocessor::Result &poseResult) const;
  void UpdateAutoSlamMode(const TrackedFrame &tracked,
                          const TrackingContext &tracking,
                          const PosePostprocessor::Result &poseResult,
                          double frameGapMs);
  PostprocessArtifacts
  BuildPostprocessArtifacts(const TrackedFrame &tracked,
                            const TrackingContext &tracking,
                            const PosePostprocessor::Result &poseResult,
                            std::chrono::steady_clock::time_point postStartTp,
                            std::chrono::steady_clock::time_point postEndTp)
      const;
  void FillPublishedFrame(std::shared_ptr<TrackedFrame> tracked,
                          const PostprocessArtifacts &artifacts,
                          PublishedFrame &published) const;

  Context &m_ctx;
  State &m_state;
};

} // namespace smartdrone::core::application
