#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "core/application/state/frame_timing_tracker.h"
#include "core/application/state/perception_pipeline.h"
#include "core/ports/imu_provider.h"
#include "core/ports/slam_engine.h"

namespace SmartDrone::Tests {

struct ReplaySlamRunnerConfig {
    int cameraFps{60};
    int slamInputFps{20};
    bool useImu{true};
    bool preferLatestFrame{true};
    int timeoutMs{1000};
    bool extractFeatures{false};
    bool extractPointCloud{false};
    bool shutdownEngineOnFinish{true};
};

struct ReplayPoseSample {
    uint64_t frameId{0};
    int64_t captureTimestampNs{0};
    int trackingState{0};
    unsigned long mapId{0};
    bool poseValid{false};
    SmartDrone::Core::Ports::PoseEstimate pose{};
    size_t imuSampleCount{0};
    bool usedVisualFeatureFrontend{false};
    int visualFeatureRawLeftCount{0};
    int visualFeatureRawRightCount{0};
    int visualFeatureMatchedStereoCount{0};
    int visualFeatureInjectedLeftCount{0};
    int visualFeatureInjectedRightCount{0};
    uint64_t visualFeatureObservationHash{0};
    int visualFeatureMatchEveryN{0};
    double visualFeatureFrontendMs{0.0};
    double visualFeatureStereoMatchMs{0.0};
    double visualFeatureTotalMs{0.0};
    double replayAcquireMs{0.0};
    double replayImuMs{0.0};
    double slamTotalMs{0.0};
    double inputPrepareMs{0.0};
    double frontendMs{0.0};
    double stereoPairMs{0.0};
    double featurePackMs{0.0};
    double monoAugmentMs{0.0};
    double lkRectifyMs{0.0};
    double lkDisparityMs{0.0};
    double lkGfttMs{0.0};
    double lkFlowMs{0.0};
    double lkCandidateMs{0.0};
    double lkPnpMs{0.0};
    double lkUpdateMs{0.0};
    double orbTrackMs{0.0};
    double orbExtractMs{0.0};
    double orbStereoMatchMs{0.0};
    double localMappingWaitMs{0.0};
    int localMappingWaitQueueBefore{0};
    int localMappingWaitQueueAfter{0};
    int localMappingWaitTimeoutMs{0};
    bool localMappingWaitRequested{false};
    bool localMappingWaitTimedOut{false};
    bool localMappingAcceptingBefore{false};
    bool localMappingAcceptingAfter{false};
    int matchesInliers{0};
    uint32_t trackedMapPointCount{0};
    uint32_t localMapPointCount{0};
    uint64_t localMapPointHash{0};
    uint64_t matchedMapPointHashBeforePoseOptimization{0};
    uint64_t trackedMapPointHash{0};
    uint32_t closeMapPointCount{0};
    uint64_t orbFrameId{0};
    int64_t referenceKeyFrameId{-1};
    int64_t lastKeyFrameId{-1};
    int64_t lastKeyFrameFrameId{-1};
    uint32_t keyFramesInMap{0};
    int stereoFeatureInitFrameId{-1};
    bool stereoFeatureInjected{false};
    bool stereoFeatureBootstrap{false};
    bool stereoFeatureStabilizing{false};
    bool realtimePoseQualityGate{false};
    float rawPoseStepMeters{0.0f};
    float gatedPoseStepMeters{0.0f};
};

using ReplayPoseSampleCallback = std::function<void(const ReplayPoseSample &)>;

class ReplaySlamRunner {
  public:
    ReplaySlamRunner(SmartDrone::Core::Ports::ICameraProvider &camera,
                     SmartDrone::Core::Ports::IImuProvider &imu,
                     SmartDrone::Core::Ports::ISlamEngine &slamEngine,
                     ReplaySlamRunnerConfig cfg);

    std::vector<ReplayPoseSample>
    Run(size_t maxFrames,
        SmartDrone::Core::Application::FrameTimingTracker *timingTracker =
            nullptr,
        const ReplayPoseSampleCallback &sampleCallback = {});

  private:
    SmartDrone::Core::Ports::ICameraProvider &m_camera;
    SmartDrone::Core::Ports::IImuProvider &m_imu;
    SmartDrone::Core::Ports::ISlamEngine &m_slamEngine;
    ReplaySlamRunnerConfig m_cfg;
    SmartDrone::Core::Application::PerceptionPipeline m_pipeline;
    int64_t m_lastFrameNs{0};
};

} // namespace SmartDrone::Tests
