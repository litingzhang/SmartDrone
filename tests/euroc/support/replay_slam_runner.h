#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "ImuTypes.h"
#include "core/application/state/frame_timing_tracker.h"
#include "core/application/state/perception_pipeline.h"
#include "core/ports/imu_provider.h"
#include "core/ports/slam_engine.h"

namespace smartdrone::tests {

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
    smartdrone::core::ports::PoseEstimate pose{};
    size_t imuSampleCount{0};
    bool usedSuperPointFrontend{false};
    int superpointRawLeftCount{0};
    int superpointRawRightCount{0};
    int superpointMatchedStereoCount{0};
    int superpointInjectedLeftCount{0};
    int superpointInjectedRightCount{0};
    double superpointFrontendMs{0.0};
    double superpointStereoMatchMs{0.0};
    double superpointTotalMs{0.0};
    double replayAcquireMs{0.0};
    double replayImuMs{0.0};
    double slamTotalMs{0.0};
    double inputPrepareMs{0.0};
    double frontendMs{0.0};
    double stereoPairMs{0.0};
    double externalPackMs{0.0};
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
    int matchesInliers{0};
    uint32_t trackedMapPointCount{0};
    uint32_t localMapPointCount{0};
};

class ReplaySlamRunner {
  public:
    ReplaySlamRunner(smartdrone::core::ports::ICameraProvider &camera, smartdrone::core::ports::IImuProvider &imu,
                     smartdrone::core::ports::ISlamEngine &slamEngine, ReplaySlamRunnerConfig cfg);

    std::vector<ReplayPoseSample> Run(size_t maxFrames,
                                      smartdrone::core::application::FrameTimingTracker *timingTracker = nullptr);

  private:
    static std::vector<ORB_SLAM3::IMU::Point> ToOrbImuPoints(
        const std::vector<smartdrone::core::ports::ImuReading> &readings);

    smartdrone::core::ports::ICameraProvider &m_camera;
    smartdrone::core::ports::IImuProvider &m_imu;
    smartdrone::core::ports::ISlamEngine &m_slamEngine;
    ReplaySlamRunnerConfig m_cfg;
    smartdrone::core::application::PerceptionPipeline m_pipeline;
    int64_t m_lastFrameNs{0};
};

} // namespace smartdrone::tests
