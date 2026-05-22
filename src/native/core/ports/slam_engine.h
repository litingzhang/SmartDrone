#pragma once

#include <cstdint>
#include <vector>

#include "camera_provider.h"
#include "imu_provider.h"
#include "slam_tracking_state.h"
#include "tracked_visual_data.h"

namespace SmartDrone::Core::Ports {

struct PoseEstimate {
    bool valid{false};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float qw{1.0f};
    float qx{0.0f};
    float qy{0.0f};
    float qz{0.0f};
};

struct SlamInputBatch {
    StereoFrame stereo;
    uint64_t frameId{0};
    int64_t captureTimestampNs{0};
    double frameTimeSec{0.0};
    std::vector<ImuReading> imu;
};

struct SlamOutput : public TrackedVisualData {
    uint64_t frameId{0};
    int64_t captureTimestampNs{0};
    PoseEstimate pose;
    bool poseValid{false};
    int trackingState{kSlamTrackingNoImagesYet};
    unsigned long mapId{0};
    bool usedVisualFeatureFrontend{false};
    int visualFeatureRawLeftCount{0};
    int visualFeatureRawRightCount{0};
    int visualFeatureMatchedStereoCount{0};
    int visualFeatureInjectedLeftCount{0};
    int visualFeatureInjectedRightCount{0};
    uint64_t visualFeatureObservationHash{0};
    int visualFeatureMatchEveryN{0};
    double visualFeaturePrepareMs{0.0};
    double visualFeatureInputMs{0.0};
    double visualFeatureForwardMs{0.0};
    double visualFeatureFrontendMs{0.0};
    double visualFeatureStereoMatchMs{0.0};
    double visualFeatureTotalMs{0.0};
    uint32_t visualFeatureImageCount{0};
    uint32_t visualFeaturePayloadBytes{0};
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

class ISlamEngine {
  public:
    virtual ~ISlamEngine() = default;

    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual SlamOutput Process(const SlamInputBatch &input, bool extractFeatures,
                               bool extractPointCloud) = 0;
};

} // namespace SmartDrone::Core::Ports
