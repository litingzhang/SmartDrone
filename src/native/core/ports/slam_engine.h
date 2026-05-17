#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core/types.hpp>

#include "camera_provider.h"
#include "imu_provider.h"
#include "slam_tracking_state.h"

namespace smartdrone::core::ports {

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

struct SlamOutput {
    uint64_t frameId{0};
    int64_t captureTimestampNs{0};
    PoseEstimate pose;
    bool poseValid{false};
    int trackingState{kSlamTrackingNoImagesYet};
    unsigned long mapId{0};
    bool usedSuperPointFrontend{false};
    int superpointRawLeftCount{0};
    int superpointRawRightCount{0};
    int superpointMatchedStereoCount{0};
    int superpointInjectedLeftCount{0};
    int superpointInjectedRightCount{0};
    uint64_t superpointExternalHash{0};
    int superpointLightGlueEveryN{0};
    double superpointPrepareMs{0.0};
    double superpointInputMs{0.0};
    double superpointForwardMs{0.0};
    double superpointFrontendMs{0.0};
    double superpointStereoMatchMs{0.0};
    double superpointTotalMs{0.0};
    uint32_t superpointImageCount{0};
    uint32_t superpointPayloadBytes{0};
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
    int externalStereoInitFrameId{-1};
    bool externalStereoInjected{false};
    bool externalStereoBootstrap{false};
    bool externalStereoStabilizing{false};
    bool realtimePoseQualityGate{false};
    float rawPoseStepMeters{0.0f};
    float gatedPoseStepMeters{0.0f};
    std::vector<cv::Point2f> leftFeatures;
    std::vector<cv::Point2f> rightFeatures;
    std::vector<float> pointCloudXyz;
};

class ISlamEngine {
  public:
    virtual ~ISlamEngine() = default;

    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual SlamOutput Process(const SlamInputBatch &input, bool extractFeatures, bool extractPointCloud) = 0;
};

} // namespace smartdrone::core::ports
