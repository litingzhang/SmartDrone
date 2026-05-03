#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core/types.hpp>

#include "ImuTypes.h"
#include "camera_provider.h"

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
    std::vector<ORB_SLAM3::IMU::Point> imu;
};

struct SlamOutput {
    uint64_t frameId{0};
    int64_t captureTimestampNs{0};
    PoseEstimate pose;
    bool poseValid{false};
    int trackingState{0};
    unsigned long mapId{0};
    bool usedSuperPointFrontend{false};
    int superpointRawLeftCount{0};
    int superpointRawRightCount{0};
    int superpointMatchedStereoCount{0};
    int superpointInjectedLeftCount{0};
    int superpointInjectedRightCount{0};
    uint64_t superpointSeedSourceFrameId{0};
    uint64_t superpointSeedCurrentFrameId{0};
    uint32_t superpointSeedAgeFrames{0};
    int superpointSeedForwardedCount{0};
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
    int matchesInliers{0};
    uint32_t trackedMapPointCount{0};
    uint32_t localMapPointCount{0};
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
