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
    bool usedXFeatFrontend{false};
    int xfeatRawLeftCount{0};
    int xfeatRawRightCount{0};
    int xfeatMatchedStereoCount{0};
    int xfeatInjectedLeftCount{0};
    int xfeatInjectedRightCount{0};
    uint64_t xfeatSeedSourceFrameId{0};
    uint64_t xfeatSeedCurrentFrameId{0};
    uint32_t xfeatSeedAgeFrames{0};
    int xfeatSeedForwardedCount{0};
    double xfeatPrepareMs{0.0};
    double xfeatWorkerWriteMs{0.0};
    double xfeatWorkerReadMs{0.0};
    double xfeatWorkerTotalMs{0.0};
    double xfeatStereoMatchMs{0.0};
    double xfeatTotalMs{0.0};
    uint32_t xfeatImageCount{0};
    uint32_t xfeatPayloadBytes{0};
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
