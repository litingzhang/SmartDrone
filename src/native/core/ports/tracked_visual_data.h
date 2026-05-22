#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core/types.hpp>

namespace SmartDrone::core::ports {

struct TrackedFeatureSnapshot {
    std::vector<cv::Point2f> leftFeatures;
    std::vector<cv::Point2f> rightFeatures;
};

struct TrackedPointCloudSnapshot {
    std::vector<float> xyz;
};

struct TrackedVisualSummary {
    int matchesInliers{0};
    uint32_t trackedMapPointCount{0};
    uint32_t localMapPointCount{0};
    uint64_t localMapPointHash{0};
    uint64_t matchedMapPointHashBeforePoseOptimization{0};
    uint64_t trackedMapPointHash{0};
    uint32_t closeMapPointCount{0};
};

struct TrackedVisualData : public TrackedVisualSummary {
    std::vector<cv::Point2f> leftFeatures;
    std::vector<cv::Point2f> rightFeatures;
    std::vector<float> pointCloudXyz;
};

struct VisualMapSnapshotRequest {
    int leftImageWidth{0};
    int leftImageHeight{0};
    int rightImageWidth{0};
    int rightImageHeight{0};
    bool includeFeatures{true};
    bool includePointCloud{false};
    size_t maxPointCloudPoints{120};
};

struct VisualMapSnapshot {
    TrackedVisualSummary summary;
    TrackedFeatureSnapshot features;
    TrackedPointCloudSnapshot pointCloud;
};

class ITrackedVisualDataProvider {
  public:
    virtual ~ITrackedVisualDataProvider() = default;

    virtual TrackedVisualSummary GetTrackedVisualSummary() const = 0;
    virtual TrackedFeatureSnapshot
    ExtractTrackedFeatures(int leftImageWidth, int leftImageHeight,
                           int rightImageWidth, int rightImageHeight) = 0;
    virtual TrackedPointCloudSnapshot
    ExtractTrackedPointCloud(size_t maxPointCloudPoints) = 0;
    virtual TrackedVisualData
    ExtractTrackedVisualData(int leftImageWidth, int leftImageHeight,
                             int rightImageWidth, int rightImageHeight,
                             bool includePointCloud,
                             size_t maxPointCloudPoints) = 0;
    virtual VisualMapSnapshot
    ExtractVisualMapSnapshot(const VisualMapSnapshotRequest &request) = 0;
};

} // namespace SmartDrone::core::ports
