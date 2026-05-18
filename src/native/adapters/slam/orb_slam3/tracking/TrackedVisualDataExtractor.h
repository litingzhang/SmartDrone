#pragma once

#include <cstddef>
#include <cstdint>

#include "Frame.h"
#include "core/ports/tracked_visual_data.h"

namespace ORB_SLAM3 {

using smartdrone::core::ports::TrackedFeatureSnapshot;
using smartdrone::core::ports::TrackedPointCloudSnapshot;
using smartdrone::core::ports::TrackedVisualData;
using smartdrone::core::ports::TrackedVisualSummary;

struct TrackedVisualSummaryInput {
  size_t trackedMapPointCount{0};
  size_t localMapPointCount{0};
  int matchesInliers{0};
  uint64_t localMapPointHash{0};
  uint64_t matchedMapPointHashBeforePoseOptimization{0};
  uint64_t trackedMapPointHash{0};
  int closeMapPointCount{0};
};

TrackedVisualSummary
BuildTrackedVisualSummary(const TrackedVisualSummaryInput &input);

TrackedFeatureSnapshot ExtractTrackedFeaturesFromFrame(const Frame &frame,
                                                       int leftImageWidth,
                                                       int leftImageHeight,
                                                       int rightImageWidth,
                                                       int rightImageHeight);

TrackedPointCloudSnapshot
ExtractTrackedPointCloudFromFrame(const Frame &frame,
                                  size_t maxPointCloudPoints);

TrackedVisualData ExtractTrackedVisualDataFromFrame(
    const Frame &frame, const TrackedVisualSummaryInput &summaryInput,
    int leftImageWidth, int leftImageHeight, int rightImageWidth,
    int rightImageHeight, bool includePointCloud, size_t maxPointCloudPoints);

} // namespace ORB_SLAM3
