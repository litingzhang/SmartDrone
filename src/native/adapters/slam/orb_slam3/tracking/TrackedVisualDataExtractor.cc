#include "TrackedVisualDataExtractor.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "MapPoint.h"

namespace ORB_SLAM3 {

TrackedVisualSummary
BuildTrackedVisualSummary(const TrackedVisualSummaryInput &input) {
  TrackedVisualSummary out;
  out.trackedMapPointCount = static_cast<uint32_t>(input.trackedMapPointCount);
  out.localMapPointCount = static_cast<uint32_t>(input.localMapPointCount);
  out.matchesInliers = input.matchesInliers;
  out.localMapPointHash = input.localMapPointHash;
  out.matchedMapPointHashBeforePoseOptimization =
      input.matchedMapPointHashBeforePoseOptimization;
  out.trackedMapPointHash = input.trackedMapPointHash;
  out.closeMapPointCount =
      static_cast<uint32_t>(std::max(0, input.closeMapPointCount));
  return out;
}

TrackedFeatureSnapshot ExtractTrackedFeaturesFromFrame(const Frame &frame,
                                                       int leftImageWidth,
                                                       int leftImageHeight,
                                                       int rightImageWidth,
                                                       int rightImageHeight) {
  TrackedFeatureSnapshot out;
  const size_t leftCount =
      std::min(frame.mvpMapPoints.size(), frame.mvKeysUn.size());
  const size_t rightCount = std::min(leftCount, frame.mvuRight.size());
  out.leftFeatures.reserve(leftCount);
  out.rightFeatures.reserve(rightCount);

  for (size_t i = 0; i < leftCount; ++i) {
    MapPoint *point = frame.mvpMapPoints[i];
    if (!point || point->isBad()) {
      continue;
    }

    const cv::Point2f &leftPt = frame.mvKeysUn[i].pt;
    if (leftPt.x >= 0.0f && leftPt.y >= 0.0f &&
        leftPt.x < static_cast<float>(leftImageWidth) &&
        leftPt.y < static_cast<float>(leftImageHeight)) {
      out.leftFeatures.push_back(leftPt);
    }

    if (i < rightCount) {
      const float rightX = frame.mvuRight[i];
      const float rightY = leftPt.y;
      if (rightX >= 0.0f && rightY >= 0.0f &&
          rightX < static_cast<float>(rightImageWidth) &&
          rightY < static_cast<float>(rightImageHeight)) {
        out.rightFeatures.emplace_back(rightX, rightY);
      }
    }
  }

  return out;
}

TrackedPointCloudSnapshot
ExtractTrackedPointCloudFromFrame(const Frame &frame,
                                  size_t maxPointCloudPoints) {
  TrackedPointCloudSnapshot out;
  const size_t cappedCloudPoints = std::max<size_t>(1, maxPointCloudPoints);
  const size_t cloudStride =
      std::max<size_t>(1, frame.mvpMapPoints.size() / cappedCloudPoints);
  out.xyz.reserve(cappedCloudPoints * 3);

  size_t cloudCount = 0;
  for (size_t i = 0;
       i < frame.mvpMapPoints.size() && cloudCount < cappedCloudPoints; ++i) {
    if (cloudStride != 1 && (i % cloudStride) != 0) {
      continue;
    }
    MapPoint *point = frame.mvpMapPoints[i];
    if (!point || point->isBad()) {
      continue;
    }
    const Eigen::Vector3f world = point->GetWorldPos();
    if (std::isfinite(world.x()) && std::isfinite(world.y()) &&
        std::isfinite(world.z())) {
      out.xyz.push_back(world.x());
      out.xyz.push_back(world.y());
      out.xyz.push_back(world.z());
      ++cloudCount;
    }
  }

  return out;
}

TrackedVisualData ExtractTrackedVisualDataFromFrame(
    const Frame &frame, const TrackedVisualSummaryInput &summaryInput,
    int leftImageWidth, int leftImageHeight, int rightImageWidth,
    int rightImageHeight, bool includePointCloud, size_t maxPointCloudPoints) {
  TrackedVisualData out;
  const TrackedVisualSummary summary = BuildTrackedVisualSummary(summaryInput);
  out.matchesInliers = summary.matchesInliers;
  out.trackedMapPointCount = summary.trackedMapPointCount;
  out.localMapPointCount = summary.localMapPointCount;
  out.localMapPointHash = summary.localMapPointHash;
  out.matchedMapPointHashBeforePoseOptimization =
      summary.matchedMapPointHashBeforePoseOptimization;
  out.trackedMapPointHash = summary.trackedMapPointHash;
  out.closeMapPointCount = summary.closeMapPointCount;

  TrackedFeatureSnapshot features =
      ExtractTrackedFeaturesFromFrame(frame, leftImageWidth, leftImageHeight,
                                      rightImageWidth, rightImageHeight);
  out.leftFeatures = std::move(features.leftFeatures);
  out.rightFeatures = std::move(features.rightFeatures);
  if (includePointCloud) {
    out.pointCloudXyz =
        ExtractTrackedPointCloudFromFrame(frame, maxPointCloudPoints).xyz;
  }
  return out;
}

} // namespace ORB_SLAM3
