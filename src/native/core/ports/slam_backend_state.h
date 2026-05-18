#pragma once

#include <cstdint>

namespace smartdrone::core::ports {

struct SlamMapSummary {
  unsigned long mapId{0};
  int matchesInliers{0};
  uint32_t trackedMapPointCount{0};
  uint32_t localMapPointCount{0};
  uint64_t localMapPointHash{0};
  uint64_t matchedMapPointHashBeforePoseOptimization{0};
  uint64_t trackedMapPointHash{0};
};

struct SlamBackendWaitStats {
  bool requested{false};
  bool timedOut{false};
  bool acceptingBefore{false};
  bool acceptingAfter{false};
  int queueBefore{0};
  int queueAfter{0};
  int timeoutMs{0};
  double waitMs{0.0};
};

struct SlamFrameTrackingStats {
  int trackingState{0};
  double featureExtractMs{0.0};
  double stereoMatchMs{0.0};
  uint32_t featureCount{0};
  uint32_t closeMapPointCount{0};
  uint64_t frameId{0};
  int64_t referenceKeyFrameId{-1};
  int64_t lastKeyFrameId{-1};
  int64_t lastKeyFrameFrameId{-1};
  uint32_t keyFramesInMap{0};
  int stereoFeatureInitFrameId{-1};
  bool stereoFeatureInjected{false};
  bool stereoFeatureBootstrap{false};
  bool stereoFeatureStabilizing{false};
};

struct SlamBackendStats {
  SlamMapSummary map;
  SlamFrameTrackingStats frame;
  SlamBackendWaitStats localMappingWait;
};

class ISlamBackendStateProvider {
public:
  virtual ~ISlamBackendStateProvider() = default;

  virtual SlamMapSummary GetMapSummary() const = 0;
  virtual SlamBackendStats GetBackendStats() const = 0;
};

} // namespace smartdrone::core::ports
