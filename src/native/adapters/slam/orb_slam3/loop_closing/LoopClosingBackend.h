#ifndef ORB_LOOP_CLOSING_BACKEND_H
#define ORB_LOOP_CLOSING_BACKEND_H

#include <memory>
#include <vector>

namespace ORB_SLAM3 {

class KeyFrame;
class LoopClosing;
class Map;
class IOrbTrackingBackend;

struct OrbLoopClosingKeyFrameRequest {
  KeyFrame *keyframe{nullptr};
};

struct OrbLoopClosingResetRequest {
  Map *map{nullptr};
  bool activeMapOnly{false};
};

struct OrbLoopClosingRuntimePeers {
  IOrbTrackingBackend *tracking{nullptr};
};

struct OrbLoopClosingStatus {
  bool finished{false};
  bool runningGlobalBundleAdjustment{false};
};

struct OrbLoopClosingTimingStats {
  std::vector<double> databaseQueryMs;
  std::vector<double> sim3EstimationMs;
  std::vector<double> placeRecognitionTotalMs;
  std::vector<double> mergeMapsMs;
  std::vector<double> weldingBundleAdjustmentMs;
  std::vector<double> mergeEssentialGraphMs;
  std::vector<double> mergeTotalMs;
  std::vector<int> mergeKeyframes;
  std::vector<int> mergeMapPoints;
  int mergeExecutions{0};
  std::vector<double> loopFusionMs;
  std::vector<double> loopEssentialGraphMs;
  std::vector<double> loopTotalMs;
  std::vector<int> loopKeyframes;
  int loopExecutions{0};
  std::vector<double> globalBundleAdjustmentMs;
  std::vector<double> mapUpdateMs;
  std::vector<double> fullGlobalBundleAdjustmentTotalMs;
  std::vector<int> globalBundleAdjustmentKeyframes;
  std::vector<int> globalBundleAdjustmentMapPoints;
  int globalBundleAdjustmentExecutions{0};
  int globalBundleAdjustmentAborts{0};
};

class IOrbLoopClosingBackend {
public:
  virtual ~IOrbLoopClosingBackend() = default;

  virtual OrbLoopClosingStatus Status() const = 0;
  virtual OrbLoopClosingTimingStats TimingStats() const = 0;
  virtual void InsertKeyFrame(
      const OrbLoopClosingKeyFrameRequest &request) = 0;
  virtual void AbortGlobalBundleAdjustment() = 0;
  virtual void RequestFinish() = 0;
  virtual void Reset(const OrbLoopClosingResetRequest &request) = 0;
  virtual void AttachRuntimePeers(
      const OrbLoopClosingRuntimePeers &peers) = 0;
};

std::unique_ptr<IOrbLoopClosingBackend>
CreateDefaultOrbLoopClosingBackend(LoopClosing *loopClosing);

} // namespace ORB_SLAM3

#endif // ORB_LOOP_CLOSING_BACKEND_H
