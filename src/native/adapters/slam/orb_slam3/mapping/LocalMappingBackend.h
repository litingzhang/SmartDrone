#ifndef LOCAL_MAPPING_BACKEND_H
#define LOCAL_MAPPING_BACKEND_H

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace ORB_SLAM3 {

class KeyFrame;
class LocalMapping;
class LoopClosing;
class Map;
class Tracking;
class IOrbTrackingBackend;

struct OrbLocalMappingStatus {
  bool stopped{false};
  bool stopRequested{false};
  bool acceptingKeyframes{false};
  bool initializing{false};
  bool finished{false};
  bool badImu{false};
  bool farPoints{false};
  float farPointThreshold{0.0f};
  int keyframesInQueue{0};
  int matchesInliers{0};
  double firstTimestamp{0.0};
  double currentKeyframeTime{0.0};
};

struct OrbLocalMappingKeyFrameRequest {
  KeyFrame *keyframe{nullptr};
};

struct OrbLocalMappingResetRequest {
  Map *map{nullptr};
  bool activeMapOnly{false};
};

struct OrbLocalMappingRuntimeConfig {
  int initFrame{0};
  float farPointThreshold{0.0f};
};

struct OrbLocalMappingRuntimePeers {
  IOrbTrackingBackend *tracking{nullptr};
  LoopClosing *loopCloser{nullptr};
};

struct OrbLocalMappingDebugSnapshot {
  bool available{false};
  unsigned int initSection{0};
  double scale{1.0};
  Eigen::Matrix3d gravityRotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d gyroBias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accBias{Eigen::Vector3d::Zero()};
  Eigen::MatrixXd inertialCovariance;
  double costTime{0.0};
  double initTime{0.0};
};

struct OrbLocalMappingTimingStats {
  std::vector<double> keyframeInsertMs;
  std::vector<double> mapPointCullingMs;
  std::vector<double> mapPointCreationMs;
  std::vector<double> localBundleAdjustmentMs;
  std::vector<double> localBundleAdjustmentSyncMs;
  std::vector<double> keyframeCullingMs;
  std::vector<double> keyframeCullingSyncMs;
  std::vector<double> totalMs;
  std::vector<int> localBundleAdjustmentEdges;
  std::vector<int> localBundleAdjustmentOptimizedKeyframes;
  std::vector<int> localBundleAdjustmentFixedKeyframes;
  std::vector<int> localBundleAdjustmentMapPoints;
  int localBundleAdjustmentExecutions{0};
  int localBundleAdjustmentAborts{0};
};

class IOrbLocalMappingBackend {
public:
  virtual ~IOrbLocalMappingBackend() = default;

  virtual OrbLocalMappingStatus Status() const = 0;
  virtual OrbLocalMappingDebugSnapshot DebugSnapshot() const = 0;
  virtual OrbLocalMappingTimingStats TimingStats() const = 0;
  virtual void SetMatchesInliers(int matchesInliers) = 0;
  virtual void InsertKeyFrame(
      const OrbLocalMappingKeyFrameRequest &request) = 0;
  virtual void InterruptBundleAdjustment() = 0;
  virtual bool SetNotStop(bool flag) = 0;
  virtual void SetFirstTimestamp(double timestamp) = 0;
  virtual void RequestStop() = 0;
  virtual void RequestFinish() = 0;
  virtual void Release() = 0;
  virtual void EmptyQueue() = 0;
  virtual void AttachRuntimePeers(
      const OrbLocalMappingRuntimePeers &peers) = 0;
  virtual void ApplyRuntimeConfig(
      const OrbLocalMappingRuntimeConfig &config) = 0;
  virtual void Reset(const OrbLocalMappingResetRequest &request) = 0;
  virtual bool Step() = 0;
};

std::unique_ptr<IOrbLocalMappingBackend>
CreateDefaultOrbLocalMappingBackend(LocalMapping *localMapping);

} // namespace ORB_SLAM3

#endif // LOCAL_MAPPING_BACKEND_H
