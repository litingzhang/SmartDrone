#ifndef ORB_TRACKING_BACKEND_H
#define ORB_TRACKING_BACKEND_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ImuTypes.h"
#include "core/ports/slam_backend_state.h"
#include "core/ports/tracked_visual_data.h"
#include "core/ports/visual_feature_data.h"

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace ORB_SLAM3 {

class KeyFrame;
class ORBextractor;
class Tracking;

constexpr int kOrbTrackingStateSystemNotReady = -1;

struct OrbFrameTrajectoryEntry {
  Sophus::SE3f relativePose;
  KeyFrame *referenceKeyFrame{nullptr};
  double timestamp{0.0};
  bool lost{false};
};

struct OrbTrackingStatus {
  int state{0};
  int sensor{0};
  int matchesInliers{0};
  double currentFrameTime{0.0};
  double lastFrameTime{0.0};
  KeyFrame *lastKeyFrame{nullptr};
  bool ok{false};
  bool recentlyLost{false};
  bool lost{false};
  bool monocular{false};
  bool stereo{false};
  bool imuMonocular{false};
  bool usesImu{false};
};

struct OrbTrackingFrameImuUpdate {
  float scale{1.0f};
  IMU::Bias bias;
  KeyFrame *referenceKeyFrame{nullptr};
};

struct OrbStereoTrackRequest {
  const cv::Mat *left{nullptr};
  const cv::Mat *right{nullptr};
  double timestamp{0.0};
  std::string filename;
};

struct OrbStereoFeatureTrackRequest {
  const cv::Mat *left{nullptr};
  const cv::Mat *right{nullptr};
  const SmartDrone::Core::Ports::StereoFeatureObservationPacket
      *features{nullptr};
  double timestamp{0.0};
  std::string filename;
};

struct OrbRgbdTrackRequest {
  const cv::Mat *image{nullptr};
  const cv::Mat *depth{nullptr};
  double timestamp{0.0};
  std::string filename;
};

struct OrbMonocularTrackRequest {
  const cv::Mat *image{nullptr};
  double timestamp{0.0};
  std::string filename;
};

struct OrbMonocularFeatureTrackRequest {
  const cv::Mat *image{nullptr};
  const SmartDrone::Core::Ports::VisualKeypointFeatureSet *features{nullptr};
  double timestamp{0.0};
  std::string filename;
};

struct OrbTrackingVisualDataRequest {
  int leftImageWidth{0};
  int leftImageHeight{0};
  int rightImageWidth{0};
  int rightImageHeight{0};
  bool includePointCloud{false};
  size_t maxPointCloudPoints{0};
};

struct OrbTrackingDatasetChangeRequest {
  bool smallCurrentMap{false};
};

struct OrbTrackingTimingSample {
  double timeMs{0.0};
};

class IOrbTrackingBackend {
public:
  virtual ~IOrbTrackingBackend() = default;

  virtual OrbTrackingStatus Status() const = 0;
  virtual SmartDrone::Core::Ports::SlamMapSummary MapSummary(
      unsigned long mapId) const = 0;
  virtual SmartDrone::Core::Ports::SlamFrameTrackingStats
  FrameTrackingStats() const = 0;
  virtual std::vector<OrbFrameTrajectoryEntry>
  FrameTrajectorySnapshot() const = 0;
  virtual bool LatestFrameTrajectoryEntry(
      OrbFrameTrajectoryEntry &entry) const = 0;
  virtual ORBextractor *LeftORBExtractor() const = 0;
  virtual ORBextractor *RightORBExtractor() const = 0;
  virtual SmartDrone::Core::Ports::TrackedVisualSummary
  TrackedVisualSummarySnapshot() const = 0;
  virtual SmartDrone::Core::Ports::TrackedFeatureSnapshot
  ExtractTrackedFeatures(
      const OrbTrackingVisualDataRequest &request) const = 0;
  virtual SmartDrone::Core::Ports::TrackedPointCloudSnapshot
  ExtractTrackedPointCloud(size_t maxPointCloudPoints) const = 0;
  virtual SmartDrone::Core::Ports::TrackedVisualData
  ExtractTrackedVisualData(
      const OrbTrackingVisualDataRequest &request) const = 0;
  virtual Sophus::SE3f TrackStereo(
      const OrbStereoTrackRequest &request) = 0;
  virtual Sophus::SE3f TrackStereoWithFeatures(
      const OrbStereoFeatureTrackRequest &request) = 0;
  virtual Sophus::SE3f TrackRgbd(const OrbRgbdTrackRequest &request) = 0;
  virtual Sophus::SE3f TrackMonocular(
      const OrbMonocularTrackRequest &request) = 0;
  virtual Sophus::SE3f TrackMonocularWithFeatures(
      const OrbMonocularFeatureTrackRequest &request) = 0;
  virtual void PushImu(const IMU::Point &imuMeasurement) = 0;
  virtual void Reset(bool activeMapOnly) = 0;
  virtual void SetOnlyTracking(bool flag) = 0;
  virtual void UpdateFrameImu(
      const OrbTrackingFrameImuUpdate &request) = 0;
  virtual void SetStateOk() = 0;
  virtual void SetImuInitializationTimestamp(double timestamp) = 0;
  virtual void ApplyDatasetChange(
      const OrbTrackingDatasetChangeRequest &request) = 0;
  virtual float ImageScale() const = 0;
  virtual void PrintTimeStats() const = 0;
  virtual void InsertRectTime(
      const OrbTrackingTimingSample &sample) = 0;
  virtual void InsertResizeTime(
      const OrbTrackingTimingSample &sample) = 0;
  virtual void InsertTrackTime(
      const OrbTrackingTimingSample &sample) = 0;
};

std::unique_ptr<IOrbTrackingBackend>
CreateDefaultOrbTrackingBackend(Tracking *tracking);

} // namespace ORB_SLAM3

#endif // ORB_TRACKING_BACKEND_H
