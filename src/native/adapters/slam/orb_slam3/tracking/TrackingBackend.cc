#include "TrackingBackend.h"

#include "System.h"
#include "Tracking.h"

namespace ORB_SLAM3 {

namespace {

class DefaultOrbTrackingBackend final : public IOrbTrackingBackend {
public:
  explicit DefaultOrbTrackingBackend(Tracking *tracking)
      : m_tracking(tracking) {}

  OrbTrackingStatus Status() const override {
    OrbTrackingStatus out;
    if (m_tracking == nullptr) {
      return out;
    }
    out.state = static_cast<int>(m_tracking->mState);
    out.sensor = m_tracking->mSensor;
    out.matchesInliers = m_tracking->GetMatchesInliers();
    out.currentFrameTime = m_tracking->mCurrentFrame.mTimeStamp;
    out.lastFrameTime = m_tracking->mLastFrame.mTimeStamp;
    out.lastKeyFrame = m_tracking->GetLastKeyFrame();
    out.ok = m_tracking->mState == Tracking::OK;
    out.recentlyLost = m_tracking->mState == Tracking::RECENTLY_LOST;
    out.lost = m_tracking->mState == Tracking::LOST;
    out.monocular = m_tracking->mSensor == System::MONOCULAR;
    out.stereo = m_tracking->mSensor == System::STEREO;
    out.imuMonocular = m_tracking->mSensor == System::IMU_MONOCULAR;
    out.usesImu = m_tracking->mSensor == System::IMU_MONOCULAR ||
                  m_tracking->mSensor == System::IMU_STEREO ||
                  m_tracking->mSensor == System::IMU_RGBD;
    return out;
  }

  SmartDrone::core::ports::SlamMapSummary MapSummary(
      unsigned long mapId) const override {
    SmartDrone::core::ports::SlamMapSummary out;
    out.mapId = mapId;
    if (m_tracking == nullptr) {
      return out;
    }
    out.matchesInliers = m_tracking->GetMatchesInliers();
    out.trackedMapPointCount =
        static_cast<uint32_t>(m_tracking->GetTrackedMapPointCount());
    out.localMapPointCount =
        static_cast<uint32_t>(m_tracking->GetLocalMapPointCount());
    out.localMapPointHash = m_tracking->GetLocalMapPointHash();
    out.matchedMapPointHashBeforePoseOptimization =
        m_tracking->GetMatchedMapPointHashBeforePoseOptimization();
    out.trackedMapPointHash = m_tracking->GetTrackedMapPointHash();
    return out;
  }

  SmartDrone::core::ports::SlamFrameTrackingStats
  FrameTrackingStats() const override {
    return m_tracking != nullptr ? m_tracking->GetFrameTrackingStats()
                                 : SmartDrone::core::ports::
                                       SlamFrameTrackingStats{};
  }

  std::vector<OrbFrameTrajectoryEntry>
  FrameTrajectorySnapshot() const override {
    return m_tracking != nullptr ? m_tracking->GetFrameTrajectorySnapshot()
                                 : std::vector<OrbFrameTrajectoryEntry>{};
  }

  bool LatestFrameTrajectoryEntry(
      OrbFrameTrajectoryEntry &entry) const override {
    return m_tracking != nullptr &&
           m_tracking->GetLatestFrameTrajectoryEntry(entry);
  }

  ORBextractor *LeftORBExtractor() const override {
    return m_tracking != nullptr ? m_tracking->GetLeftORBExtractor() : nullptr;
  }

  ORBextractor *RightORBExtractor() const override {
    return m_tracking != nullptr ? m_tracking->GetRightORBExtractor() : nullptr;
  }

  SmartDrone::core::ports::TrackedVisualSummary
  TrackedVisualSummarySnapshot() const override {
    return m_tracking != nullptr
               ? m_tracking->GetTrackedVisualSummary()
               : SmartDrone::core::ports::TrackedVisualSummary{};
  }

  SmartDrone::core::ports::TrackedFeatureSnapshot
  ExtractTrackedFeatures(
      const OrbTrackingVisualDataRequest &request) const override {
    return m_tracking != nullptr
               ? m_tracking->ExtractTrackedFeatures(
                     request.leftImageWidth, request.leftImageHeight,
                     request.rightImageWidth, request.rightImageHeight)
               : SmartDrone::core::ports::TrackedFeatureSnapshot{};
  }

  SmartDrone::core::ports::TrackedPointCloudSnapshot
  ExtractTrackedPointCloud(size_t maxPointCloudPoints) const override {
    return m_tracking != nullptr
               ? m_tracking->ExtractTrackedPointCloud(maxPointCloudPoints)
               : SmartDrone::core::ports::TrackedPointCloudSnapshot{};
  }

  SmartDrone::core::ports::TrackedVisualData ExtractTrackedVisualData(
      const OrbTrackingVisualDataRequest &request) const override {
    return m_tracking != nullptr
               ? m_tracking->ExtractTrackedVisualData(
                     request.leftImageWidth, request.leftImageHeight,
                     request.rightImageWidth, request.rightImageHeight,
                     request.includePointCloud, request.maxPointCloudPoints)
               : SmartDrone::core::ports::TrackedVisualData{};
  }

  Sophus::SE3f TrackStereo(
      const OrbStereoTrackRequest &request) override {
    if (m_tracking == nullptr || request.left == nullptr ||
        request.right == nullptr) {
      return Sophus::SE3f();
    }
    return m_tracking->GrabImageStereo(*request.left, *request.right,
                                       request.timestamp, request.filename);
  }

  Sophus::SE3f TrackStereoWithFeatures(
      const OrbStereoFeatureTrackRequest &request) override {
    if (m_tracking == nullptr || request.left == nullptr ||
        request.right == nullptr || request.features == nullptr) {
      return Sophus::SE3f();
    }
    return m_tracking->GrabImageStereoWithFeatures(
        *request.left, *request.right, *request.features, request.timestamp,
        request.filename);
  }

  Sophus::SE3f TrackRgbd(const OrbRgbdTrackRequest &request) override {
    if (m_tracking == nullptr || request.image == nullptr ||
        request.depth == nullptr) {
      return Sophus::SE3f();
    }
    return m_tracking->GrabImageRGBD(*request.image, *request.depth,
                                     request.timestamp, request.filename);
  }

  Sophus::SE3f TrackMonocular(
      const OrbMonocularTrackRequest &request) override {
    if (m_tracking == nullptr || request.image == nullptr) {
      return Sophus::SE3f();
    }
    return m_tracking->GrabImageMonocular(*request.image, request.timestamp,
                                          request.filename);
  }

  Sophus::SE3f TrackMonocularWithFeatures(
      const OrbMonocularFeatureTrackRequest &request) override {
    if (m_tracking == nullptr || request.image == nullptr ||
        request.features == nullptr) {
      return Sophus::SE3f();
    }
    return m_tracking->GrabImageMonocularWithFeatures(
        *request.image, *request.features, request.timestamp,
        request.filename);
  }

  void PushImu(const IMU::Point &imuMeasurement) override {
    if (m_tracking != nullptr) {
      m_tracking->GrabImuData(imuMeasurement);
    }
  }

  void Reset(bool activeMapOnly) override {
    if (m_tracking == nullptr) {
      return;
    }
    if (activeMapOnly) {
      m_tracking->ResetActiveMap();
      return;
    }
    m_tracking->Reset();
  }

  void SetOnlyTracking(bool flag) override {
    if (m_tracking != nullptr) {
      m_tracking->InformOnlyTracking(flag);
    }
  }

  void UpdateFrameImu(
      const OrbTrackingFrameImuUpdate &request) override {
    if (m_tracking == nullptr) {
      return;
    }
    m_tracking->UpdateFrameIMU(request.scale, request.bias,
                               request.referenceKeyFrame);
  }

  void SetStateOk() override {
    if (m_tracking != nullptr) {
      m_tracking->mState = Tracking::OK;
    }
  }

  void SetImuInitializationTimestamp(double timestamp) override {
    if (m_tracking != nullptr) {
      m_tracking->t0IMU = timestamp;
    }
  }

  void ApplyDatasetChange(
      const OrbTrackingDatasetChangeRequest &request) override {
    if (m_tracking == nullptr) {
      return;
    }
    if (request.smallCurrentMap) {
      m_tracking->ResetActiveMap();
    } else {
      m_tracking->CreateMapInAtlas();
    }
    m_tracking->NewDataset();
  }

  float ImageScale() const override {
    return m_tracking != nullptr ? m_tracking->GetImageScale() : 1.0f;
  }

  void PrintTimeStats() const override {
#ifdef REGISTER_TIMES
    if (m_tracking != nullptr) {
      m_tracking->PrintTimeStats();
    }
#endif
  }

  void InsertRectTime(const OrbTrackingTimingSample &sample) override {
#ifdef REGISTER_TIMES
    if (m_tracking != nullptr) {
      m_tracking->vdRectStereo_ms.push_back(sample.timeMs);
    }
#endif
  }

  void InsertResizeTime(const OrbTrackingTimingSample &sample) override {
#ifdef REGISTER_TIMES
    if (m_tracking != nullptr) {
      m_tracking->vdResizeImage_ms.push_back(sample.timeMs);
    }
#endif
  }

  void InsertTrackTime(const OrbTrackingTimingSample &sample) override {
#ifdef REGISTER_TIMES
    if (m_tracking != nullptr) {
      m_tracking->vdTrackTotal_ms.push_back(sample.timeMs);
    }
#endif
  }

private:
  Tracking *m_tracking{nullptr};
};

} // namespace

std::unique_ptr<IOrbTrackingBackend>
CreateDefaultOrbTrackingBackend(Tracking *tracking) {
  return std::make_unique<DefaultOrbTrackingBackend>(tracking);
}

} // namespace ORB_SLAM3
