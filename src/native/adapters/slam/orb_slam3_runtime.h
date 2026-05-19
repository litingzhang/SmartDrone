#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "core/application/config/app_args.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/slam_backend.h"
#include "core/ports/slam_engine.h"
#include "core/ports/visual_descriptor.h"

namespace smartdrone::adapters::slam {

class OrbSlam3Runtime final : public core::ports::ISlamTrackingBackend {
public:
  OrbSlam3Runtime(std::string vocabularyPath, std::string settingsPath,
                  SensorMode sensorMode, bool useViewer);
  ~OrbSlam3Runtime() override;

  bool Available() const override;
  void SetOperationMode(core::domain::SlamOperationMode mode) override;
  void StepBackend() override;
  void Shutdown() override;
  bool ShutdownAndSaveTrajectoryEuRoC(const std::string &path) override;

  Sophus::SE3f TrackRaw(const core::ports::SlamInputBatch &input,
                        core::ports::SlamInputMode inputMode,
                        bool useImu);
  Sophus::SE3f
  TrackRaw(const core::ports::SlamTrackRequest &request) override;
  Sophus::SE3f TrackPreparedStereoWithFeatures(
      const core::ports::PreparedStereoFeatureTrackRequest &request) override;
  bool PrepareStereoImagesForTracking(const cv::Mat &left, const cv::Mat &right,
                                      cv::Mat &leftPrepared,
                                      cv::Mat &rightPrepared) const;
  bool PrepareStereoImagesForTracking(
      const core::ports::StereoPreprocessRequest &request,
      core::ports::StereoPreprocessResult &result) const override;

  int TrackingState() const override;
  int TrackedMapPointCount() const override;
  bool IsTrackingInitializing() const override;
  bool IsTrackingRecovering() const override;
  bool HasTrackingInitialized() const override;
  const core::ports::IVisualDescriptorProvider *
  LeftDescriptorProvider() override;
  const core::ports::IVisualDescriptorProvider *
  RightDescriptorProvider() override;
  bool GetLatestFrameTrajectoryPoseEuRoC(Sophus::SE3f &twc,
                                         double *timestamp = nullptr,
                                         bool *lost = nullptr) const override;

  core::ports::SlamMapSummary GetMapSummary() const override;
  core::ports::SlamBackendStats GetBackendStats() const override;
  core::ports::TrackedVisualSummary GetTrackedVisualSummary() const override;
  core::ports::TrackedFeatureSnapshot
  ExtractTrackedFeatures(int leftImageWidth, int leftImageHeight,
                         int rightImageWidth, int rightImageHeight) override;
  core::ports::TrackedPointCloudSnapshot
  ExtractTrackedPointCloud(size_t maxPointCloudPoints) override;
  core::ports::TrackedVisualData
  ExtractTrackedVisualData(int leftImageWidth, int leftImageHeight,
                           int rightImageWidth, int rightImageHeight,
                           bool includePointCloud,
                           size_t maxPointCloudPoints) override;
  core::ports::VisualMapSnapshot ExtractVisualMapSnapshot(
      const core::ports::VisualMapSnapshotRequest &request) override;
  void
  LogStereoFeatureDiagnostics(uint64_t frameId,
                              const core::ports::StereoFeatureObservationPacket
                                  &observations) const override;
  bool Optimize(const core::ports::SlamBackendOptimizationRequest &request,
                core::ports::SlamBackendOptimizationResult &result) override;
  bool ApplyMappingOperation(
      const core::ports::SlamBackendMappingRequest &request,
      core::ports::SlamBackendMappingResult &result) override;
  bool ApplyLoopClosureOperation(
      const core::ports::SlamBackendLoopClosureRequest &request,
      core::ports::SlamBackendLoopClosureResult &result) override;

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
  std::unique_ptr<core::ports::IVisualDescriptorProvider>
      m_leftDescriptorProvider;
  std::unique_ptr<core::ports::IVisualDescriptorProvider>
      m_rightDescriptorProvider;
  core::domain::SlamOperationMode m_operationMode{
      core::domain::SlamOperationMode::Mapping};
};

} // namespace smartdrone::adapters::slam
