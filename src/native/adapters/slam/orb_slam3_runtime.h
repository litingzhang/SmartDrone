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

namespace SmartDrone::Adapters::Slam {

class OrbSlam3Runtime final : public Core::Ports::ISlamTrackingBackend {
  public:
    OrbSlam3Runtime(std::string vocabularyPath, std::string settingsPath,
                    SensorMode sensorMode, bool useViewer);
    ~OrbSlam3Runtime() override;

    bool Available() const override;
    void SetOperationMode(Core::Domain::SlamOperationMode mode) override;
    void StepBackend() override;
    void Shutdown() override;
    bool ShutdownAndSaveTrajectoryEuRoC(const std::string &path) override;

    Sophus::SE3f TrackRaw(const Core::Ports::SlamInputBatch &input,
                          Core::Ports::SlamInputMode inputMode,
                          bool useImu);
    Sophus::SE3f
    TrackRaw(const Core::Ports::SlamTrackRequest &request) override;
    Sophus::SE3f TrackPreparedStereoWithFeatures(
        const Core::Ports::PreparedStereoFeatureTrackRequest &request) override;
    bool PrepareStereoImagesForTracking(const cv::Mat &left, const cv::Mat &right,
                                        cv::Mat &leftPrepared,
                                        cv::Mat &rightPrepared) const;
    bool PrepareStereoImagesForTracking(
        const Core::Ports::StereoPreprocessRequest &request,
        Core::Ports::StereoPreprocessResult &result) const override;

    int TrackingState() const override;
    int TrackedMapPointCount() const override;
    bool IsTrackingInitializing() const override;
    bool IsTrackingRecovering() const override;
    bool HasTrackingInitialized() const override;
    const Core::Ports::IVisualDescriptorProvider *
    LeftDescriptorProvider() override;
    const Core::Ports::IVisualDescriptorProvider *
    RightDescriptorProvider() override;
    bool GetLatestFrameTrajectoryPoseEuRoC(Sophus::SE3f &twc,
                                           double *timestamp = nullptr,
                                           bool *lost = nullptr) const override;

    Core::Ports::SlamMapSummary GetMapSummary() const override;
    Core::Ports::SlamBackendStats GetBackendStats() const override;
    Core::Ports::TrackedVisualSummary GetTrackedVisualSummary() const override;
    Core::Ports::TrackedFeatureSnapshot
    ExtractTrackedFeatures(int leftImageWidth, int leftImageHeight,
                           int rightImageWidth, int rightImageHeight) override;
    Core::Ports::TrackedPointCloudSnapshot
    ExtractTrackedPointCloud(size_t maxPointCloudPoints) override;
    Core::Ports::TrackedVisualData
    ExtractTrackedVisualData(int leftImageWidth, int leftImageHeight,
                             int rightImageWidth, int rightImageHeight,
                             bool includePointCloud,
                             size_t maxPointCloudPoints) override;
    Core::Ports::VisualMapSnapshot ExtractVisualMapSnapshot(
        const Core::Ports::VisualMapSnapshotRequest &request) override;
    void
    LogStereoFeatureDiagnostics(uint64_t frameId,
                                const Core::Ports::StereoFeatureObservationPacket
                                    &observations) const override;
    bool Optimize(const Core::Ports::SlamBackendOptimizationRequest &request,
                  Core::Ports::SlamBackendOptimizationResult &result) override;
    bool ApplyMappingOperation(
        const Core::Ports::SlamBackendMappingRequest &request,
        Core::Ports::SlamBackendMappingResult &result) override;
    bool ApplyLoopClosureOperation(
        const Core::Ports::SlamBackendLoopClosureRequest &request,
        Core::Ports::SlamBackendLoopClosureResult &result) override;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
    std::unique_ptr<Core::Ports::IVisualDescriptorProvider>
        m_leftDescriptorProvider;
    std::unique_ptr<Core::Ports::IVisualDescriptorProvider>
        m_rightDescriptorProvider;
    Core::Domain::SlamOperationMode m_operationMode{
        Core::Domain::SlamOperationMode::Mapping};
};

} // namespace SmartDrone::Adapters::Slam
