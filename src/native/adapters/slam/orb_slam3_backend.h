#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "adapters/slam/external_feature_types.h"
#include "core/application/config/app_args.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/slam_engine.h"

namespace smartdrone::adapters::slam {

enum class SlamInputMode : uint8_t;

struct TrackedVisualSnapshot {
    int matchesInliers{0};
    size_t trackedMapPointCount{0};
    size_t localMapPointCount{0};
    uint64_t localMapPointHash{0};
    uint64_t matchedMapPointHashBeforePoseOptimization{0};
    uint64_t trackedMapPointHash{0};
    std::vector<float> pointCloudXyz;
    std::vector<cv::Point2f> leftFeatures;
    std::vector<cv::Point2f> rightFeatures;
};

class OrbSlam3Backend final {
  public:
    OrbSlam3Backend(std::string vocabularyPath, std::string settingsPath, SensorMode sensorMode, bool useViewer);
    ~OrbSlam3Backend();

    bool Available() const;
    void SetOperationMode(core::domain::SlamOperationMode mode);
    void Shutdown();
    bool ShutdownAndSaveTrajectoryEuRoC(const std::string &path);

    Sophus::SE3f TrackRaw(const core::ports::SlamInputBatch &input, SlamInputMode inputMode, bool useImu);
    Sophus::SE3f TrackPreparedStereoWithFeatures(const core::ports::SlamInputBatch &input,
                                                 const cv::Mat &leftPrepared,
                                                 const cv::Mat &rightPrepared,
                                                 const ExternalStereoObservationPacket &observations,
                                                 bool useImu);
    bool PrepareStereoImagesForTracking(const cv::Mat &left, const cv::Mat &right,
                                        cv::Mat &leftPrepared, cv::Mat &rightPrepared) const;

    int TrackingState() const;
    int TrackedMapPointCount() const;
    bool IsTrackingInitializing() const;
    bool IsTrackingRecovering() const;
    bool HasTrackingInitialized() const;
    const ExternalDescriptorProvider *LeftDescriptorProvider();
    const ExternalDescriptorProvider *RightDescriptorProvider();
    bool GetLatestFrameTrajectoryPoseEuRoC(Sophus::SE3f &twc, double *timestamp = nullptr,
                                           bool *lost = nullptr) const;

    void CopyMapSummaryToOutput(core::ports::SlamOutput &out) const;
    void CopyTrackingStatsToOutput(core::ports::SlamOutput &out) const;
    TrackedVisualSnapshot ExtractTrackedVisualSnapshot(int leftImageWidth, int leftImageHeight,
                                                       int rightImageWidth, int rightImageHeight,
                                                       bool includePointCloud,
                                                       size_t maxPointCloudPoints);
    void LogExternalStereoDfx(uint64_t frameId,
                              const ExternalStereoObservationPacket &observations) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
    std::unique_ptr<ExternalDescriptorProvider> m_leftDescriptorProvider;
    std::unique_ptr<ExternalDescriptorProvider> m_rightDescriptorProvider;
    core::domain::SlamOperationMode m_operationMode{core::domain::SlamOperationMode::Mapping};
};

} // namespace smartdrone::adapters::slam
