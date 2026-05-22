#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "core/domain/runtime_mode.h"
#include "core/ports/slam_backend_modules.h"
#include "core/ports/slam_backend_state.h"
#include "core/ports/slam_engine.h"
#include "core/ports/slam_trajectory.h"
#include "core/ports/visual_descriptor.h"

namespace SmartDrone::core::ports {

enum class SlamInputMode : uint8_t {
    Stereo,
    MonoLeft,
    MonoRight,
};

struct SlamTrackRequest {
    const SlamInputBatch *input{nullptr};
    SlamInputMode inputMode{SlamInputMode::Stereo};
    bool useImu{false};
};

struct StereoPreprocessRequest {
    const cv::Mat *left{nullptr};
    const cv::Mat *right{nullptr};
};

struct StereoPreprocessResult {
    cv::Mat leftPrepared;
    cv::Mat rightPrepared;
};

struct PreparedStereoFeatureTrackRequest {
    const SlamInputBatch *input{nullptr};
    const cv::Mat *leftPrepared{nullptr};
    const cv::Mat *rightPrepared{nullptr};
    const StereoFeatureObservationPacket *observations{nullptr};
    bool useImu{false};
};

class ISlamBackendLifecycle {
  public:
    virtual ~ISlamBackendLifecycle() = default;

    virtual bool Available() const = 0;
    virtual void SetOperationMode(core::domain::SlamOperationMode mode) = 0;
    virtual void StepBackend()
    {
    }
    virtual void Shutdown() = 0;
};

class ISlamFrameTracker {
  public:
    virtual ~ISlamFrameTracker() = default;

    virtual Sophus::SE3f TrackRaw(const SlamTrackRequest &request) = 0;
};

class ISlamStereoFeatureInjection {
  public:
    virtual ~ISlamStereoFeatureInjection() = default;

    virtual Sophus::SE3f TrackPreparedStereoWithFeatures(
        const PreparedStereoFeatureTrackRequest &request) = 0;
};

class ISlamStereoPreprocessor {
  public:
    virtual ~ISlamStereoPreprocessor() = default;

    virtual bool
    PrepareStereoImagesForTracking(const StereoPreprocessRequest &request,
                                   StereoPreprocessResult &result) const = 0;
};

class ISlamTrackingStatusProvider {
  public:
    virtual ~ISlamTrackingStatusProvider() = default;

    virtual int TrackingState() const = 0;
    virtual int TrackedMapPointCount() const = 0;
    virtual bool IsTrackingInitializing() const = 0;
    virtual bool IsTrackingRecovering() const = 0;
    virtual bool HasTrackingInitialized() const = 0;
};

class ISlamDescriptorProviderSource {
  public:
    virtual ~ISlamDescriptorProviderSource() = default;

    virtual const IVisualDescriptorProvider *LeftDescriptorProvider() = 0;
    virtual const IVisualDescriptorProvider *RightDescriptorProvider() = 0;
};

class ISlamStereoFeatureDiagnosticsSink {
  public:
    virtual ~ISlamStereoFeatureDiagnosticsSink() = default;

    virtual void LogStereoFeatureDiagnostics(
        uint64_t frameId,
        const StereoFeatureObservationPacket &observations) const = 0;
};

class ISlamTrackingBackend : public ISlamBackendLifecycle,
                             public ISlamFrameTracker,
                             public ISlamStereoFeatureInjection,
                             public ISlamStereoPreprocessor,
                             public ISlamTrackingStatusProvider,
                             public ISlamDescriptorProviderSource,
                             public ISlamTrajectoryProvider,
                             public ISlamTrajectorySaver,
                             public ISlamBackendStateProvider,
                             public ITrackedVisualDataProvider,
                             public ISlamStereoFeatureDiagnosticsSink,
                             public ISlamBackendOptimizer,
                             public ISlamBackendLocalMapper,
                             public ISlamBackendLoopCloser {
  public:
    ~ISlamTrackingBackend() override = default;
};

} // namespace SmartDrone::core::ports
