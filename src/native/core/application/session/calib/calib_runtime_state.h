#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include "core/application/session/calib/calib_capture_result.h"
#include "core/application/session/calib/calib_imu_sample_result.h"
#include "core/application/session/epg/messages/calib_epg_messages.h"

namespace SmartDrone::Core::Application {

struct ApplicationRuntimeFactories;
struct LivePoseState;
struct UnifiedConfig;

struct CalibRuntimeStateConfig {
    const UnifiedConfig &cfg;
    std::atomic<bool> &stop;
    LivePoseState &livePose;
    const ApplicationRuntimeFactories &factories;
};

class CalibRuntimeState final {
  public:
    explicit CalibRuntimeState(CalibRuntimeStateConfig config);
    ~CalibRuntimeState();

    bool EnsureStarted();
    bool ShouldFinishCapture() const;
    CalibFrameCaptureResult TryCaptureFrame(CalibStereoFrame &frame);
    bool TryBuildSavePair(std::shared_ptr<CalibStereoFrame> frame,
                          CalibSavePair &savePair);
    bool WriteSavePair(const CalibSavePair &pair);
    bool WriteImuSample(const ImuSample &sample);
    bool EnqueuePreview(const CalibStereoFrame &frame);
    CalibImuSampleResult StepImuSample();
    void RequestStop();
    bool FlushStorage();
    void MarkStorageFlushed();
    bool StorageFlushed() const;
    bool Finalized() const;
    void FinalizeAfterStorageFlushed(bool sessionOk);
    void Finalize(bool sessionOk);

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Core::Application
