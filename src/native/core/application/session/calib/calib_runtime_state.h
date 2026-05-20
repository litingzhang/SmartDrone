#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include "core/application/session/calib/calib_capture_result.h"
#include "core/application/session/calib/calib_imu_sample_result.h"
#include "core/application/session/epg/messages/calib_epg_messages.h"

namespace smartdrone::core::application {

struct LivePoseState;
struct UnifiedConfig;

struct CalibRuntimeStateConfig {
    const UnifiedConfig &cfg;
    std::atomic<bool> &stop;
    LivePoseState &livePose;
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
    bool EnqueuePreview(const CalibStereoFrame &frame);
    CalibImuSampleResult StepImuSample();
    bool Finalized() const;
    void Finalize(bool sessionOk);

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace smartdrone::core::application
