#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "core/application/session/calib/calib_capture_result.h"
#include "core/application/session/calib/calib_imu_sample_result.h"
#include "core/application/session/epg/messages/calib_epg_messages.h"

namespace SmartDrone::Core::Application {

struct ApplicationRuntimeFactories;
struct MainRuntimeAliases;

enum class CalibSessionPortOpenStatus : std::uint8_t {
    Ready,
    StorageFailed,
    PreviewFailed,
    CameraFailed,
};

struct CalibSessionPortOpenResult {
    CalibSessionPortOpenStatus status{CalibSessionPortOpenStatus::StorageFailed};
};

struct CalibSessionPortSetConfig {
    const MainRuntimeAliases &aliases;
    std::string root;
    const ApplicationRuntimeFactories &factories;
};

class CalibSessionPortSet final {
  public:
    explicit CalibSessionPortSet(CalibSessionPortSetConfig config);
    ~CalibSessionPortSet();

    CalibSessionPortOpenResult Open();
    bool ShouldFinishCapture(int maxFrames) const;
    CalibFrameCaptureResult TryCaptureFrame(CalibStereoFrame &frame);
    bool TryBuildSavePair(std::shared_ptr<CalibStereoFrame> frame,
                          CalibSavePair &savePair);
    bool WriteSavePair(const CalibSavePair &pair);
    bool WriteImuSample(const ImuSample &sample);
    bool EnqueuePreview(const CalibStereoFrame &frame);
    CalibImuSampleResult StepImuSample();
    bool FlushAndCloseStorage();
    void StopPorts();
    void LogFinalStatus() const;

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Core::Application
