#pragma once

#include <memory>

#include "core/application/session/calib/calib_capture_result.h"
#include "core/application/session/epg/messages/calib_epg_messages.h"

namespace smartdrone::core::application {

struct MainRuntimeAliases;

class CalibCameraInputPort final {
  public:
    explicit CalibCameraInputPort(const MainRuntimeAliases &aliases);
    ~CalibCameraInputPort();

    bool Open();
    CalibFrameCaptureResult TryCaptureFrame(CalibStereoFrame &frame);
    void Stop();
    bool Opened() const;

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace smartdrone::core::application
