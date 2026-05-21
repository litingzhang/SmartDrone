#pragma once

#include "core/application/session/slam/slam_frame_step_result.h"

namespace smartdrone::core::application {

class SlamRuntimeControlPort;

class SlamBackendMaintenancePort final {
  public:
    explicit SlamBackendMaintenancePort(SlamRuntimeControlPort &control);

    SlamFrameStepResult StepBackend();

  private:
    SlamRuntimeControlPort &m_control;
};

} // namespace smartdrone::core::application
