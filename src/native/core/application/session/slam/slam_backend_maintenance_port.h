#pragma once

#include "core/application/session/slam/slam_frame_step_result.h"

namespace SmartDrone::Core::Application {

class SlamRuntimeControlPort;

class SlamBackendMaintenancePort final {
  public:
    explicit SlamBackendMaintenancePort(SlamRuntimeControlPort &control);

    SlamFrameStepResult StepBackend();

  private:
    SlamRuntimeControlPort &m_control;
};

} // namespace SmartDrone::Core::Application
