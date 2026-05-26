#pragma once

#include "core/application/session/slam/slam_frame_step_result.h"
#include "core/ports/slam_backend_maintenance.h"

namespace SmartDrone::Core::Application {

class SlamBackendMaintenancePort final {
  public:
    explicit SlamBackendMaintenancePort(
        SmartDrone::Core::Ports::ISlamBackendMaintenance *backend);

    void RequestStop();
    bool Stopped() const;
    SlamFrameStepResult StepBackend();

  private:
    SmartDrone::Core::Ports::ISlamBackendMaintenance *m_backend{nullptr};
};

} // namespace SmartDrone::Core::Application
