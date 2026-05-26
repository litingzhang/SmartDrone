#include "core/application/session/slam/slam_backend_maintenance_port.h"

namespace SmartDrone::Core::Application {

SlamBackendMaintenancePort::SlamBackendMaintenancePort(
    SmartDrone::Core::Ports::ISlamBackendMaintenance *backend)
    : m_backend(backend)
{
}

void SlamBackendMaintenancePort::RequestStop()
{
    if (m_backend != nullptr) {
        m_backend->RequestBackendStop();
    }
}

bool SlamBackendMaintenancePort::Stopped() const
{
    return m_backend == nullptr || m_backend->BackendStopped();
}

SlamFrameStepResult SlamBackendMaintenancePort::StepBackend()
{
    if (m_backend != nullptr) {
        m_backend->StepBackend();
    }
    return SlamFrameStepResult::Continue;
}

} // namespace SmartDrone::Core::Application
