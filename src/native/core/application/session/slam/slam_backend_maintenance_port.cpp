#include "core/application/session/slam/slam_backend_maintenance_port.h"

#include "core/application/session/slam/slam_runtime_control_port.h"

namespace smartdrone::core::application {

SlamBackendMaintenancePort::SlamBackendMaintenancePort(
    SlamRuntimeControlPort &control)
    : m_control(control)
{
}

SlamFrameStepResult SlamBackendMaintenancePort::StepBackend()
{
    m_control.StepBackend();
    return SlamFrameStepResult::Continue;
}

} // namespace smartdrone::core::application
