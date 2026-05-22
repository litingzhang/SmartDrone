#include "core/application/runtime/system_runtime_step_services.h"

namespace SmartDrone::core::application {

SystemRuntimeStepServices::SystemRuntimeStepServices(
    SystemRuntimeStepServicesConfig config)
    : m_config(std::move(config))
{
}

bool SystemRuntimeStepServices::Valid() const
{
    return m_config.vehicleTelemetryRx && m_config.setpointStream &&
           m_config.manualControl && m_config.forceRestart &&
           m_config.sessionSupervisor && m_config.epgRedeploy;
}

void SystemRuntimeStepServices::OnVehicleTelemetryRxGraphTick()
{
    Call(m_config.vehicleTelemetryRx);
}

void SystemRuntimeStepServices::OnSetpointStreamGraphTick()
{
    Call(m_config.setpointStream);
}

void SystemRuntimeStepServices::OnManualControlGraphTick()
{
    Call(m_config.manualControl);
}

void SystemRuntimeStepServices::OnForceRestartGraphTick()
{
    Call(m_config.forceRestart);
}

void SystemRuntimeStepServices::OnSessionSupervisorGraphTick()
{
    Call(m_config.sessionSupervisor);
}

void SystemRuntimeStepServices::OnEpgRedeployGraphTick(
    EpgRedeployCoordinator &coordinator)
{
    if (m_config.epgRedeploy) {
        m_config.epgRedeploy(coordinator);
    }
}

void SystemRuntimeStepServices::Call(const RuntimeGraphStepFn &step)
{
    if (step) {
        step();
    }
}

} // namespace SmartDrone::core::application
