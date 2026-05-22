#include "core/application/runtime/system_runtime_tasks.h"

#include <utility>

#include "core/application/runtime/system_runtime_messages.h"

namespace SmartDrone::Core::Application {

VehicleTelemetryRxTask::VehicleTelemetryRxTask(
    std::shared_ptr<SystemRuntimeStepServices> services)
    : m_services(std::move(services))
{
}

void VehicleTelemetryRxTask::OnTick(Epg::TaskContext &context)
{
    DrainSystemRuntimePulse(context);
    if (m_services) {
        m_services->OnVehicleTelemetryRxGraphTick();
    }
    PushSystemRuntimePulse(context, m_pulseSequence);
}

const bool VEHICLE_TELEMETRY_RX_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<VehicleTelemetryRxTask>(
        "VehicleTelemetryRxTask");

SetpointStreamTask::SetpointStreamTask(
    std::shared_ptr<SystemRuntimeStepServices> services)
    : m_services(std::move(services))
{
}

void SetpointStreamTask::OnTick(Epg::TaskContext &context)
{
    DrainSystemRuntimePulse(context);
    if (m_services) {
        m_services->OnSetpointStreamGraphTick();
    }
    PushSystemRuntimePulse(context, m_pulseSequence);
}

const bool SETPOINT_STREAM_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SetpointStreamTask>(
        "SetpointStreamTask");

UdpCommandTask::UdpCommandTask(std::shared_ptr<UdpCommandRuntime> runtime)
    : m_runtime(std::move(runtime))
{
}

UdpCommandTask::~UdpCommandTask()
{
    if (m_runtime) {
        m_runtime->Stop();
    }
}

void UdpCommandTask::OnTick(Epg::TaskContext &context)
{
    DrainSystemRuntimePulse(context);
    if (m_runtime) {
        m_runtime->OnGraphTick();
    }
    PushSystemRuntimePulse(context, m_pulseSequence);
}

const bool UDP_COMMAND_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<UdpCommandTask>(
        "UdpCommandTask");

ManualControlTask::ManualControlTask(
    std::shared_ptr<SystemRuntimeStepServices> services)
    : m_services(std::move(services))
{
}

void ManualControlTask::OnTick(Epg::TaskContext &context)
{
    DrainSystemRuntimePulse(context);
    if (m_services) {
        m_services->OnManualControlGraphTick();
    }
    PushSystemRuntimePulse(context, m_pulseSequence);
}

const bool MANUAL_CONTROL_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<ManualControlTask>(
        "ManualControlTask");

ForceRestartTask::ForceRestartTask(
    std::shared_ptr<SystemRuntimeStepServices> services)
    : m_services(std::move(services))
{
}

void ForceRestartTask::OnTick(Epg::TaskContext &context)
{
    DrainSystemRuntimePulse(context);
    if (m_services) {
        m_services->OnForceRestartGraphTick();
    }
    PushSystemRuntimePulse(context, m_pulseSequence);
}

const bool FORCE_RESTART_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<ForceRestartTask>(
        "ForceRestartTask");

RuntimeSupervisorTask::RuntimeSupervisorTask(
    std::shared_ptr<SystemRuntimeStepServices> services)
    : m_services(std::move(services))
{
}

void RuntimeSupervisorTask::OnTick(Epg::TaskContext &context)
{
    DrainSystemRuntimePulse(context);
    if (m_services) {
        m_services->OnSessionSupervisorGraphTick();
    }
    PushSystemRuntimePulse(context, m_pulseSequence);
}

const bool RUNTIME_SUPERVISOR_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<RuntimeSupervisorTask>(
        "RuntimeSupervisorTask");

EpgRedeployTask::EpgRedeployTask(
    std::shared_ptr<SystemRuntimeStepServices> services,
    std::shared_ptr<EpgRedeployCoordinator> redeploy)
    : m_services(std::move(services)), m_redeploy(std::move(redeploy))
{
}

void EpgRedeployTask::OnTick(Epg::TaskContext &context)
{
    DrainSystemRuntimePulse(context);
    if (m_services && m_redeploy) {
        m_services->OnEpgRedeployGraphTick(*m_redeploy);
    }
    PushSystemRuntimePulse(context, m_pulseSequence);
}

const bool EPG_REDEPLOY_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<EpgRedeployTask>(
        "EpgRedeployTask");

DiscoveryBeaconTask::DiscoveryBeaconTask(
    std::shared_ptr<DiscoveryBeaconRuntime> runtime)
    : m_runtime(std::move(runtime))
{
}

DiscoveryBeaconTask::~DiscoveryBeaconTask()
{
    if (m_runtime) {
        m_runtime->Stop();
    }
}

void DiscoveryBeaconTask::OnTick(Epg::TaskContext &context)
{
    DrainSystemRuntimePulse(context);
    if (m_runtime) {
        m_runtime->OnGraphTick();
    }
    PushSystemRuntimePulse(context, m_pulseSequence);
}

const bool DISCOVERY_BEACON_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<DiscoveryBeaconTask>(
        "DiscoveryBeaconTask");

} // namespace SmartDrone::Core::Application
