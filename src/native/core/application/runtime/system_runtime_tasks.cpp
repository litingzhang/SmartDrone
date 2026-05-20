#include "core/application/runtime/system_runtime_tasks.h"

#include <utility>

namespace smartdrone::core::application {

VehicleTelemetryRxTask::VehicleTelemetryRxTask(
    std::shared_ptr<SystemRuntimeStepServices> services)
    : m_services(std::move(services))
{
}

void VehicleTelemetryRxTask::OnTick(epg::TaskContext &context)
{
    (void)context;
    if (m_services) {
        m_services->OnVehicleTelemetryRxGraphTick();
    }
}

EPG_REGISTER_TASK_TYPE(VehicleTelemetryRxTask, "VehicleTelemetryRxTask")

SetpointStreamTask::SetpointStreamTask(
    std::shared_ptr<SystemRuntimeStepServices> services)
    : m_services(std::move(services))
{
}

void SetpointStreamTask::OnTick(epg::TaskContext &context)
{
    (void)context;
    if (m_services) {
        m_services->OnSetpointStreamGraphTick();
    }
}

EPG_REGISTER_TASK_TYPE(SetpointStreamTask, "SetpointStreamTask")

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

void UdpCommandTask::OnTick(epg::TaskContext &context)
{
    (void)context;
    if (m_runtime) {
        m_runtime->OnGraphTick();
    }
}

EPG_REGISTER_TASK_TYPE(UdpCommandTask, "UdpCommandTask")

ManualControlTask::ManualControlTask(
    std::shared_ptr<SystemRuntimeStepServices> services)
    : m_services(std::move(services))
{
}

void ManualControlTask::OnTick(epg::TaskContext &context)
{
    (void)context;
    if (m_services) {
        m_services->OnManualControlGraphTick();
    }
}

EPG_REGISTER_TASK_TYPE(ManualControlTask, "ManualControlTask")

ForceRestartTask::ForceRestartTask(
    std::shared_ptr<SystemRuntimeStepServices> services)
    : m_services(std::move(services))
{
}

void ForceRestartTask::OnTick(epg::TaskContext &context)
{
    (void)context;
    if (m_services) {
        m_services->OnForceRestartGraphTick();
    }
}

EPG_REGISTER_TASK_TYPE(ForceRestartTask, "ForceRestartTask")

RuntimeSupervisorTask::RuntimeSupervisorTask(
    std::shared_ptr<SystemRuntimeStepServices> services)
    : m_services(std::move(services))
{
}

void RuntimeSupervisorTask::OnTick(epg::TaskContext &context)
{
    (void)context;
    if (m_services) {
        m_services->OnSessionSupervisorGraphTick();
    }
}

EPG_REGISTER_TASK_TYPE(RuntimeSupervisorTask, "RuntimeSupervisorTask")

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

void DiscoveryBeaconTask::OnTick(epg::TaskContext &context)
{
    (void)context;
    if (m_runtime) {
        m_runtime->OnGraphTick();
    }
}

EPG_REGISTER_TASK_TYPE(DiscoveryBeaconTask, "DiscoveryBeaconTask")

} // namespace smartdrone::core::application
