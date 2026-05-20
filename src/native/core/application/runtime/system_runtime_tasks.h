#pragma once

#include <memory>

#include "common/epg/epg.h"
#include "core/application/runtime/discovery_beacon_runtime.h"
#include "core/application/runtime/system_runtime_step_services.h"
#include "core/application/runtime/udp_command_runtime.h"

namespace smartdrone::core::application {

class VehicleTelemetryRxTask final : public epg::ITask {
  public:
    explicit VehicleTelemetryRxTask(
        std::shared_ptr<SystemRuntimeStepServices> services);

    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
};

class SetpointStreamTask final : public epg::ITask {
  public:
    explicit SetpointStreamTask(
        std::shared_ptr<SystemRuntimeStepServices> services);

    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
};

class UdpCommandTask final : public epg::ITask {
  public:
    explicit UdpCommandTask(std::shared_ptr<UdpCommandRuntime> runtime);
    ~UdpCommandTask() override;

    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<UdpCommandRuntime> m_runtime;
};

class ManualControlTask final : public epg::ITask {
  public:
    explicit ManualControlTask(
        std::shared_ptr<SystemRuntimeStepServices> services);

    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
};

class ForceRestartTask final : public epg::ITask {
  public:
    explicit ForceRestartTask(
        std::shared_ptr<SystemRuntimeStepServices> services);

    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
};

class RuntimeSupervisorTask final : public epg::ITask {
  public:
    explicit RuntimeSupervisorTask(
        std::shared_ptr<SystemRuntimeStepServices> services);

    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
};

class DiscoveryBeaconTask final : public epg::ITask {
  public:
    explicit DiscoveryBeaconTask(
        std::shared_ptr<DiscoveryBeaconRuntime> runtime);
    ~DiscoveryBeaconTask() override;

    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<DiscoveryBeaconRuntime> m_runtime;
};

} // namespace smartdrone::core::application
