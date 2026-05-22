#pragma once

#include <memory>
#include <cstdint>

#include "common/epg/epg.h"
#include "core/application/runtime/discovery_beacon_runtime.h"
#include "core/application/runtime/system_runtime_step_services.h"
#include "core/application/runtime/udp_command_runtime.h"

namespace SmartDrone::Core::Application {

class VehicleTelemetryRxTask final : public Epg::ITask {
  public:
    explicit VehicleTelemetryRxTask(
        std::shared_ptr<SystemRuntimeStepServices> services);

    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
    std::uint64_t m_pulseSequence{0};
};

class SetpointStreamTask final : public Epg::ITask {
  public:
    explicit SetpointStreamTask(
        std::shared_ptr<SystemRuntimeStepServices> services);

    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
    std::uint64_t m_pulseSequence{0};
};

class UdpCommandTask final : public Epg::ITask {
  public:
    explicit UdpCommandTask(std::shared_ptr<UdpCommandRuntime> runtime);
    ~UdpCommandTask() override;

    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<UdpCommandRuntime> m_runtime;
    std::uint64_t m_pulseSequence{0};
};

class ManualControlTask final : public Epg::ITask {
  public:
    explicit ManualControlTask(
        std::shared_ptr<SystemRuntimeStepServices> services);

    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
    std::uint64_t m_pulseSequence{0};
};

class ForceRestartTask final : public Epg::ITask {
  public:
    explicit ForceRestartTask(
        std::shared_ptr<SystemRuntimeStepServices> services);

    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
    std::uint64_t m_pulseSequence{0};
};

class RuntimeSupervisorTask final : public Epg::ITask {
  public:
    explicit RuntimeSupervisorTask(
        std::shared_ptr<SystemRuntimeStepServices> services);

    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
    std::uint64_t m_pulseSequence{0};
};

class EpgRedeployTask final : public Epg::ITask {
  public:
    explicit EpgRedeployTask(
        std::shared_ptr<SystemRuntimeStepServices> services,
        std::shared_ptr<EpgRedeployCoordinator> redeploy);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SystemRuntimeStepServices> m_services;
    std::shared_ptr<EpgRedeployCoordinator> m_redeploy;
    std::uint64_t m_pulseSequence{0};
};

class DiscoveryBeaconTask final : public Epg::ITask {
  public:
    explicit DiscoveryBeaconTask(
        std::shared_ptr<DiscoveryBeaconRuntime> runtime);
    ~DiscoveryBeaconTask() override;

    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<DiscoveryBeaconRuntime> m_runtime;
    std::uint64_t m_pulseSequence{0};
};

} // namespace SmartDrone::Core::Application
