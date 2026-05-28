#pragma once

#include <memory>

#include "core/application/runtime/udp_command_runtime_config.h"
#include "core/application/runtime/udp_command_runtime_phase.h"

namespace SmartDrone::Core::Application {

class UdpCommandRuntime final : public IUdpCommandRuntimePhase {
  public:
    explicit UdpCommandRuntime(UdpCommandRuntimeConfig config);
    ~UdpCommandRuntime();

    bool Start();
    void Stop() override;
    void OnGraphTick();
    void Step();
    void StepReceive() override;
    void StepHeartbeatTx() override;
    void StepHeartbeatTimeout() override;
    void StepStateTx() override;
    void StepPointCloudTx() override;

  private:
    class Impl;

    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Core::Application
