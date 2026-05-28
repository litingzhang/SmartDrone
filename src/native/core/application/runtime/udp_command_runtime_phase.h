#pragma once

namespace SmartDrone::Core::Application {

class IUdpCommandRuntimePhase {
  public:
    virtual ~IUdpCommandRuntimePhase() = default;

    virtual void Stop() = 0;
    virtual void StepReceive() = 0;
    virtual void StepHeartbeatTx() = 0;
    virtual void StepHeartbeatTimeout() = 0;
    virtual void StepStateTx() = 0;
    virtual void StepPointCloudTx() = 0;
};

} // namespace SmartDrone::Core::Application
