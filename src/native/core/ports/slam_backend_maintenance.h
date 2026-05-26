#pragma once

namespace SmartDrone::Core::Ports {

class ISlamBackendMaintenance {
  public:
    virtual ~ISlamBackendMaintenance() = default;

    virtual void RequestBackendStop() = 0;
    virtual bool BackendStopped() const = 0;
    virtual void StepBackend() = 0;
};

} // namespace SmartDrone::Core::Ports
