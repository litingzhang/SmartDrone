#pragma once

namespace SmartDrone::Core::Application {

class ISessionGraphRuntime {
  public:
    virtual ~ISessionGraphRuntime() = default;

    virtual bool Start() = 0;
    virtual void Step() = 0;
    virtual void RequestStop() = 0;
    virtual void Stop() = 0;
    virtual bool Done() = 0;
    virtual bool Ok() const = 0;
};

} // namespace SmartDrone::Core::Application
