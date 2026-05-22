#pragma once

#include <memory>

namespace SmartDrone::Core::Application {

struct ImuThreadState;
struct MainRuntimeAliases;

class ImuSensorPoller {
  public:
    ImuSensorPoller(const MainRuntimeAliases &aliases, ImuThreadState &state);
    ~ImuSensorPoller();

    bool Start();
    void Stop();
    void Step();
    bool Failed() const;

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Core::Application
