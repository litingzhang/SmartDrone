#pragma once

#include <cstdint>
#include <string>

namespace SmartDrone::Core::Ports {

struct RuntimeActionRequest {
    std::string action;
    uint32_t sequence{0};
};

class ICommandChannel {
  public:
    virtual ~ICommandChannel() = default;

    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual bool Healthy() const = 0;
};

} // namespace SmartDrone::Core::Ports
