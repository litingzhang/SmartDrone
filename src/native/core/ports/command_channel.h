#pragma once

#include <cstdint>
#include <string>

namespace SmartDrone::core::ports {

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

} // namespace SmartDrone::core::ports
