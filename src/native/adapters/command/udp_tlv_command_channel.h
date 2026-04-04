#pragma once

#include "core/ports/command_channel.h"

namespace smartdrone::adapters::command {

class UdpTlvCommandChannel final : public core::ports::ICommandChannel {
  public:
    bool Start() override;
    void Stop() override;
    bool Healthy() const override;
};

} // namespace smartdrone::adapters::command
