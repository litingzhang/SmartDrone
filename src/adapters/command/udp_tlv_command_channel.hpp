#pragma once

#include "core/ports/command_channel.hpp"

namespace smartdrone::adapters::command {

class UdpTlvCommandChannel final : public core::ports::ICommandChannel {
public:
    bool Start() override { return true; }
    void Stop() override {}
    bool Healthy() const override { return true; }
};

}  // namespace smartdrone::adapters::command
