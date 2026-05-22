#pragma once

#include <atomic>
#include <cstdint>

#include "common/tlv/udp_server.h"
#include "core/ports/command_channel.h"

namespace SmartDrone::adapters::command {

class UdpTlvCommandChannel final : public core::ports::ICommandChannel {
  public:
    explicit UdpTlvCommandChannel(uint16_t port = 14550);

    bool Start() override;
    void Stop() override;
    bool Healthy() const override;

  private:
    uint16_t m_port{14550};
    std::atomic<bool> m_started{false};
    std::atomic<bool> m_healthy{false};
    UdpServer m_server;
};

} // namespace SmartDrone::adapters::command
