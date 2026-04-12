#include "adapters/command/udp_tlv_command_channel.h"

namespace smartdrone::adapters::command {

UdpTlvCommandChannel::UdpTlvCommandChannel(uint16_t port) : m_port(port) {}

bool UdpTlvCommandChannel::Start()
{
    if (m_started.load(std::memory_order_relaxed)) {
        return m_healthy.load(std::memory_order_relaxed);
    }
    const bool ok = m_server.Open(m_port);
    m_healthy.store(ok, std::memory_order_relaxed);
    m_started.store(ok, std::memory_order_relaxed);
    return ok;
}

void UdpTlvCommandChannel::Stop()
{
    m_server.Close();
    m_started.store(false, std::memory_order_relaxed);
    m_healthy.store(false, std::memory_order_relaxed);
}

bool UdpTlvCommandChannel::Healthy() const
{
    return m_started.load(std::memory_order_relaxed) && m_healthy.load(std::memory_order_relaxed);
}

} // namespace smartdrone::adapters::command
