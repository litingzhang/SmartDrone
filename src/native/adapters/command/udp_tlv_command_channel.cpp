#include "adapters/command/udp_tlv_command_channel.h"

namespace smartdrone::adapters::command {

bool UdpTlvCommandChannel::Start() { return true; }

void UdpTlvCommandChannel::Stop() {}

bool UdpTlvCommandChannel::Healthy() const { return true; }

} // namespace smartdrone::adapters::command
