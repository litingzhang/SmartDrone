#pragma once

#include <string>

#include "common/tlv/tlv_parser.h"
#include "common/tlv/udp_server.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/runtime_command_service.h"
#include "core/application/runtime/udp_command_runtime.h"

namespace SmartDrone::core::application {

bool RuntimeConfigPayloadLengthValid(std::uint16_t len);
RemoteRuntimeConfig ParseRuntimeConfigFrame(const TlvFrame &frame,
                                            const UnifiedConfig &currentCfg);
void ApplyPeerIp(RemoteRuntimeConfig &remote, const UdpPeer &peer,
                 const PeerToIpStringFn &peerToIpString);
ConfigUpdate BuildRuntimeConfigUpdate(const RemoteRuntimeConfig &remote);
std::string BuildRuntimeConfigAckMessage(const std::string &message,
                                         const RemoteRuntimeConfig &remote);

} // namespace SmartDrone::core::application
