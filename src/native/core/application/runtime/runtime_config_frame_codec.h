#pragma once

#include <string>

#include "common/tlv/tlv_parser.h"
#include "core/application/config/runtime_app_types.h"

namespace SmartDrone::Core::Application {

bool RuntimeConfigPayloadLengthValid(std::uint16_t len);
RemoteRuntimeConfig ParseRuntimeConfigFrame(const TlvFrame &frame,
                                            const UnifiedConfig &currentCfg);
void ApplyConfigPeerIp(RemoteRuntimeConfig &remote,
                       const std::string &peerIp);
std::string BuildRuntimeConfigAckMessage(const std::string &message,
                                         const RemoteRuntimeConfig &remote);

} // namespace SmartDrone::Core::Application
