#pragma once

#include <memory>

#include "core/application/runtime/discovery_beacon_runtime.h"
#include "core/application/runtime/epg_dfx_snapshot.h"
#include "core/application/runtime/epg_redeploy_coordinator.h"
#include "core/application/runtime/system_runtime_step_services.h"
#include "core/application/runtime/udp_command_runtime.h"
#include "core/application/epg/epg_registry.h"

namespace smartdrone::core::application {

struct SystemRuntimeTaskFactoryDeps {
    std::shared_ptr<SystemRuntimeStepServices> stepServices;
    std::shared_ptr<UdpCommandRuntime> commandRuntime;
    std::shared_ptr<DiscoveryBeaconRuntime> discoveryRuntime;
    std::shared_ptr<EpgGraphRef> graphRef;
    std::shared_ptr<EpgRedeployCoordinator> redeploy;
};

EpgTaskFactoryResolver MakeSystemRuntimeTaskFactoryResolver(
    SystemRuntimeTaskFactoryDeps deps);

} // namespace smartdrone::core::application
