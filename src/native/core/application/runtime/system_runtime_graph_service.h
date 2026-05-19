#pragma once

#include <functional>
#include <memory>

#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/runtime_aliases.h"
#include "core/application/runtime/system_runtime_step_services.h"
#include "core/application/runtime/udp_command_runtime.h"

namespace smartdrone::core::application {

struct SystemRuntimeGraphConfig {
    MainRuntimeAliases aliases;
    int discoveryPort{0};
    RuntimeGraphStepFn stepVehicleTelemetryRx;
    RuntimeGraphStepFn stepSetpointStream;
    RuntimeGraphStepFn stepManualControl;
    RuntimeGraphStepFn stepForceRestart;
    RuntimeGraphStepFn stepSessionSupervisor;
    UdpCommandRuntimeConfig commandRuntime;
    BuildCapabilitiesPayloadFn buildCapabilitiesPayload;
    BuildConfigPayloadFn buildConfigPayload;
    PeerToIpStringFn peerToIpString;
};

class SystemRuntimeGraph final {
  public:
    explicit SystemRuntimeGraph(SystemRuntimeGraphConfig config);
    ~SystemRuntimeGraph();

    bool Start();
    void Stop();

  private:
    class Impl;

    std::unique_ptr<Impl> m_impl;
};

} // namespace smartdrone::core::application
