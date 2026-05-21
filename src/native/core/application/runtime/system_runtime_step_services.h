#pragma once

#include <functional>
#include <utility>

#include "core/application/runtime/epg_redeploy_coordinator.h"

namespace smartdrone::core::application {

using RuntimeGraphStepFn = std::function<void()>;
using RuntimeGraphRedeployStepFn =
    std::function<void(EpgRedeployCoordinator &)>;

struct SystemRuntimeStepServicesConfig {
    RuntimeGraphStepFn vehicleTelemetryRx;
    RuntimeGraphStepFn setpointStream;
    RuntimeGraphStepFn manualControl;
    RuntimeGraphStepFn forceRestart;
    RuntimeGraphStepFn sessionSupervisor;
    RuntimeGraphRedeployStepFn epgRedeploy;
};

class SystemRuntimeStepServices final {
  public:
    explicit SystemRuntimeStepServices(SystemRuntimeStepServicesConfig config);

    bool Valid() const;
    void OnVehicleTelemetryRxGraphTick();
    void OnSetpointStreamGraphTick();
    void OnManualControlGraphTick();
    void OnForceRestartGraphTick();
    void OnSessionSupervisorGraphTick();
    void OnEpgRedeployGraphTick(EpgRedeployCoordinator &coordinator);

  private:
    void Call(const RuntimeGraphStepFn &step);

    SystemRuntimeStepServicesConfig m_config;
};

} // namespace smartdrone::core::application
