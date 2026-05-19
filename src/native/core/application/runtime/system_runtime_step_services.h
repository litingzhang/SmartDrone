#pragma once

#include <functional>
#include <utility>

namespace smartdrone::core::application {

using RuntimeGraphStepFn = std::function<void()>;

struct SystemRuntimeStepServicesConfig {
    RuntimeGraphStepFn vehicleTelemetryRx;
    RuntimeGraphStepFn setpointStream;
    RuntimeGraphStepFn manualControl;
    RuntimeGraphStepFn forceRestart;
    RuntimeGraphStepFn sessionSupervisor;
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

  private:
    void Call(const RuntimeGraphStepFn &step);

    SystemRuntimeStepServicesConfig m_config;
};

} // namespace smartdrone::core::application
