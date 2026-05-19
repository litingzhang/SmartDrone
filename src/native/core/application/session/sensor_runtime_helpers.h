#pragma once

#include <memory>
#include <string_view>

#include "core/application/config/runtime_app_types.h"
#include "core/application/session/imu_runtime_state.h"
#include "core/ports/camera_provider.h"

namespace smartdrone::core::application {

class ImuSensorPoller {
  public:
    ImuSensorPoller(const MainRuntimeAliases &aliases, ImuThreadState &state);
    ~ImuSensorPoller();

    bool Start();
    void Stop();
    void Step();
    bool Failed() const;

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

std::unique_ptr<smartdrone::core::ports::ICameraProvider> CreateCameraProvider();
std::string_view CompiledCameraProviderName();
bool CompiledCameraProviderUsesPackedStereo();

} // namespace smartdrone::core::application
