#pragma once

#include <cstdint>
#include <memory>

#include "core/application/sensors/imu_runtime_state.h"

namespace SmartDrone::Core::Application {

struct MainRuntimeAliases;

enum class CalibImuSamplePortStatus : std::uint8_t {
    Ready,
    Pending,
    Failed,
};

struct CalibImuSamplePortResult {
    CalibImuSamplePortStatus status{CalibImuSamplePortStatus::Pending};
    ImuSample sample{};
};

class CalibImuSamplePort final {
  public:
    explicit CalibImuSamplePort(const MainRuntimeAliases &aliases);
    ~CalibImuSamplePort();

    CalibImuSamplePortResult ReadSample();
    void Stop();

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Core::Application
