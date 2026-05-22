#pragma once

#include <memory>

#include "core/ports/imu_sample_source.h"

namespace SmartDrone::Core::Application {
struct MainRuntimeAliases;
} // namespace SmartDrone::Core::Application

namespace SmartDrone::Adapters::Imu {

class Icm42688SampleSource final
    : public SmartDrone::Core::Ports::IImuSampleSource {
  public:
    explicit Icm42688SampleSource(
        const SmartDrone::Core::Application::MainRuntimeAliases &aliases);
    ~Icm42688SampleSource() override;

    bool Start() override;
    bool EnsureOpen() override;
    void Stop() override;
    SmartDrone::Core::Ports::ImuSampleReadStatus ReadSample(
        ImuSample &sample) override;
    ImuScale Scale() const override;
    bool Failed() const override;

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Adapters::Imu
