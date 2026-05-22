#pragma once

#include <memory>

#include "core/ports/imu_sample_source.h"

namespace SmartDrone::core::application {
struct MainRuntimeAliases;
} // namespace SmartDrone::core::application

namespace SmartDrone::adapters::imu {

class Icm42688SampleSource final
    : public SmartDrone::core::ports::IImuSampleSource {
  public:
    explicit Icm42688SampleSource(
        const SmartDrone::core::application::MainRuntimeAliases &aliases);
    ~Icm42688SampleSource() override;

    bool Start() override;
    bool EnsureOpen() override;
    void Stop() override;
    SmartDrone::core::ports::ImuSampleReadStatus ReadSample(
        ImuSample &sample) override;
    ImuScale Scale() const override;
    bool Failed() const override;

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::adapters::imu
