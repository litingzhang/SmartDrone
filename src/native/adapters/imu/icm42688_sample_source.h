#pragma once

#include <memory>

#include "core/ports/imu_sample_source.h"

namespace smartdrone::core::application {
struct MainRuntimeAliases;
} // namespace smartdrone::core::application

namespace smartdrone::adapters::imu {

class Icm42688SampleSource final
    : public smartdrone::core::ports::IImuSampleSource {
  public:
    explicit Icm42688SampleSource(
        const smartdrone::core::application::MainRuntimeAliases &aliases);
    ~Icm42688SampleSource() override;

    bool Start() override;
    bool EnsureOpen() override;
    void Stop() override;
    smartdrone::core::ports::ImuSampleReadStatus ReadSample(
        ImuSample &sample) override;
    ImuScale Scale() const override;
    bool Failed() const override;

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace smartdrone::adapters::imu
