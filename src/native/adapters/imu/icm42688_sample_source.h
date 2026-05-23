#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "core/ports/imu_sample_source.h"

namespace SmartDrone::Adapters::Imu {

struct Icm42688SampleSourceConfig {
    std::string spiDev;
    std::uint32_t spiSpeed{0};
    std::uint8_t spiMode{0};
    std::uint8_t spiBits{0};
    std::string gpiochip;
    unsigned drdyLine{0};
    int imuHz{0};
    int accelFsG{0};
    int gyroFsDps{0};
    std::uint8_t imuStartReg{0};
};

class Icm42688SampleSource final
    : public SmartDrone::Core::Ports::IImuSampleSource {
  public:
    explicit Icm42688SampleSource(Icm42688SampleSourceConfig config);
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
