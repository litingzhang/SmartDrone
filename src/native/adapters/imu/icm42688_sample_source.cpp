#include "adapters/imu/icm42688_sample_source.h"

#include <cstdint>
#include <memory>
#include <utility>

#include "adapters/imu/icm42688/icm42688_imu.h"
#include "platform/linux/gpio/drdy_gpio.h"
#include "platform/linux/spi/spi_dev.h"

namespace SmartDrone::Adapters::Imu {

namespace {

using ImuSampleReadStatus = SmartDrone::Core::Ports::ImuSampleReadStatus;

} // namespace

class Icm42688SampleSource::Impl final {
  public:
    explicit Impl(Icm42688SampleSourceConfig config)
        : m_config(std::move(config))
    {
    }

    bool Start()
    {
        if (m_started) {
            return true;
        }
        if (m_failed) {
            return false;
        }
        if (!OpenSpi()) {
            return false;
        }
        BeginConfigure();
        m_started = true;
        return true;
    }

    bool EnsureOpen()
    {
        if (!Start()) {
            return false;
        }
        if (m_configuring) {
            StepConfiguration();
        }
        return m_open;
    }

    void Stop()
    {
        m_started = false;
        m_configuring = false;
        m_open = false;
        m_scale = {};
        m_spi.reset();
        m_drdy.reset();
    }

    ImuSampleReadStatus ReadSample(ImuSample &sample)
    {
        if (!EnsureOpen()) {
            return m_failed ? ImuSampleReadStatus::Failed
                            : ImuSampleReadStatus::Pending;
        }
        if (!m_drdy) {
            return ImuSampleReadStatus::Pending;
        }
        int64_t timestampNs = 0;
        if (!m_drdy->ReadReadyTimestamp(timestampNs)) {
            return ImuSampleReadStatus::Pending;
        }
        return ReadReadySample(timestampNs, sample);
    }

    ImuScale Scale() const
    {
        return m_scale;
    }

    bool Failed() const
    {
        return m_failed;
    }

  private:
    bool OpenSpi()
    {
        m_spi.reset(new SpiDev(m_config.spiDev));
        if (m_spi->Open(m_config.spiSpeed, m_config.spiMode,
                        m_config.spiBits)) {
            return true;
        }
        return Fail();
    }

    void BeginConfigure()
    {
        m_configSequencer.Reset(m_config.imuHz, m_config.accelFsG,
                                m_config.gyroFsDps);
        m_configuring = true;
    }

    void StepConfiguration()
    {
        ImuScale scale{};
        const auto status = m_configSequencer.Step(*m_spi, scale);
        if (status == Icm42688ConfigSequencer::Status::Pending) {
            return;
        }
        if (status == Icm42688ConfigSequencer::Status::Failed) {
            Fail();
            return;
        }
        FinishConfiguration(scale);
    }

    void FinishConfiguration(const ImuScale &scale)
    {
        m_scale = scale;
        if (OpenDrdy()) {
            m_configuring = false;
            m_open = true;
        }
    }

    bool OpenDrdy()
    {
        m_drdy.reset(new DrdyGpio());
        if (!m_drdy->Open(m_config.gpiochip, m_config.drdyLine)) {
            return Fail();
        }
        std::uint8_t status = 0;
        m_spi->ReadReg(REG_INT_STATUS, status);
        return true;
    }

    ImuSampleReadStatus ReadReadySample(std::int64_t timestampNs,
                                        ImuSample &sample)
    {
        sample.tNs = timestampNs;
        std::uint8_t status = 0;
        m_spi->ReadReg(REG_INT_STATUS, status);
        if (!m_spi->ReadRegs(m_config.imuStartReg, m_raw12,
                             sizeof(m_raw12))) {
            return ImuSampleReadStatus::Failed;
        }
        ConvertRaw12AccelGyroToSi(m_raw12, m_scale, sample);
        return ImuSampleReadStatus::Ready;
    }

    bool Fail()
    {
        m_failed = true;
        Stop();
        return false;
    }

    Icm42688SampleSourceConfig m_config;
    std::unique_ptr<SpiDev> m_spi;
    std::unique_ptr<DrdyGpio> m_drdy;
    Icm42688ConfigSequencer m_configSequencer;
    ImuScale m_scale{};
    std::uint8_t m_raw12[12]{};
    bool m_started{false};
    bool m_failed{false};
    bool m_configuring{false};
    bool m_open{false};
};

Icm42688SampleSource::Icm42688SampleSource(
    Icm42688SampleSourceConfig config)
    : m_impl(new Impl(std::move(config)))
{
}

Icm42688SampleSource::~Icm42688SampleSource() = default;

bool Icm42688SampleSource::Start()
{
    return m_impl->Start();
}

bool Icm42688SampleSource::EnsureOpen()
{
    return m_impl->EnsureOpen();
}

void Icm42688SampleSource::Stop()
{
    m_impl->Stop();
}

ImuSampleReadStatus Icm42688SampleSource::ReadSample(ImuSample &sample)
{
    return m_impl->ReadSample(sample);
}

ImuScale Icm42688SampleSource::Scale() const
{
    return m_impl->Scale();
}

bool Icm42688SampleSource::Failed() const
{
    return m_impl->Failed();
}

} // namespace SmartDrone::Adapters::Imu
