#include "adapters/imu/icm42688_sample_source.h"

#include <cstdint>
#include <memory>

#include "adapters/imu/icm42688/icm42688_imu.h"
#include "core/application/config/runtime_app_types.h"
#include "platform/linux/gpio/drdy_gpio.h"
#include "platform/linux/spi/spi_dev.h"

namespace smartdrone::adapters::imu {

namespace {

using MainRuntimeAliases =
    smartdrone::core::application::MainRuntimeAliases;
using ImuSampleReadStatus = smartdrone::core::ports::ImuSampleReadStatus;

} // namespace

class Icm42688SampleSource::Impl final {
  public:
    explicit Impl(const MainRuntimeAliases &aliases) : m_aliases(aliases)
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
        if (!m_drdy->WaitTs(0, timestampNs)) {
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
        m_spi.reset(new SpiDev(m_aliases.spiDev));
        if (m_spi->Open(m_aliases.spiSpeed, m_aliases.spiMode,
                        m_aliases.spiBits)) {
            return true;
        }
        return Fail();
    }

    void BeginConfigure()
    {
        m_configSequencer.Reset(m_aliases.imuHz, m_aliases.accelFsG,
                                m_aliases.gyroFsDps);
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
        if (!m_drdy->Open(m_aliases.gpiochip, m_aliases.drdyLine)) {
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
        if (!m_spi->ReadRegs(m_aliases.imuStartReg, m_raw12,
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

    const MainRuntimeAliases &m_aliases;
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
    const smartdrone::core::application::MainRuntimeAliases &aliases)
    : m_impl(new Impl(aliases))
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

} // namespace smartdrone::adapters::imu
