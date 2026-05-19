#include "core/application/session/sensor_runtime_helpers.h"

#include <atomic>
#include <cstdint>
#include <iostream>
#include <memory>

#include "adapters/imu/icm42688/icm42688_imu.h"
#include "common/time_utils.h"
#include "platform/linux/gpio/drdy_gpio.h"
#include "platform/linux/spi/spi_dev.h"

namespace smartdrone::core::application {

class ImuSensorPoller::Impl final {
  public:
    Impl(const MainRuntimeAliases &aliases, ImuThreadState &state) : m_aliases(aliases), m_state(state) {}

    bool Start()
    {
        if (m_started) {
            return true;
        }
        if (m_failed) {
            return false;
        }
        SetScale(ImuScale{});
        if (!OpenSpi()) {
            return false;
        }
        BeginConfigure();
        m_started = true;
        return true;
    }

    void Stop()
    {
        m_state.imuOk.store(false, std::memory_order_relaxed);
        m_started = false;
        m_configuring = false;
        m_spi.reset();
        m_drdy.reset();
    }

    void Step()
    {
        if (!m_started || !m_spi) {
            return;
        }
        if (m_configuring) {
            StepConfiguration();
            return;
        }
        if (!m_drdy) {
            return;
        }
        for (int i = 0; i < kMaxSamplesPerStep; ++i) {
            int64_t tNs = 0;
            if (!m_drdy->WaitTs(0, tNs)) {
                return;
            }
            ReadSample(tNs);
        }
    }

    bool Failed() const { return m_failed; }

  private:
    static constexpr int kMaxSamplesPerStep = 8;

    bool OpenSpi()
    {
        m_spi.reset(new SpiDev(m_aliases.spiDev));
        if (m_spi->Open(m_aliases.spiSpeed, m_aliases.spiMode, m_aliases.spiBits)) {
            return true;
        }
        return Fail();
    }

    void BeginConfigure()
    {
        m_configSequencer.Reset(m_aliases.imuHz, m_aliases.accelFsG, m_aliases.gyroFsDps);
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
        SetScale(scale);
        if (OpenDrdy()) {
            m_configuring = false;
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
        m_state.imuOk.store(true, std::memory_order_relaxed);
        return true;
    }

    bool Fail()
    {
        m_failed = true;
        Stop();
        return false;
    }

    void SetScale(const ImuScale &scale)
    {
        m_state.accelLsbPerG.store(scale.accelLsbPerG, std::memory_order_relaxed);
        m_state.gyroLsbPerDps.store(scale.gyroLsbPerDps, std::memory_order_relaxed);
    }

    bool AcceptTimestamp(int64_t tNs)
    {
        if (m_lastAcceptedTsNs == 0 || tNs > m_lastAcceptedTsNs) {
            return true;
        }
        m_state.imuDrop.fetch_add(1, std::memory_order_relaxed);
        LogNonMonotonicTimestamp(tNs);
        return false;
    }

    void LogNonMonotonicTimestamp(int64_t tNs)
    {
        const uint64_t nowUs = MonoTimeUs();
        if (m_lastNonMonotonicLogUs != 0 && nowUs - m_lastNonMonotonicLogUs < 1000000ULL) {
            return;
        }
        std::cerr << "[imu] dropped non-monotonic DRDY timestamp"
                  << " prev_ns=" << m_lastAcceptedTsNs << " cur_ns=" << tNs << "\n";
        m_lastNonMonotonicLogUs = nowUs;
    }

    ImuScale CurrentScale() const
    {
        ImuScale scale{};
        scale.accelLsbPerG = m_state.accelLsbPerG.load(std::memory_order_relaxed);
        scale.gyroLsbPerDps = m_state.gyroLsbPerDps.load(std::memory_order_relaxed);
        return scale;
    }

    void ReadSample(int64_t tNs)
    {
        if (!AcceptTimestamp(tNs)) {
            return;
        }
        ImuSample sample{};
        sample.tNs = tNs;
        std::uint8_t status = 0;
        m_spi->ReadReg(REG_INT_STATUS, status);
        if (!m_spi->ReadRegs(m_aliases.imuStartReg, m_raw12, sizeof(m_raw12))) {
            m_state.imuDrop.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        ConvertRaw12AccelGyroToSi(m_raw12, CurrentScale(), sample);
        m_state.imuBuffer.Push(sample);
        m_lastAcceptedTsNs = tNs;
        m_state.imuCnt.fetch_add(1, std::memory_order_relaxed);
    }

    const MainRuntimeAliases &m_aliases;
    ImuThreadState &m_state;
    std::unique_ptr<SpiDev> m_spi;
    std::unique_ptr<DrdyGpio> m_drdy;
    Icm42688ConfigSequencer m_configSequencer;
    std::uint8_t m_raw12[12]{};
    int64_t m_lastAcceptedTsNs{0};
    uint64_t m_lastNonMonotonicLogUs{0};
    bool m_started{false};
    bool m_failed{false};
    bool m_configuring{false};
};

ImuSensorPoller::ImuSensorPoller(const MainRuntimeAliases &aliases, ImuThreadState &state)
    : m_impl(new Impl(aliases, state))
{
}

ImuSensorPoller::~ImuSensorPoller() = default;

bool ImuSensorPoller::Start() { return m_impl->Start(); }

void ImuSensorPoller::Stop() { m_impl->Stop(); }

void ImuSensorPoller::Step() { m_impl->Step(); }

bool ImuSensorPoller::Failed() const { return m_impl->Failed(); }

} // namespace smartdrone::core::application
