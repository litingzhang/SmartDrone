#include "core/application/sensors/imu_sensor_poller.h"

#include <atomic>
#include <cstdint>
#include <iostream>
#include <memory>

#include "common/time_utils.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/sensors/imu_sample_source_provider.h"
#include "core/application/sensors/imu_runtime_state.h"

namespace SmartDrone::Core::Application {

namespace {

using ImuSampleReadStatus = SmartDrone::Core::Ports::ImuSampleReadStatus;

} // namespace

class ImuSensorPoller::Impl final {
  public:
    Impl(const MainRuntimeAliases &aliases, ImuThreadState &state)
        : m_state(state), m_sampleSource(CreateImuSampleSource(aliases))
    {
    }

    bool Start()
    {
        if (m_sampleSource->Start()) {
            SetScale(m_sampleSource->Scale());
            return true;
        }
        return false;
    }

    void Stop()
    {
        m_state.imuOk.store(false, std::memory_order_relaxed);
        m_sampleSource->Stop();
    }

    void Step()
    {
        for (int i = 0; i < kMaxSamplesPerStep; ++i) {
            ImuSample sample{};
            const ImuSampleReadStatus status =
                m_sampleSource->ReadSample(sample);
            if (status == ImuSampleReadStatus::Pending) {
                UpdateScale();
                return;
            }
            if (status == ImuSampleReadStatus::Failed) {
                m_state.imuDrop.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            UpdateScale();
            HandleSample(sample);
        }
    }

    bool Failed() const
    {
        return m_sampleSource->Failed();
    }

  private:
    static constexpr int kMaxSamplesPerStep = 8;

    void SetScale(const ImuScale &scale)
    {
        m_state.accelLsbPerG.store(scale.accelLsbPerG,
                                   std::memory_order_relaxed);
        m_state.gyroLsbPerDps.store(scale.gyroLsbPerDps,
                                    std::memory_order_relaxed);
    }

    void UpdateScale()
    {
        const ImuScale scale = m_sampleSource->Scale();
        if (scale.accelLsbPerG <= 0.0f || scale.gyroLsbPerDps <= 0.0f) {
            return;
        }
        SetScale(scale);
        m_state.imuOk.store(true, std::memory_order_relaxed);
    }

    bool AcceptTimestamp(std::int64_t timestampNs)
    {
        if (m_lastAcceptedTsNs == 0 ||
            timestampNs > m_lastAcceptedTsNs) {
            return true;
        }
        m_state.imuDrop.fetch_add(1, std::memory_order_relaxed);
        LogNonMonotonicTimestamp(timestampNs);
        return false;
    }

    void LogNonMonotonicTimestamp(std::int64_t timestampNs)
    {
        const std::uint64_t nowUs = MonoTimeUs();
        if (m_lastNonMonotonicLogUs != 0 &&
            nowUs - m_lastNonMonotonicLogUs < 1000000ULL) {
            return;
        }
        std::cerr << "[imu] dropped non-monotonic DRDY timestamp"
                  << " prev_ns=" << m_lastAcceptedTsNs
                  << " cur_ns=" << timestampNs << "\n";
        m_lastNonMonotonicLogUs = nowUs;
    }

    void HandleSample(const ImuSample &sample)
    {
        if (!AcceptTimestamp(sample.tNs)) {
            return;
        }
        m_state.imuBuffer.Push(sample);
        m_lastAcceptedTsNs = sample.tNs;
        m_state.imuCnt.fetch_add(1, std::memory_order_relaxed);
    }

    ImuThreadState &m_state;
    std::unique_ptr<SmartDrone::Core::Ports::IImuSampleSource> m_sampleSource;
    std::int64_t m_lastAcceptedTsNs{0};
    std::uint64_t m_lastNonMonotonicLogUs{0};
};

ImuSensorPoller::ImuSensorPoller(const MainRuntimeAliases &aliases,
                                 ImuThreadState &state)
    : m_impl(new Impl(aliases, state))
{
}

ImuSensorPoller::~ImuSensorPoller() = default;

bool ImuSensorPoller::Start()
{
    return m_impl->Start();
}

void ImuSensorPoller::Stop()
{
    m_impl->Stop();
}

void ImuSensorPoller::Step()
{
    m_impl->Step();
}

bool ImuSensorPoller::Failed() const
{
    return m_impl->Failed();
}

} // namespace SmartDrone::Core::Application
