#include "core/application/session/calib/calib_imu_sample_port.h"

#include "core/application/config/runtime_app_types.h"
#include "core/application/sensors/imu_sample_source_provider.h"

namespace SmartDrone::Core::Application {

namespace {

using ImuSampleReadStatus = SmartDrone::Core::Ports::ImuSampleReadStatus;

} // namespace

class CalibImuSamplePort::Impl final {
  public:
    explicit Impl(const MainRuntimeAliases &aliases)
        : m_sampleSource(CreateImuSampleSource(aliases))
    {
    }

    ~Impl()
    {
        Stop();
    }

    CalibImuSamplePortResult ReadSample()
    {
        if (!EnsureOpened()) {
            return {CalibImuSamplePortStatus::Pending, {}};
        }
        ImuSample sample{};
        const ImuSampleReadStatus status = m_sampleSource->ReadSample(sample);
        return BuildResult(status, sample);
    }

    void Stop()
    {
        if (m_sampleSource) {
            m_sampleSource->Stop();
        }
        m_opened = false;
    }

  private:
    bool EnsureOpened()
    {
        if (m_opened || m_openFailed) {
            return m_opened;
        }
        if (!m_sampleSource->EnsureOpen()) {
            m_openFailed = m_sampleSource->Failed();
            return false;
        }
        m_opened = true;
        return true;
    }

    static CalibImuSamplePortResult BuildResult(
        ImuSampleReadStatus status,
        const ImuSample &sample)
    {
        if (status == ImuSampleReadStatus::Ready) {
            return {CalibImuSamplePortStatus::Ready, sample};
        }
        if (status == ImuSampleReadStatus::Failed) {
            return {CalibImuSamplePortStatus::Failed, {}};
        }
        return {CalibImuSamplePortStatus::Pending, {}};
    }

    std::unique_ptr<SmartDrone::Core::Ports::IImuSampleSource> m_sampleSource;
    bool m_opened{false};
    bool m_openFailed{false};
};

CalibImuSamplePort::CalibImuSamplePort(const MainRuntimeAliases &aliases)
    : m_impl(new Impl(aliases))
{
}

CalibImuSamplePort::~CalibImuSamplePort() = default;

CalibImuSamplePortResult CalibImuSamplePort::ReadSample()
{
    return m_impl->ReadSample();
}

void CalibImuSamplePort::Stop()
{
    m_impl->Stop();
}

} // namespace SmartDrone::Core::Application
