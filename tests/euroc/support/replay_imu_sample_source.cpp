#include "core/application/sensors/imu_sample_source_provider.h"

#include <memory>
#include <vector>

#include "core/application/config/runtime_app_types.h"
#include "core/ports/imu_sample_source.h"
#include "support/replay_dataset.h"

namespace {

const std::vector<SmartDrone::Tests::ReplayImuSample> *g_replayImuSamples{
    nullptr};

class ReplayImuSampleSource final
    : public SmartDrone::Core::Ports::IImuSampleSource {
  public:
    bool Start() override
    {
        m_cursor = 0;
        return true;
    }

    bool EnsureOpen() override
    {
        return true;
    }

    void Stop() override
    {
    }

    SmartDrone::Core::Ports::ImuSampleReadStatus ReadSample(ImuSample &sample)
        override
    {
        if (g_replayImuSamples == nullptr ||
            m_cursor >= g_replayImuSamples->size()) {
            return SmartDrone::Core::Ports::ImuSampleReadStatus::Pending;
        }
        const auto &source = (*g_replayImuSamples)[m_cursor++];
        sample.tNs = source.timestampNs;
        sample.ax = source.ax;
        sample.ay = source.ay;
        sample.az = source.az;
        sample.gx = source.gx;
        sample.gy = source.gy;
        sample.gz = source.gz;
        return SmartDrone::Core::Ports::ImuSampleReadStatus::Ready;
    }

    ImuScale Scale() const override
    {
        return {};
    }

    bool Failed() const override
    {
        return false;
    }

  private:
    std::size_t m_cursor{0};
};

} // namespace

namespace SmartDrone::Tests {

void SetReplayImuSampleSourceDataset(const ReplayDataset &dataset)
{
    g_replayImuSamples = &dataset.ImuSamples();
}

void ClearReplayImuSampleSourceDataset()
{
    g_replayImuSamples = nullptr;
}

ReplayImuSampleSourceScope::ReplayImuSampleSourceScope(
    const ReplayDataset &dataset)
{
    SetReplayImuSampleSourceDataset(dataset);
}

ReplayImuSampleSourceScope::~ReplayImuSampleSourceScope()
{
    ClearReplayImuSampleSourceDataset();
}

} // namespace SmartDrone::Tests

namespace SmartDrone::Core::Application {

std::unique_ptr<SmartDrone::Core::Ports::IImuSampleSource>
CreateImuSampleSource(const MainRuntimeAliases &)
{
    return std::make_unique<ReplayImuSampleSource>();
}

} // namespace SmartDrone::Core::Application
