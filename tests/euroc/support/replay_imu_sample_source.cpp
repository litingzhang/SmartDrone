#include "core/application/sensors/imu_sample_source_provider.h"

#include <memory>

#include "core/application/config/runtime_app_types.h"
#include "core/ports/imu_sample_source.h"

namespace {

class ReplayNoopImuSampleSource final
    : public SmartDrone::Core::Ports::IImuSampleSource {
  public:
    bool Start() override
    {
        return true;
    }

    bool EnsureOpen() override
    {
        return true;
    }

    void Stop() override
    {
    }

    SmartDrone::Core::Ports::ImuSampleReadStatus ReadSample(ImuSample &)
        override
    {
        return SmartDrone::Core::Ports::ImuSampleReadStatus::Pending;
    }

    ImuScale Scale() const override
    {
        return {};
    }

    bool Failed() const override
    {
        return false;
    }
};

} // namespace

namespace SmartDrone::Core::Application {

std::unique_ptr<SmartDrone::Core::Ports::IImuSampleSource>
CreateImuSampleSource(const MainRuntimeAliases &)
{
    return std::make_unique<ReplayNoopImuSampleSource>();
}

} // namespace SmartDrone::Core::Application
