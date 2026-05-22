#include "core/application/sensors/imu_sample_source_provider.h"

#include "adapters/imu/icm42688_sample_source.h"

namespace SmartDrone::Core::Application {

std::unique_ptr<SmartDrone::Core::Ports::IImuSampleSource>
CreateImuSampleSource(const MainRuntimeAliases &aliases)
{
    return std::make_unique<SmartDrone::Adapters::Imu::Icm42688SampleSource>(
        aliases);
}

} // namespace SmartDrone::Core::Application
