#include "core/application/sensors/imu_sample_source_provider.h"

#include "adapters/imu/icm42688_sample_source.h"

namespace SmartDrone::core::application {

std::unique_ptr<SmartDrone::core::ports::IImuSampleSource>
CreateImuSampleSource(const MainRuntimeAliases &aliases)
{
    return std::make_unique<SmartDrone::adapters::imu::Icm42688SampleSource>(
        aliases);
}

} // namespace SmartDrone::core::application
