#include "core/application/sensors/imu_sample_source_provider.h"

#include "adapters/imu/icm42688_sample_source.h"

namespace smartdrone::core::application {

std::unique_ptr<smartdrone::core::ports::IImuSampleSource>
CreateImuSampleSource(const MainRuntimeAliases &aliases)
{
    return std::make_unique<smartdrone::adapters::imu::Icm42688SampleSource>(
        aliases);
}

} // namespace smartdrone::core::application
