#include "core/application/sensors/imu_sample_source_provider.h"

#include <memory>

#include "adapters/imu/icm42688_sample_source.h"
#include "core/application/config/runtime_app_types.h"

namespace {

SmartDrone::Adapters::Imu::Icm42688SampleSourceConfig
BuildIcm42688SampleSourceConfig(
    const SmartDrone::Core::Application::MainRuntimeAliases &aliases)
{
    SmartDrone::Adapters::Imu::Icm42688SampleSourceConfig config{};
    config.spiDev = aliases.spiDev;
    config.spiSpeed = aliases.spiSpeed;
    config.spiMode = aliases.spiMode;
    config.spiBits = aliases.spiBits;
    config.gpiochip = aliases.gpiochip;
    config.drdyLine = aliases.drdyLine;
    config.imuHz = aliases.imuHz;
    config.accelFsG = aliases.accelFsG;
    config.gyroFsDps = aliases.gyroFsDps;
    config.imuStartReg = aliases.imuStartReg;
    return config;
}

} // namespace

namespace SmartDrone::Core::Application {

std::unique_ptr<SmartDrone::Core::Ports::IImuSampleSource>
CreateImuSampleSource(const MainRuntimeAliases &aliases)
{
    return std::make_unique<SmartDrone::Adapters::Imu::Icm42688SampleSource>(
        BuildIcm42688SampleSourceConfig(aliases));
}

} // namespace SmartDrone::Core::Application
