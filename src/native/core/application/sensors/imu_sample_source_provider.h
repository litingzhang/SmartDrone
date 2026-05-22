#pragma once

#include <memory>

#include "core/ports/imu_sample_source.h"

namespace SmartDrone::Core::Application {

struct MainRuntimeAliases;

std::unique_ptr<SmartDrone::Core::Ports::IImuSampleSource>
CreateImuSampleSource(const MainRuntimeAliases &aliases);

} // namespace SmartDrone::Core::Application
