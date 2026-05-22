#pragma once

#include <memory>

#include "core/ports/imu_sample_source.h"

namespace SmartDrone::core::application {

struct MainRuntimeAliases;

std::unique_ptr<SmartDrone::core::ports::IImuSampleSource>
CreateImuSampleSource(const MainRuntimeAliases &aliases);

} // namespace SmartDrone::core::application
