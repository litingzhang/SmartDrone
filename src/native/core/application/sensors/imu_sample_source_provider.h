#pragma once

#include <memory>

#include "core/ports/imu_sample_source.h"

namespace smartdrone::core::application {

struct MainRuntimeAliases;

std::unique_ptr<smartdrone::core::ports::IImuSampleSource>
CreateImuSampleSource(const MainRuntimeAliases &aliases);

} // namespace smartdrone::core::application
