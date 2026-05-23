#pragma once

#include <string>

namespace SmartDrone::Core::Application {

std::string NormalizeOrbAcceleration(std::string acceleration);
std::string NormalizeOrbAccelerationOrCpu(std::string acceleration);
void ApplyOrbAccelerationEnvironment(const std::string &acceleration);
void ResetOrbAccelerationEnvironment();

} // namespace SmartDrone::Core::Application
