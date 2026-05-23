#include "core/application/config/orb_acceleration_config.h"

#include <algorithm>
#include <cctype>
#include <utility>

#include "common/environment.h"

namespace SmartDrone::Core::Application {
namespace {

void SetOrbAccelerationEnvironment(const char *orbAccel,
                                   const char *vpiRemap)
{
    if (orbAccel[0] != '\0') {
        SmartDrone::Common::SetEnvVar("SMART_DRONE_ORB_ACCEL", orbAccel);
    } else {
        SmartDrone::Common::UnsetEnvVar("SMART_DRONE_ORB_ACCEL");
    }
    if (vpiRemap[0] != '\0') {
        SmartDrone::Common::SetEnvVar("SMART_DRONE_ORB_VPI_REMAP", vpiRemap);
    } else {
        SmartDrone::Common::UnsetEnvVar("SMART_DRONE_ORB_VPI_REMAP");
    }
    SmartDrone::Common::UnsetEnvVar("SMART_DRONE_ORB_CUDA_PYRAMID");
}

} // namespace

std::string NormalizeOrbAcceleration(std::string acceleration)
{
    std::transform(acceleration.begin(), acceleration.end(),
                   acceleration.begin(), [](unsigned char c) {
                       return static_cast<char>(std::tolower(c));
                   });
    if (acceleration == "cuda" || acceleration == "gpu" ||
        acceleration == "opencv_cuda" || acceleration == "opencv-cuda") {
        return "cuda";
    }
    if (acceleration == "vpi" || acceleration == "vpi_remap" ||
        acceleration == "vpi-remap" || acceleration == "vpi_cuda_remap" ||
        acceleration == "vpi-cuda-remap") {
        return "vpi-remap";
    }
    if (acceleration == "cpu" || acceleration == "off" ||
        acceleration.empty()) {
        return "cpu";
    }
    return acceleration;
}

std::string NormalizeOrbAccelerationOrCpu(std::string acceleration)
{
    acceleration = NormalizeOrbAcceleration(std::move(acceleration));
    if (acceleration == "cpu" || acceleration == "cuda" ||
        acceleration == "vpi-remap") {
        return acceleration;
    }
    return "cpu";
}

void ApplyOrbAccelerationEnvironment(const std::string &acceleration)
{
    const std::string normalized = NormalizeOrbAcceleration(acceleration);
    if (normalized == "cuda") {
        SetOrbAccelerationEnvironment("cuda", "");
        return;
    }
    if (normalized == "vpi-remap") {
        SetOrbAccelerationEnvironment("", "1");
        return;
    }
    ResetOrbAccelerationEnvironment();
}

void ResetOrbAccelerationEnvironment()
{
    SetOrbAccelerationEnvironment("", "");
}

} // namespace SmartDrone::Core::Application
