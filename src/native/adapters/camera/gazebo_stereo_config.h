#pragma once

#include <string>

#include "adapters/camera/gazebo_stereo_types.h"

namespace SmartDrone::Adapters::Camera {

struct GazeboStereoConfigLoadResult {
    bool ok{false};
    GazeboStereoConfig config;
    std::string error;
};

GazeboStereoConfigLoadResult LoadGazeboStereoConfig(
    const std::string &configPath);
GazeboImageFaultConfig LoadGazeboImageFaultConfigFromEnvironment(
    GazeboImageFaultConfig defaults = {});
bool TryLoadGazeboImageFaultState(const std::string &statePath,
                                  std::uint64_t currentGeneration,
                                  GazeboImageFaultConfig &out);

} // namespace SmartDrone::Adapters::Camera
