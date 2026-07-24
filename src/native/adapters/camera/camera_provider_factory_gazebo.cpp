#include "adapters/camera/camera_provider_factory.h"

#include "adapters/camera/gazebo_stereo_camera.h"
#include "adapters/camera/gazebo_stereo_config.h"
#include "adapters/simulation/gazebo_measurement_clock.h"
#include "adapters/simulation/gazebo_truth_pose_source.h"
#include "common/environment.h"

namespace SmartDrone::Adapters::Camera {

std::unique_ptr<SmartDrone::Core::Ports::ICameraProvider>
CreateCameraProvider(
    const std::shared_ptr<SmartDrone::Core::Ports::IMeasurementClock> &clock)
{
    auto gazeboClock = std::dynamic_pointer_cast<
        SmartDrone::Adapters::Simulation::GazeboMeasurementClock>(clock);
    if (!gazeboClock) {
        gazeboClock = std::make_shared<
            SmartDrone::Adapters::Simulation::GazeboMeasurementClock>();
    }
    return std::make_unique<GazeboStereoCamera>(std::move(gazeboClock));
}

std::shared_ptr<SmartDrone::Core::Ports::IMeasurementClock>
CreateMeasurementClock()
{
    return std::make_shared<
        SmartDrone::Adapters::Simulation::GazeboMeasurementClock>();
}

std::shared_ptr<SmartDrone::Core::Ports::IExternalPoseSource>
CreateExternalPoseSource(
    const std::shared_ptr<SmartDrone::Core::Ports::IMeasurementClock> &clock)
{
    if (SmartDrone::Common::EnvStringValue("SMART_DRONE_SIM_PROFILE", "") !=
        "truth") {
        return {};
    }
    auto gazeboClock = std::dynamic_pointer_cast<
        SmartDrone::Adapters::Simulation::GazeboMeasurementClock>(clock);
    if (!gazeboClock) {
        return {};
    }
    auto source = std::make_shared<
        SmartDrone::Adapters::Simulation::GazeboTruthPoseSource>(
        std::move(gazeboClock));
    return source->Open() ? source : nullptr;
}

SmartDrone::Core::Ports::CameraRuntimeOverrides
LoadCameraProviderRuntimeOverrides()
{
    const std::string path = SmartDrone::Common::EnvStringValue(
        "SMART_DRONE_SIM_CONFIG", "");
    const GazeboStereoConfigLoadResult loaded = LoadGazeboStereoConfig(path);
    if (!loaded.ok) {
        return {};
    }
    return {loaded.config.cameraWidth, loaded.config.cameraHeight,
            loaded.config.cameraFps, loaded.config.calibrationPath};
}

std::string_view CompiledCameraProviderName()
{
    return "gz_stereo";
}

bool CompiledCameraProviderUsesPackedStereo()
{
    return false;
}

} // namespace SmartDrone::Adapters::Camera
