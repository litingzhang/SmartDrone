#pragma once

#include <memory>
#include <string_view>

#include "core/ports/camera_provider.h"
#include "core/ports/camera_runtime_overrides.h"
#include "core/ports/external_pose_source.h"
#include "core/ports/measurement_clock.h"

namespace SmartDrone::Adapters::Camera {

std::unique_ptr<SmartDrone::Core::Ports::ICameraProvider>
CreateCameraProvider(
    const std::shared_ptr<SmartDrone::Core::Ports::IMeasurementClock> &clock);
std::shared_ptr<SmartDrone::Core::Ports::IMeasurementClock>
CreateMeasurementClock();
std::shared_ptr<SmartDrone::Core::Ports::IExternalPoseSource>
CreateExternalPoseSource(
    const std::shared_ptr<SmartDrone::Core::Ports::IMeasurementClock> &clock);
SmartDrone::Core::Ports::CameraRuntimeOverrides
LoadCameraProviderRuntimeOverrides();
std::string_view CompiledCameraProviderName();
bool CompiledCameraProviderUsesPackedStereo();

} // namespace SmartDrone::Adapters::Camera
