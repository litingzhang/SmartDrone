#include "adapters/camera/camera_provider_factory.h"
#include "adapters/camera/libcamera_stereo_camera.h"

namespace SmartDrone::Adapters::Camera {

std::unique_ptr<SmartDrone::Core::Ports::ICameraProvider> CreateCameraProvider()
{
    return std::make_unique<LibcameraStereoCamera>();
}

std::string_view CompiledCameraProviderName()
{
    return "libcamera_stereo_ov9281";
}

bool CompiledCameraProviderUsesPackedStereo()
{
    return false;
}

} // namespace SmartDrone::Adapters::Camera
