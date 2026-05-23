#include "adapters/camera/camera_provider_factory.h"
#include "adapters/camera/uvc_stereo_camera.h"

namespace SmartDrone::Adapters::Camera {

std::unique_ptr<SmartDrone::Core::Ports::ICameraProvider> CreateCameraProvider()
{
    return std::make_unique<UvcStereoCamera>();
}

std::string_view CompiledCameraProviderName()
{
    return "uvc_stereo_opencv";
}

bool CompiledCameraProviderUsesPackedStereo()
{
    return true;
}

} // namespace SmartDrone::Adapters::Camera
