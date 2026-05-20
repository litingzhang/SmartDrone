#include "adapters/camera/uvc_stereo_camera.h"
#include "core/application/sensors/camera_runtime_provider.h"

namespace smartdrone::core::application {

std::unique_ptr<smartdrone::core::ports::ICameraProvider> CreateCameraProvider()
{
    return std::make_unique<smartdrone::adapters::camera::UvcStereoCamera>();
}

std::string_view CompiledCameraProviderName()
{
    return "uvc_stereo_opencv";
}

bool CompiledCameraProviderUsesPackedStereo()
{
    return true;
}

} // namespace smartdrone::core::application
