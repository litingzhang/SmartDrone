#include "adapters/camera/libcamera_stereo_camera.h"
#include "core/application/sensors/camera_runtime_provider.h"

namespace smartdrone::core::application {

std::unique_ptr<smartdrone::core::ports::ICameraProvider> CreateCameraProvider()
{
    return std::make_unique<smartdrone::adapters::camera::LibcameraStereoCamera>();
}

std::string_view CompiledCameraProviderName()
{
    return "libcamera_stereo_ov9281";
}

bool CompiledCameraProviderUsesPackedStereo()
{
    return false;
}

} // namespace smartdrone::core::application
