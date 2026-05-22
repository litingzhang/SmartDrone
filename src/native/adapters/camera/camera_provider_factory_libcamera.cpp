#include "adapters/camera/camera_provider_factory.h"
#include "adapters/camera/libcamera_stereo_camera.h"

namespace SmartDrone::adapters::camera {

std::unique_ptr<SmartDrone::core::ports::ICameraProvider> CreateCameraProvider()
{
    return std::make_unique<LibcameraStereoCamera>();
}

SmartDrone::core::ports::CameraOpenConfig MakeCameraOpenConfig(
    const SmartDrone::core::application::MainRuntimeAliases &aliases)
{
    SmartDrone::core::ports::CameraOpenConfig config{};
    config.width = aliases.width;
    config.height = aliases.height;
    config.fps = aliases.fps;
    config.leftCameraIndex = aliases.leftCamIndex;
    config.rightCameraIndex = aliases.rightCamIndex;
    config.exposureUs = aliases.exposureUs;
    config.pairWindowMs = aliases.pairMs;
    config.keepWindowMs = aliases.keepMs;
    config.pairQueue = aliases.pairQueue;
    config.uvcDeviceIndex = aliases.uvcDeviceIndex;
    config.uvcEyeWidth = aliases.uvcEyeWidth;
    config.uvcEyeHeight = aliases.uvcEyeHeight;
    config.autoExposureDisabled = aliases.aeDisable;
    config.requestY8 = aliases.requestY8;
    config.r16Normalize = aliases.r16Norm;
    config.uvcPackedStereo = aliases.uvcPackedStereo;
    config.uvcSwapEyes = aliases.uvcSwapEyes;
    config.gain = aliases.gain;
    return config;
}

std::string_view CompiledCameraProviderName()
{
    return "libcamera_stereo_ov9281";
}

bool CompiledCameraProviderUsesPackedStereo()
{
    return false;
}

} // namespace SmartDrone::adapters::camera
