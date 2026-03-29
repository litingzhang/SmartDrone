#pragma once

#include <string_view>
#include <vector>

#include "core/domain/capabilities.hpp"

namespace smartdrone::core::application {

class ConfigRegistry {
public:
    static constexpr std::string_view kCameraExposureUs = "camera.exposure_us";
    static constexpr std::string_view kCameraGain = "camera.gain";
    static constexpr std::string_view kCameraPairWindowMs = "camera.pair_window_ms";
    static constexpr std::string_view kSlamInputFps = "slam.input_fps";
    static constexpr std::string_view kSlamPerceptionMode = "slam.perception_mode";
    static constexpr std::string_view kSlamOperationMode = "slam.operation_mode";
    static constexpr std::string_view kStreamUdpEnabled = "stream.udp_enabled";
    static constexpr std::string_view kStreamUdpIp = "stream.udp_ip";
    static constexpr std::string_view kStreamSendImage = "stream.send_image";
    static constexpr std::string_view kStreamSendFeature = "stream.send_feature";
    static constexpr std::string_view kStreamSendMap = "stream.send_map";

    static std::vector<domain::ConfigDescriptor> DefaultDescriptors()
    {
        return {
            Make(kCameraExposureUs, "Camera exposure time in microseconds", false, true, true),
            Make(kCameraGain, "Camera analog gain", false, true, true),
            Make(kCameraPairWindowMs, "Stereo pairing window in milliseconds", false, true, true),
            Make(kSlamInputFps, "Input frame rate delivered to the SLAM engine", true, false, false),
            Make(kSlamPerceptionMode, "SLAM perception mode such as stereo or stereo-imu", false, true, false),
            Make(kSlamOperationMode, "SLAM operating mode such as mapping or localization", true, false, false),
            Make(kStreamUdpEnabled, "Enable UDP preview and telemetry streaming", false, true, false),
            Make(kStreamUdpIp, "Destination IP for UDP preview streaming", false, true, false),
            Make(kStreamSendImage, "Enable image preview streaming", false, true, false),
            Make(kStreamSendFeature, "Enable tracked feature streaming", false, true, false),
            Make(kStreamSendMap, "Enable pose and map streaming", false, true, false),
        };
    }

private:
    static domain::ConfigDescriptor Make(std::string_view key,
                                         std::string_view description,
                                         bool hotReloadable,
                                         bool requiresPipelineRestart,
                                         bool requiresDeviceRestart)
    {
        domain::ConfigDescriptor out{};
        out.key = std::string(key);
        out.description = std::string(description);
        out.hotReloadable = hotReloadable;
        out.requiresPipelineRestart = requiresPipelineRestart;
        out.requiresDeviceRestart = requiresDeviceRestart;
        return out;
    }
};

}  // namespace smartdrone::core::application
