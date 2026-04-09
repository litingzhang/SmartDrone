#pragma once

#include <string_view>
#include <vector>

#include "core/domain/capabilities.h"

namespace smartdrone::core::application {

class ConfigRegistry {
  public:
    static constexpr std::string_view kCameraExposureUs = "camera.exposure_us";
    static constexpr std::string_view kCameraGain = "camera.gain";
    static constexpr std::string_view kCameraAutoExposure = "camera.auto_exposure";
    static constexpr std::string_view kCameraPairWindowMs = "camera.pair_window_ms";
    static constexpr std::string_view kSlamInputFps = "slam.input_fps";
    static constexpr std::string_view kSlamPerceptionMode = "slam.perception_mode";
    static constexpr std::string_view kSlamOperationMode = "slam.operation_mode";
    static constexpr std::string_view kStreamUdpEnabled = "stream.udp_enabled";
    static constexpr std::string_view kStreamUdpIp = "stream.udp_ip";
    static constexpr std::string_view kStreamSendImage = "stream.send_image";
    static constexpr std::string_view kStreamSendFeature = "stream.send_feature";
    static constexpr std::string_view kStreamSendMap = "stream.send_map";

    static std::vector<domain::ConfigDescriptor> DefaultDescriptors();

  private:
    static domain::ConfigDescriptor Make(std::string_view key, std::string_view description, bool hotReloadable,
                                         bool requiresPipelineRestart, bool requiresDeviceRestart);
};

} // namespace smartdrone::core::application
