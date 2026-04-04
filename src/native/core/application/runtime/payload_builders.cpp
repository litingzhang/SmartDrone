#include "core/application/runtime/payload_builders.h"

#include <sstream>

#include "core/application/config/capability_catalog.h"

namespace smartdrone::core::application {

std::vector<uint8_t> TextPayloadFromString(const std::string& text)
{
    return std::vector<uint8_t>(text.begin(), text.end());
}

std::string JoinStrings(const std::vector<std::string>& values, const char* sep)
{
    std::string out;
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) out += sep;
        out += values[i];
    }
    return out;
}

std::vector<uint8_t> BuildCapabilitiesPayload()
{
    const auto capabilities = CapabilityCatalog::BuildDefault();
    std::vector<std::string> runtimeModes;
    for (const auto mode : capabilities.runtimeModes) runtimeModes.emplace_back(smartdrone::core::domain::ToString(mode));
    std::vector<std::string> perceptionModes;
    for (const auto mode : capabilities.perceptionModes) perceptionModes.emplace_back(smartdrone::core::domain::ToString(mode));
    std::vector<std::string> slamModes;
    for (const auto mode : capabilities.slamModes) slamModes.emplace_back(smartdrone::core::domain::ToString(mode));
    std::vector<std::string> configKeys;
    configKeys.reserve(capabilities.configKeys.size());
    for (const auto& item : capabilities.configKeys) configKeys.push_back(item.key);

    std::ostringstream oss;
    oss << "runtime_modes=" << JoinStrings(runtimeModes, ",") << "\n";
    oss << "perception_modes=" << JoinStrings(perceptionModes, ",") << "\n";
    oss << "slam_modes=" << JoinStrings(slamModes, ",") << "\n";
    oss << "camera_providers=" << JoinStrings(capabilities.cameraProviders, ",") << "\n";
    oss << "imu_providers=" << JoinStrings(capabilities.imuProviders, ",") << "\n";
    oss << "slam_engines=" << JoinStrings(capabilities.slamEngines, ",") << "\n";
    oss << "command_channels=" << JoinStrings(capabilities.commandChannels, ",") << "\n";
    oss << "config_keys=" << JoinStrings(configKeys, ",") << "\n";
    return TextPayloadFromString(oss.str());
}

std::vector<uint8_t> BuildConfigPayload(const UnifiedConfig& cfg, smartdrone::core::domain::RuntimeMode runtimeMode)
{
    std::ostringstream oss;
    oss << "runtime.mode=" << smartdrone::core::domain::ToString(runtimeMode) << "\n";
    oss << "camera.exposure_us=" << cfg.app.camera.exposureUs << "\n";
    oss << "camera.gain=" << cfg.app.camera.gain << "\n";
    oss << "camera.pair_window_ms=" << cfg.app.camera.pairMs << "\n";
    oss << "slam.input_fps=" << cfg.app.runtime.slamInputFps << "\n";
    oss << "slam.perception_mode=" << ToSensorModeText(cfg.app.sensorMode) << "\n";
    oss << "slam.operation_mode=" << smartdrone::core::domain::ToString(cfg.app.runtime.slamOperationMode) << "\n";
    oss << "slam.settings=" << cfg.app.settings << "\n";
    oss << "stream.udp_enabled=" << (cfg.app.udp.enable ? "true" : "false") << "\n";
    oss << "stream.udp_ip=" << cfg.app.udp.ip << "\n";
    oss << "stream.send_image=" << (cfg.app.udp.sendImage ? "true" : "false") << "\n";
    oss << "stream.send_feature=" << (cfg.app.udp.sendFeature ? "true" : "false") << "\n";
    oss << "stream.send_map=" << (cfg.app.udp.sendMap ? "true" : "false") << "\n";
    return TextPayloadFromString(oss.str());
}

}  // namespace smartdrone::core::application
