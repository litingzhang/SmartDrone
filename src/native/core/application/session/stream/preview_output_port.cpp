#include "core/application/session/stream/preview_output_port.h"

#include "core/application/config/runtime_app_types.h"

namespace SmartDrone::core::application {

PreviewOutputOpenConfig MakePreviewOutputOpenConfig(
    const MainRuntimeAliases &aliases)
{
    PreviewOutputOpenConfig config{};
    config.ip = aliases.udpIp;
    config.port = aliases.udpPort;
    config.jpegQuality = aliases.udpJpegQ;
    config.maxPayload = aliases.udpPayload;
    config.maxQueue = aliases.udpQueue;
    return config;
}

} // namespace SmartDrone::core::application
