#include "core/application/session/epg/session_graph_runtime_factory.h"

#include "core/application/session/epg/calib_session_graph_service.h"
#include "core/application/session/epg/slam_session_graph_service.h"

namespace SmartDrone::core::application {

std::unique_ptr<ISessionGraphRuntime> CreateSessionGraphRuntime(const SessionGraphRuntimeFactoryConfig &config)
{
    if (config.mode == domain::RuntimeMode::Slam) {
        return std::make_unique<SlamSessionGraphRuntime>(SlamSessionGraphRuntimeConfig{
            config.cfg,
            config.tuning,
            config.telemetry,
            config.posePublisher,
            config.stop,
            config.livePose,
            config.runningFlag,
            config.factories,
        });
    }
    if (config.mode == domain::RuntimeMode::Calib) {
        return std::make_unique<CalibSessionGraphRuntime>(
            CalibSessionGraphRuntimeConfig{
                config.cfg, config.stop, config.livePose, config.runningFlag,
                config.factories});
    }
    return nullptr;
}

} // namespace SmartDrone::core::application
