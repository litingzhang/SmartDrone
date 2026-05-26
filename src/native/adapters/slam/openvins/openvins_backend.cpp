#include "adapters/slam/openvins/openvins_runtime.h"
#include "adapters/slam/openvins/openvins_slam_engine.h"

#include <memory>
#include "adapters/slam/engine/slam_engine_factory.h"

namespace SmartDrone::Adapters::Slam {

namespace {

ControlledSlamEngine CreateOpenVinsSlamEngine(
    const SlamEngineFactoryConfig &config)
{
    auto engine = std::make_unique<OpenVinsSlamEngine>(config.settingsPath);
    ControlledSlamEngine out{};
    out.control = engine.get();
    out.backendMaintenance = engine.get();
    out.engine = std::move(engine);
    return out;
}

const SlamEngineFactoryRegistrar OPENVINS_SLAM_ENGINE_REGISTRAR(
    SlamBackend::OpenVins, CreateOpenVinsSlamEngine);

} // namespace

} // namespace SmartDrone::Adapters::Slam
