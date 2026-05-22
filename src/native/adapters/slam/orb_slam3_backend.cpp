#include "adapters/slam/orb_slam3_backend.h"

#include <memory>
#include <utility>

#include "adapters/slam/orb_slam3_runtime.h"
#include "adapters/slam/slam_engine_adapter.h"
#include "adapters/slam/slam_engine_factory.h"

namespace SmartDrone::adapters::slam {

namespace {

ControlledSlamEngine
CreateOrbSlam3SlamEngine(const SlamEngineFactoryConfig &config)
{
    auto backend = std::make_unique<OrbSlam3Runtime>(
        config.vocabularyPath, config.settingsPath, config.sensorMode,
        config.useViewer);
    auto engine = std::make_unique<SlamEngineAdapter>(
        std::move(backend), config.inputMode, config.useImu, config.settingsPath);
    ControlledSlamEngine out{};
    out.control = engine.get();
    out.engine = std::move(engine);
    return out;
}

const SlamEngineFactoryRegistrar
    kOrbSlam3SlamEngineRegistrar(SlamBackend::OrbSlam3,
                                 CreateOrbSlam3SlamEngine);

} // namespace

} // namespace SmartDrone::adapters::slam
