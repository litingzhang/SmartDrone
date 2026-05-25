#include "adapters/slam/engine/slam_engine_factory.h"

#include <cstddef>
#include <iostream>

#include "adapters/slam/fixed_factory_registry.h"

namespace SmartDrone::Adapters::Slam {

namespace {

constexpr std::size_t MAX_SLAM_ENGINE_FACTORY_SLOTS = 16;

FixedFactoryRegistry<SlamBackend, SlamEngineFactory,
                     MAX_SLAM_ENGINE_FACTORY_SLOTS> &
SlamEngineFactoryRegistry()
{
    static FixedFactoryRegistry<SlamBackend, SlamEngineFactory,
                                MAX_SLAM_ENGINE_FACTORY_SLOTS>
        registry;
    return registry;
}

SlamEngineFactory LookupSlamEngineFactory(SlamBackend backend)
{
    return SlamEngineFactoryRegistry().Find(backend);
}

} // namespace

void RegisterSlamEngineFactory(SlamBackend backend, SlamEngineFactory factory)
{
    SlamEngineFactoryRegistry().Register(backend, factory);
}

SlamEngineFactoryRegistrar::SlamEngineFactoryRegistrar(
    SlamBackend backend, SlamEngineFactory factory)
{
    RegisterSlamEngineFactory(backend, factory);
}

ControlledSlamEngine CreateOrbSlam3Engine(const OrbSlam3EngineConfig &config)
{
    SlamEngineFactory factory = LookupSlamEngineFactory(SlamBackend::OrbSlam3);
    if (factory == nullptr) {
        return {};
    }
    SlamEngineFactoryConfig factoryConfig{};
    factoryConfig.backend = SlamBackend::OrbSlam3;
    factoryConfig.vocabularyPath = config.vocabularyPath;
    factoryConfig.settingsPath = config.settingsPath;
    factoryConfig.sensorMode = config.sensorMode;
    factoryConfig.useViewer = config.useViewer;
    factoryConfig.useImu = config.useImu;
    factoryConfig.inputMode = config.inputMode;
    return factory(factoryConfig);
}

ControlledSlamEngine CreateSlamEngine(const SlamEngineFactoryConfig &config)
{
    SlamEngineFactory factory = LookupSlamEngineFactory(config.backend);
    ControlledSlamEngine out =
        factory != nullptr ? factory(config) : ControlledSlamEngine{};
    if (out.engine != nullptr) {
        return out;
    }

    if (config.backend == SlamBackend::OrbSlam3) {
        std::cerr << "[slam_factory] ORB-SLAM3 backend returned no engine; "
                     "falling back to KLT\n";
    }

    SlamEngineFactory fallbackFactory = LookupSlamEngineFactory(SlamBackend::Klt);
    if (fallbackFactory == nullptr || config.backend == SlamBackend::Klt) {
        return {};
    }
    return fallbackFactory(config);
}

} // namespace SmartDrone::Adapters::Slam
