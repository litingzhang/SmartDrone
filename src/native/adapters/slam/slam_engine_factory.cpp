#include "adapters/slam/slam_engine_factory.h"

#include <array>
#include <cstddef>
#include <iostream>
#include <mutex>

namespace SmartDrone::Adapters::Slam {

namespace {

constexpr size_t kMaxSlamEngineFactorySlots = 16;

struct SlamEngineFactoryRegistryEntry {
    SlamBackend backend{SlamBackend::Klt};
    SlamEngineFactory factory{nullptr};
};

std::array<SlamEngineFactoryRegistryEntry, kMaxSlamEngineFactorySlots> &
SlamEngineFactoryRegistry()
{
    static std::array<SlamEngineFactoryRegistryEntry, kMaxSlamEngineFactorySlots>
        registry{};
    return registry;
}

std::mutex &SlamEngineFactoryRegistryMutex()
{
    static std::mutex mutex;
    return mutex;
}

SlamEngineFactory LookupSlamEngineFactory(SlamBackend backend)
{
    std::lock_guard<std::mutex> lock(SlamEngineFactoryRegistryMutex());
    for (const SlamEngineFactoryRegistryEntry &entry :
         SlamEngineFactoryRegistry()) {
        if (entry.factory != nullptr && entry.backend == backend) {
            return entry.factory;
        }
    }
    return nullptr;
}

} // namespace

void RegisterSlamEngineFactory(SlamBackend backend, SlamEngineFactory factory)
{
    if (factory == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(SlamEngineFactoryRegistryMutex());
    auto &registry = SlamEngineFactoryRegistry();
    for (SlamEngineFactoryRegistryEntry &entry : registry) {
        if (entry.factory != nullptr && entry.backend == backend) {
            entry.factory = factory;
            return;
        }
    }
    for (SlamEngineFactoryRegistryEntry &entry : registry) {
        if (entry.factory == nullptr) {
            entry.backend = backend;
            entry.factory = factory;
            return;
        }
    }
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
