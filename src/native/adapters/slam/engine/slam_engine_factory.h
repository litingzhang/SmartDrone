#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "adapters/slam/dpvo/dpvo_tensorrt_engine.h"
#include "adapters/slam/engine/slam_engine_control.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/slam_backend.h"
#include "core/ports/slam_engine.h"

namespace SmartDrone::Adapters::Slam {

using SlamInputMode = Core::Ports::SlamInputMode;

struct OrbSlam3EngineConfig {
    std::string vocabularyPath;
    std::string settingsPath;
    SensorMode sensorMode{SensorMode::Stereo};
    bool useViewer{false};
    bool useImu{false};
    SlamInputMode inputMode{SlamInputMode::Stereo};
};

struct SlamEngineFactoryConfig {
    SlamBackend backend{SlamBackend::Klt};
    std::string vocabularyPath;
    std::string settingsPath;
    SensorMode sensorMode{SensorMode::Stereo};
    bool useViewer{false};
    bool useImu{false};
    SlamInputMode inputMode{SlamInputMode::Stereo};
    DpvoRuntimeConfig dpvoRuntime;
};

struct ControlledSlamEngine {
    std::unique_ptr<Core::Ports::ISlamEngine> engine;
    ISlamRuntimeControl *control{nullptr};
};

using SlamEngineFactory =
    ControlledSlamEngine (*)(const SlamEngineFactoryConfig &config);

void RegisterSlamEngineFactory(SlamBackend backend, SlamEngineFactory factory);

class SlamEngineFactoryRegistrar {
  public:
    SlamEngineFactoryRegistrar(SlamBackend backend, SlamEngineFactory factory);
};

ControlledSlamEngine CreateOrbSlam3Engine(const OrbSlam3EngineConfig &config);
ControlledSlamEngine CreateSlamEngine(const SlamEngineFactoryConfig &config);

} // namespace SmartDrone::Adapters::Slam
