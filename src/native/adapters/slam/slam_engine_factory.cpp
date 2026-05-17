#include "adapters/slam/slam_engine_factory.h"

#include <memory>
#include <utility>

#include "adapters/slam/dpvo_tensorrt_engine.h"
#include "adapters/slam/klt_slam_engine.h"

#if defined(SMART_DRONE_ENABLE_ORB_SLAM3)
#include "adapters/slam/orb_slam3_backend.h"
#include "adapters/slam/slam_engine_adapter.h"
#endif

namespace smartdrone::adapters::slam {

ControlledSlamEngine CreateOrbSlam3Engine(const OrbSlam3EngineConfig &config)
{
#if defined(SMART_DRONE_ENABLE_ORB_SLAM3)
    auto backend = std::make_unique<OrbSlam3Backend>(config.vocabularyPath, config.settingsPath,
                                                     config.sensorMode, config.useViewer);
    auto engine = std::make_unique<SlamEngineAdapter>(std::move(backend), config.inputMode, config.useImu,
                                                      config.settingsPath);
    ControlledSlamEngine out{};
    out.control = engine.get();
    out.engine = std::move(engine);
    return out;
#else
    (void)config;
    return {};
#endif
}

ControlledSlamEngine CreateSlamEngine(const SlamEngineFactoryConfig &config)
{
    if (config.backend == SlamBackend::Klt) {
        auto engine = std::make_unique<KltSlamEngine>(config.settingsPath);
        ControlledSlamEngine out{};
        out.control = engine.get();
        out.engine = std::move(engine);
        return out;
    }

    if (config.backend == SlamBackend::DpvoTensorRt) {
        DpvoTensorRtConfig dpvoConfig = MakeDpvoTensorRtConfig(config.runtime, config.settingsPath);
        ControlledSlamEngine out{};
        out.engine = std::make_unique<DpvoTensorRtEngine>(std::move(dpvoConfig));
        return out;
    }

    OrbSlam3EngineConfig orbConfig{};
    orbConfig.vocabularyPath = config.vocabularyPath;
    orbConfig.settingsPath = config.settingsPath;
    orbConfig.sensorMode = config.sensorMode;
    orbConfig.useViewer = config.useViewer;
    orbConfig.useImu = config.useImu;
    orbConfig.inputMode = config.inputMode;
    ControlledSlamEngine out = CreateOrbSlam3Engine(orbConfig);
    if (out.engine != nullptr) {
        return out;
    }

    auto fallback = std::make_unique<KltSlamEngine>(config.settingsPath);
    out.control = fallback.get();
    out.engine = std::move(fallback);
    return out;
}

} // namespace smartdrone::adapters::slam
