#include "adapters/slam/engine/slam_mode_strategy.h"
#include "adapters/slam/engine/slam_tracking_backend.h"

namespace SmartDrone::Adapters::Slam {

namespace {

const SlamModeStrategyRegistrar
    kOrbModeStrategyRegistration(FeatureFrontend::Orb, &CreateOrbModeStrategy);

} // namespace

FeatureFrontend OrbModeStrategy::Frontend() const
{
    return FeatureFrontend::Orb;
}

Core::Ports::SlamOutput
OrbModeStrategy::Process(SlamEngineAdapter &engine,
                         const Core::Ports::SlamInputBatch &input,
                         bool extractFeatures, bool extractPointCloud)
{
    return RunSlamTrackingBackend(engine, input, extractFeatures,
                                  extractPointCloud, nullptr);
}

std::unique_ptr<SlamModeStrategy> CreateOrbModeStrategy()
{
    return std::make_unique<OrbModeStrategy>();
}

} // namespace SmartDrone::Adapters::Slam
