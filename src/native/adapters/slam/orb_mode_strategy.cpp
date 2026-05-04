#include "adapters/slam/slam_mode_strategy.h"
#include "adapters/slam/slam_tracking_backend.h"

namespace smartdrone::adapters::slam {

FeatureFrontend OrbModeStrategy::Frontend() const { return FeatureFrontend::Orb; }

core::ports::SlamOutput OrbModeStrategy::Process(SlamEngineAdapter &engine,
                                                 const core::ports::SlamInputBatch &input,
                                                 bool extractFeatures, bool extractPointCloud)
{
    return RunSlamTrackingBackend(engine, input, extractFeatures, extractPointCloud, nullptr);
}

std::unique_ptr<SlamModeStrategy> CreateOrbModeStrategy()
{
    return std::make_unique<OrbModeStrategy>();
}

} // namespace smartdrone::adapters::slam
