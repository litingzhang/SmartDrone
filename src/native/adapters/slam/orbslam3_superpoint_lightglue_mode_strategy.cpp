#include "adapters/slam/orbslam3_mode_strategy.h"
#include "adapters/slam/orbslam3_mode_common.h"

namespace smartdrone::adapters::slam {

FeatureFrontend SuperPointLightGlueModeStrategy::Frontend() const { return FeatureFrontend::SuperPointLightGlue; }

core::ports::SlamOutput SuperPointLightGlueModeStrategy::Process(OrbSlam3Engine &engine,
                                                                 const core::ports::SlamInputBatch &input,
                                                                 bool extractFeatures, bool extractPointCloud)
{
    return OrbModeStrategy::ProcessOrbSlamBackend(engine, input, extractFeatures, extractPointCloud, true);
}

std::unique_ptr<SlamModeStrategy> CreateSuperPointLightGlueModeStrategy()
{
    return std::make_unique<SuperPointLightGlueModeStrategy>();
}

} // namespace smartdrone::adapters::slam
