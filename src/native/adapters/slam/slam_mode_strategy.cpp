#include "adapters/slam/slam_mode_strategy.h"

#include "adapters/slam/slam_engine_adapter.h"

namespace smartdrone::adapters::slam {

std::unique_ptr<SlamModeStrategy> CreateSlamModeStrategy(FeatureFrontend frontend)
{
    switch (frontend) {
    case FeatureFrontend::SuperPointLightGlue:
        return CreateSuperPointLightGlueModeStrategy();
    case FeatureFrontend::XFeatLightGlue:
        return CreateXFeatLightGlueModeStrategy();
    case FeatureFrontend::Orb:
    default:
        return CreateOrbModeStrategy();
    }
}

core::ports::SlamOutput SlamEngineAdapter::Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                                bool extractPointCloud)
{
    if (!m_modeStrategy || m_modeStrategy->Frontend() != m_featureFrontend) {
        m_modeStrategy = CreateSlamModeStrategy(m_featureFrontend);
    }
    return m_modeStrategy->Process(*this, input, extractFeatures, extractPointCloud);
}

} // namespace smartdrone::adapters::slam
