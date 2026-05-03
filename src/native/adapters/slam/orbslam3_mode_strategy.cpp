#include "adapters/slam/orbslam3_mode_strategy.h"

#include "adapters/slam/orbslam3_engine.h"

namespace smartdrone::adapters::slam {

std::unique_ptr<SlamModeStrategy> CreateSlamModeStrategy(FeatureFrontend frontend)
{
    switch (frontend) {
    case FeatureFrontend::LK:
        return CreateKltModeStrategy();
    case FeatureFrontend::LkGfttPerFrame:
        return CreateKltPerFrameModeStrategy();
    case FeatureFrontend::SuperPointLightGlue:
        return CreateSuperPointLightGlueModeStrategy();
    case FeatureFrontend::Orb:
    default:
        return CreateOrbModeStrategy();
    }
}

core::ports::SlamOutput OrbSlam3Engine::Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                                bool extractPointCloud)
{
    if (!m_modeStrategy || m_modeStrategy->Frontend() != m_featureFrontend) {
        m_modeStrategy = CreateSlamModeStrategy(m_featureFrontend);
    }
    return m_modeStrategy->Process(*this, input, extractFeatures, extractPointCloud);
}

} // namespace smartdrone::adapters::slam
