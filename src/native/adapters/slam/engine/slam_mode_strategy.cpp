#include "adapters/slam/engine/slam_mode_strategy.h"

#include <cstddef>
#include <utility>

#include "adapters/slam/engine/slam_engine_adapter.h"
#include "adapters/slam/fixed_factory_registry.h"

namespace SmartDrone::Adapters::Slam {

namespace {

constexpr std::size_t MAX_FEATURE_FRONTEND_SLOTS = 16;

FixedFactoryRegistry<FeatureFrontend, SlamModeStrategyFactory,
                     MAX_FEATURE_FRONTEND_SLOTS> &
SlamModeStrategyRegistry()
{
    static FixedFactoryRegistry<FeatureFrontend, SlamModeStrategyFactory,
                                MAX_FEATURE_FRONTEND_SLOTS>
        registry;
    return registry;
}

} // namespace

void RegisterSlamModeStrategy(FeatureFrontend frontend,
                              SlamModeStrategyFactory factory)
{
    SlamModeStrategyRegistry().Register(frontend, factory);
}

SlamModeStrategyRegistrar::SlamModeStrategyRegistrar(
    FeatureFrontend frontend, SlamModeStrategyFactory factory)
{
    RegisterSlamModeStrategy(frontend, factory);
}

std::unique_ptr<SlamModeStrategy>
CreateSlamModeStrategy(FeatureFrontend frontend)
{
    SlamModeStrategyFactory factory = SlamModeStrategyRegistry().Find(frontend);
    if (factory != nullptr) {
        return factory();
    }
    return CreateOrbModeStrategy();
}

Core::Ports::SlamOutput
SlamEngineAdapter::Process(const Core::Ports::SlamInputBatch &input,
                           bool extractFeatures, bool extractPointCloud)
{
    if (!m_modeStrategy || m_modeStrategy->Frontend() != m_featureFrontend) {
        m_modeStrategy = CreateSlamModeStrategy(m_featureFrontend);
    }
    return m_modeStrategy->Process(*this, input, extractFeatures,
                                   extractPointCloud);
}

} // namespace SmartDrone::Adapters::Slam
