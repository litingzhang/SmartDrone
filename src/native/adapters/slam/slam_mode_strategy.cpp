#include "adapters/slam/slam_mode_strategy.h"

#include <array>
#include <cstddef>
#include <mutex>
#include <utility>

#include "adapters/slam/slam_engine_adapter.h"

namespace SmartDrone::adapters::slam {

namespace {

constexpr size_t kMaxFeatureFrontendSlots = 16;

struct SlamModeStrategyRegistryEntry {
    FeatureFrontend frontend{FeatureFrontend::Orb};
    SlamModeStrategyFactory factory{nullptr};
};

std::array<SlamModeStrategyRegistryEntry, kMaxFeatureFrontendSlots> &
SlamModeStrategyRegistry()
{
    static std::array<SlamModeStrategyRegistryEntry, kMaxFeatureFrontendSlots>
        registry{};
    return registry;
}

std::mutex &SlamModeStrategyRegistryMutex()
{
    static std::mutex mutex;
    return mutex;
}

} // namespace

void RegisterSlamModeStrategy(FeatureFrontend frontend,
                              SlamModeStrategyFactory factory)
{
    if (factory == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(SlamModeStrategyRegistryMutex());
    auto &registry = SlamModeStrategyRegistry();
    for (SlamModeStrategyRegistryEntry &entry : registry) {
        if (entry.factory != nullptr && entry.frontend == frontend) {
            entry.factory = factory;
            return;
        }
    }
    for (SlamModeStrategyRegistryEntry &entry : registry) {
        if (entry.factory == nullptr) {
            entry.frontend = frontend;
            entry.factory = factory;
            return;
        }
    }
}

SlamModeStrategyRegistrar::SlamModeStrategyRegistrar(
    FeatureFrontend frontend, SlamModeStrategyFactory factory)
{
    RegisterSlamModeStrategy(frontend, factory);
}

std::unique_ptr<SlamModeStrategy>
CreateSlamModeStrategy(FeatureFrontend frontend)
{
    {
        std::lock_guard<std::mutex> lock(SlamModeStrategyRegistryMutex());
        for (const SlamModeStrategyRegistryEntry &entry :
             SlamModeStrategyRegistry()) {
            if (entry.factory != nullptr && entry.frontend == frontend) {
                return entry.factory();
            }
        }
    }
    return CreateOrbModeStrategy();
}

core::ports::SlamOutput
SlamEngineAdapter::Process(const core::ports::SlamInputBatch &input,
                           bool extractFeatures, bool extractPointCloud)
{
    if (!m_modeStrategy || m_modeStrategy->Frontend() != m_featureFrontend) {
        m_modeStrategy = CreateSlamModeStrategy(m_featureFrontend);
    }
    return m_modeStrategy->Process(*this, input, extractFeatures,
                                   extractPointCloud);
}

} // namespace SmartDrone::adapters::slam
