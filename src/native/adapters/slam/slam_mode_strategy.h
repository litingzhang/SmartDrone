#pragma once

#include <cstdint>
#include <memory>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "core/application/config/app_args.h"
#include "core/ports/slam_engine.h"

namespace SmartDrone::adapters::slam {

class SlamEngineAdapter;

class SlamModeStrategy {
  public:
    virtual ~SlamModeStrategy() = default;

    virtual FeatureFrontend Frontend() const = 0;
    virtual core::ports::SlamOutput
    Process(SlamEngineAdapter &engine, const core::ports::SlamInputBatch &input,
            bool extractFeatures, bool extractPointCloud) = 0;
};

class OrbModeStrategy final : public SlamModeStrategy {
  public:
    FeatureFrontend Frontend() const override;
    core::ports::SlamOutput Process(SlamEngineAdapter &engine,
                                    const core::ports::SlamInputBatch &input,
                                    bool extractFeatures,
                                    bool extractPointCloud) override;
};

class VisualFeatureLightGlueModeStrategy final : public SlamModeStrategy {
  public:
    explicit VisualFeatureLightGlueModeStrategy(FeatureFrontend frontend);

    FeatureFrontend Frontend() const override;
    core::ports::SlamOutput Process(SlamEngineAdapter &engine,
                                    const core::ports::SlamInputBatch &input,
                                    bool extractFeatures,
                                    bool extractPointCloud) override;

  private:
    FeatureFrontend m_frontend;
};

using SlamModeStrategyFactory = std::unique_ptr<SlamModeStrategy> (*)();

void RegisterSlamModeStrategy(FeatureFrontend frontend,
                              SlamModeStrategyFactory factory);

class SlamModeStrategyRegistrar {
  public:
    SlamModeStrategyRegistrar(FeatureFrontend frontend,
                              SlamModeStrategyFactory factory);
};

std::unique_ptr<SlamModeStrategy>
CreateSlamModeStrategy(FeatureFrontend frontend);
std::unique_ptr<SlamModeStrategy> CreateOrbModeStrategy();
std::unique_ptr<SlamModeStrategy> CreateSuperPointLightGlueModeStrategy();
std::unique_ptr<SlamModeStrategy> CreateXFeatLightGlueModeStrategy();

} // namespace SmartDrone::adapters::slam
