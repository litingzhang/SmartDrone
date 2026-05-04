#pragma once

#include <cstdint>
#include <memory>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "core/application/config/app_args.h"
#include "core/ports/slam_engine.h"

namespace smartdrone::adapters::slam {

class SlamEngineAdapter;

class SlamModeStrategy {
  public:
    virtual ~SlamModeStrategy() = default;

    virtual FeatureFrontend Frontend() const = 0;
    virtual core::ports::SlamOutput Process(SlamEngineAdapter &engine, const core::ports::SlamInputBatch &input,
                                            bool extractFeatures, bool extractPointCloud) = 0;
};

class OrbModeStrategy final : public SlamModeStrategy {
  public:
    FeatureFrontend Frontend() const override;
    core::ports::SlamOutput Process(SlamEngineAdapter &engine, const core::ports::SlamInputBatch &input,
                                    bool extractFeatures, bool extractPointCloud) override;
};

class KltModeStrategy final : public SlamModeStrategy {
  public:
    FeatureFrontend Frontend() const override;
    core::ports::SlamOutput Process(SlamEngineAdapter &engine, const core::ports::SlamInputBatch &input,
                                    bool extractFeatures, bool extractPointCloud) override;

  private:
    static Sophus::SE3f ApplyLoopClosure(SlamEngineAdapter &engine, const cv::Mat &leftRect, uint64_t frameId,
                                         const Sophus::SE3f &rawTwc);
};

class KltPerFrameModeStrategy final : public SlamModeStrategy {
  public:
    FeatureFrontend Frontend() const override;
    core::ports::SlamOutput Process(SlamEngineAdapter &engine, const core::ports::SlamInputBatch &input,
                                    bool extractFeatures, bool extractPointCloud) override;
};

class SuperPointLightGlueModeStrategy final : public SlamModeStrategy {
  public:
    FeatureFrontend Frontend() const override;
    core::ports::SlamOutput Process(SlamEngineAdapter &engine, const core::ports::SlamInputBatch &input,
                                    bool extractFeatures, bool extractPointCloud) override;
};

std::unique_ptr<SlamModeStrategy> CreateSlamModeStrategy(FeatureFrontend frontend);
std::unique_ptr<SlamModeStrategy> CreateOrbModeStrategy();
std::unique_ptr<SlamModeStrategy> CreateKltModeStrategy();
std::unique_ptr<SlamModeStrategy> CreateKltPerFrameModeStrategy();
std::unique_ptr<SlamModeStrategy> CreateSuperPointLightGlueModeStrategy();

} // namespace smartdrone::adapters::slam
