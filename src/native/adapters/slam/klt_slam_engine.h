#pragma once

#include <memory>
#include <string>

#include "adapters/slam/slam_engine_control.h"
#include "adapters/slam/slam_mode_state.h"
#include "core/ports/slam_engine.h"

namespace smartdrone::adapters::slam {

class KltSlamEngine final : public core::ports::ISlamEngine, public ISlamRuntimeControl {
  public:
    explicit KltSlamEngine(std::string settingsPath);
    ~KltSlamEngine() override;

    bool Start() override;
    void Stop() override;
    core::ports::SlamOutput Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                    bool extractPointCloud) override;

    void SetOperationMode(core::domain::SlamOperationMode mode) override;
    void SetFeatureFrontend(FeatureFrontend frontend) override;
    void SetExternalFeatureFrontendClient(ExternalFeatureFrontendClient *client) override;
    void SetExternalFeatureInputSizeLimit(int maxWidth, int maxHeight) override;
    void SetStereoVoLoopClosure(bool enabled, float scale = 1.20f, float relaxation = 1.40f) override;
    void SetStereoVoPerFrameAcceleration(std::string acceleration) override;

  private:
    core::ports::SlamOutput ProcessContinuousKlt(const core::ports::SlamInputBatch &input, bool extractFeatures);
    core::ports::SlamOutput ProcessPerFrameKlt(const core::ports::SlamInputBatch &input, bool extractFeatures);

    std::unique_ptr<SlamModeSharedState> m_state;
    FeatureFrontend m_frontend{FeatureFrontend::LkGfttPerFrame};
    std::string m_settingsPath;
};

} // namespace smartdrone::adapters::slam
