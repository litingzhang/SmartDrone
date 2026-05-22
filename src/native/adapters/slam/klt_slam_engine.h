#pragma once

#include <memory>
#include <string>

#include "adapters/slam/slam_engine_control.h"
#include "adapters/slam/slam_mode_state.h"
#include "core/ports/slam_engine.h"

namespace SmartDrone::Adapters::Slam {

class KltSlamEngine final : public Core::Ports::ISlamEngine, public ISlamRuntimeControl {
  public:
    explicit KltSlamEngine(std::string settingsPath);
    ~KltSlamEngine() override;

    bool Start() override;
    void Stop() override;
    Core::Ports::SlamOutput Process(const Core::Ports::SlamInputBatch &input, bool extractFeatures,
                                    bool extractPointCloud) override;

    void SetOperationMode(Core::Domain::SlamOperationMode mode) override;
    void SetFeatureFrontend(FeatureFrontend frontend) override;
    void SetVisualFeatureFrontend(Core::Ports::IVisualFeatureFrontend *frontend) override;
    void SetVisualFeatureInputSizeLimit(int maxWidth, int maxHeight) override;
    void SetStereoVoLoopClosure(bool enabled, float scale = 1.20f, float relaxation = 1.40f) override;
    void SetStereoVoPerFrameAcceleration(std::string acceleration) override;

  private:
    Core::Ports::SlamOutput ProcessContinuousKlt(const Core::Ports::SlamInputBatch &input, bool extractFeatures);
    Core::Ports::SlamOutput ProcessPerFrameKlt(const Core::Ports::SlamInputBatch &input, bool extractFeatures);

    std::unique_ptr<SlamModeSharedState> m_state;
    FeatureFrontend m_frontend{FeatureFrontend::LkGfttPerFrame};
    std::string m_settingsPath;
};

} // namespace SmartDrone::Adapters::Slam
