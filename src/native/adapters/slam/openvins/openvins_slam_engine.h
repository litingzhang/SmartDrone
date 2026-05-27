#pragma once

#include <memory>
#include <string>

#include "adapters/slam/engine/slam_engine_control.h"
#include "adapters/slam/openvins/openvins_runtime.h"
#include "core/ports/slam_backend_maintenance.h"
#include "core/ports/slam_engine.h"

namespace SmartDrone::Adapters::Slam {

class OpenVinsSlamEngine final
    : public Core::Ports::ISlamEngine,
      public ISlamRuntimeControl,
      public Core::Ports::ISlamBackendMaintenance {
  public:
    explicit OpenVinsSlamEngine(std::string settingsPath);
    ~OpenVinsSlamEngine() override;

    bool Start() override;
    void Stop() override;
    Core::Ports::SlamOutput Process(const Core::Ports::SlamInputBatch &input,
                                    bool extractFeatures,
                                    bool extractPointCloud) override;

    void SetOperationMode(Core::Domain::SlamOperationMode mode) override;
    void SetFeatureFrontend(FeatureFrontend frontend) override;
    void SetVisualFeatureFrontend(
        Core::Ports::IVisualFeatureFrontend *frontend) override;
    void SetVisualFeatureInputSizeLimit(int maxWidth,
                                        int maxHeight) override;
    void SetStereoVoLoopClosure(bool enabled, float scale = 1.20f,
                                float relaxation = 1.40f) override;
    void SetStereoVoPerFrameAcceleration(std::string acceleration) override;
    bool ShutdownAndSaveTrajectoryEuRoC(const std::string &path) override;

    void RequestBackendStop() override;
    bool BackendStopped() const override;
    void StepBackend() override;

  private:
    Core::Ports::SlamOutput BuildBootstrapOutput(
        const Core::Ports::SlamInputBatch &input) const;
    bool FillContinuityPose(const Core::Ports::SlamInputBatch &input,
                            Core::Ports::SlamOutput &out);

    OpenVinsRuntime m_runtime;
    Core::Ports::PoseEstimate m_lastPose{};
    bool m_haveLastPose{false};
};

} // namespace SmartDrone::Adapters::Slam
