#pragma once

#include <memory>
#include <string>

#include <sophus/se3.hpp>

#include "adapters/slam/slam_engine_control.h"
#include "adapters/slam/slam_engine_factory.h"
#include "adapters/slam/slam_mode_state.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/slam_engine.h"

namespace smartdrone::adapters::slam {

class OrbSlam3Backend;
class SlamEngineAccess;
class SlamModeStrategy;

} // namespace smartdrone::adapters::slam

namespace smartdrone::adapters::slam {

class SlamEngineAdapter final : public core::ports::ISlamEngine, public ISlamRuntimeControl {
  public:
    SlamEngineAdapter(std::unique_ptr<OrbSlam3Backend> backend, SlamInputMode inputMode, bool useImu,
                      std::string settingsPath = {});
    ~SlamEngineAdapter() override;

    bool Start() override;
    void SetOperationMode(core::domain::SlamOperationMode mode) override;
    void SetFeatureFrontend(FeatureFrontend frontend) override;
    void SetExternalFeatureFrontendClient(ExternalFeatureFrontendClient *client) override;
    void SetExternalFeatureInputSizeLimit(int maxWidth, int maxHeight) override;
    void SetStereoVoLoopClosure(bool enabled, float scale = 1.20f, float relaxation = 1.40f) override;
    void SetStereoVoPerFrameAcceleration(std::string acceleration) override;
    void Stop() override;
    bool ShutdownAndSaveTrajectoryEuRoC(const std::string &path) override;
    core::ports::SlamOutput Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                    bool extractPointCloud) override;

  private:
    friend class SlamEngineAccess;

    void StabilizeOutputPose(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec, int trackingState);
    void MaintainRealtimePoseContinuity(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
                                        int trackingState);
    void GateRealtimePoseQuality(core::ports::SlamOutput &out, double timestampSec);
    void ResetRealtimeOutputAlignment();
    void ResetOutputSmoother();

    std::unique_ptr<OrbSlam3Backend> m_orbBackend;
    std::unique_ptr<SlamModeSharedState> m_modeState;
    SlamInputMode m_inputMode{SlamInputMode::Stereo};
    bool m_useImu{false};
    FeatureFrontend m_featureFrontend{FeatureFrontend::Orb};
    std::unique_ptr<SlamModeStrategy> m_modeStrategy;
    core::ports::PoseEstimate m_lastStablePose{};
    bool m_haveLastStablePose{false};
    double m_lastStableTimestampSec{0.0};
    float m_stableVelX{0.0f};
    float m_stableVelY{0.0f};
    float m_stableVelZ{0.0f};
    core::ports::PoseEstimate m_smoothedOutputPose{};
    bool m_haveSmoothedOutputPose{false};
    double m_smoothedOutputTimestampSec{0.0};
    float m_smoothVelX{0.0f};
    float m_smoothVelY{0.0f};
    float m_smoothVelZ{0.0f};
    Sophus::SE3f m_realtimeOutputFromRawPose{};
    bool m_realtimeOutputMapContinuityActive{false};
    unsigned long m_realtimeOutputMapContinuityMapId{0};
    bool m_realtimeOutputHaveLastMapId{false};
    unsigned long m_realtimeOutputLastMapId{0};

    std::string m_settingsPath;
};

} // namespace smartdrone::adapters::slam
