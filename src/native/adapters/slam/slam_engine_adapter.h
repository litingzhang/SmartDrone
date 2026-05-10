#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "System.h"
#include "adapters/slam/slam_mode_state.h"
#include "core/application/config/app_args.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/slam_engine.h"

namespace smartdrone::adapters::slam {

enum class SlamInputMode : uint8_t {
    Stereo,
    MonoLeft,
    MonoRight,
};

class SlamEngineAccess;
class SlamModeStrategy;
class ExternalFeatureFrontendClient;

class SlamEngineAdapter final : public core::ports::ISlamEngine {
  public:
    SlamEngineAdapter(std::unique_ptr<ORB_SLAM3::System> system, SlamInputMode inputMode, bool useImu,
                   std::string settingsPath = {});
    ~SlamEngineAdapter() override;

    bool Start() override;
    void SetOperationMode(core::domain::SlamOperationMode mode);
    void SetFeatureFrontend(FeatureFrontend frontend);
    void SetExternalFeatureFrontendClient(ExternalFeatureFrontendClient *client);
    void SetExternalFeatureInputSizeLimit(int maxWidth, int maxHeight);
    void SetStereoVoLoopClosure(bool enabled, float scale = 1.20f, float relaxation = 1.40f);
    void SetStereoVoPerFrameAcceleration(std::string acceleration);
    void Stop() override;
    bool ShutdownAndSaveTrajectoryEuRoC(const std::string &path);
    core::ports::SlamOutput Process(const core::ports::SlamInputBatch &input, bool extractFeatures,
                                    bool extractPointCloud) override;

  private:
    friend class SlamEngineAccess;

    void StabilizeOutputPose(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec, int trackingState);
    void MaintainRealtimePoseContinuity(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
                                        int trackingState);
    void GateRealtimePoseQuality(core::ports::SlamOutput &out, double timestampSec);

    std::unique_ptr<ORB_SLAM3::System> m_system;
    std::unique_ptr<SlamModeSharedState> m_modeState;
    SlamInputMode m_inputMode{SlamInputMode::Stereo};
    bool m_useImu{false};
    core::domain::SlamOperationMode m_operationMode{core::domain::SlamOperationMode::Mapping};
    FeatureFrontend m_featureFrontend{FeatureFrontend::Orb};
    std::unique_ptr<SlamModeStrategy> m_modeStrategy;
    core::ports::PoseEstimate m_lastStablePose{};
    bool m_haveLastStablePose{false};
    double m_lastStableTimestampSec{0.0};
    float m_stableVelX{0.0f};
    float m_stableVelY{0.0f};
    float m_stableVelZ{0.0f};

    std::string m_settingsPath;
};

} // namespace smartdrone::adapters::slam
