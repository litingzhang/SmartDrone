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

class SlamEngineAccess;
class SlamModeStrategy;

} // namespace smartdrone::adapters::slam

namespace smartdrone::adapters::slam {

class SlamEngineAdapter final : public core::ports::ISlamEngine, public ISlamRuntimeControl {
  public:
    SlamEngineAdapter(std::unique_ptr<core::ports::ISlamTrackingBackend> backend, SlamInputMode inputMode, bool useImu,
                      std::string settingsPath = {});
    ~SlamEngineAdapter() override;

    bool Start() override;
    void SetOperationMode(core::domain::SlamOperationMode mode) override;
    void SetFeatureFrontend(FeatureFrontend frontend) override;
    void SetVisualFeatureFrontend(core::ports::IVisualFeatureFrontend *frontend) override;
    void SetVisualFeatureInputSizeLimit(int maxWidth, int maxHeight) override;
    void SetStereoVoLoopClosure(bool enabled, float scale = 1.20f, float relaxation = 1.40f) override;
    void SetStereoVoPerFrameAcceleration(std::string acceleration) override;
    void StepBackend() override;
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
    double StablePoseDeltaTime(double timestampSec) const;
    double SmoothedPoseDeltaTime(double timestampSec) const;
    bool AcceptStableRealtimePose(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
                                  double dt, int trackingState);
    bool AcceptRealtimeBootstrapPose(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
                                     int trackingState, bool rawIdentity);
    bool PredictRealtimePose(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec, double dt,
                             int trackingState, bool rawIdentity);
    bool HandleRealtimeMapBridge(core::ports::SlamOutput &out);
    bool ApplyRealtimeMapContinuity(core::ports::SlamOutput &out);
    bool ApplyRealtimeResetGuard(core::ports::SlamOutput &out);
    bool ShouldApplyRealtimeFeatureGate(const core::ports::SlamOutput &out) const;
    void ApplyRealtimeInnovationGate(core::ports::SlamOutput &out, double timestampSec, float maxStep);
    void ApplyRealtimeStepGate(core::ports::SlamOutput &out, float maxStep);
    void StoreSmoothedPose(const core::ports::PoseEstimate &pose, double timestampSec);
    bool UseAlphaBetaSmoother() const;
    void PredictInvalidSmoothedPose(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec);
    void PredictIdentitySmoothedPose(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec, double dt);
    core::ports::PoseEstimate PredictSmoothedPose(double dt) const;
    void ClampInnovationVector(float &innovationX, float &innovationY, float &innovationZ, float maxInnovation) const;
    void LimitSmoothedStep(core::ports::PoseEstimate &pose, float maxStep) const;
    void UpdateAlphaBetaVelocity(float innovationX, float innovationY, float innovationZ, double dt, float beta,
                                 float maxSpeed);
    void ApplyAlphaBetaSmoother(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec, double dt);
    core::ports::PoseEstimate PredictGuardedSmoothedPose(double dt, float maxGuardStep, float maxSpeed);
    void UpdateGuardMeasuredVelocity(const core::ports::PoseEstimate &pose, double dt, float maxSpeed);
    void ApplyGuardSmoother(core::ports::PoseEstimate &pose, bool &poseValid, double timestampSec, double dt,
                            int trackingState);
    void ResetRealtimeOutputAlignment();
    void ResetOutputSmoother();

    std::unique_ptr<core::ports::ISlamTrackingBackend> m_trackingBackend;
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
