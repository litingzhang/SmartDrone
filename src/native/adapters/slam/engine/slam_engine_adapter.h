#pragma once

#include <memory>
#include <string>

#include <sophus/se3.hpp>

#include "adapters/slam/engine/slam_engine_control.h"
#include "adapters/slam/engine/slam_engine_factory.h"
#include "adapters/slam/engine/slam_mode_state.h"
#include "core/domain/runtime_mode.h"
#include "core/ports/slam_engine.h"

namespace SmartDrone::Adapters::Slam {

class SlamEngineAccess;
class SlamModeStrategy;

} // namespace SmartDrone::Adapters::Slam

namespace SmartDrone::Adapters::Slam {

class SlamEngineAdapter final : public Core::Ports::ISlamEngine, public ISlamRuntimeControl {
  public:
    SlamEngineAdapter(std::unique_ptr<Core::Ports::ISlamTrackingBackend> backend, SlamInputMode inputMode, bool useImu,
                      std::string settingsPath = {});
    ~SlamEngineAdapter() override;

    bool Start() override;
    void SetOperationMode(Core::Domain::SlamOperationMode mode) override;
    void SetFeatureFrontend(FeatureFrontend frontend) override;
    void SetVisualFeatureFrontend(Core::Ports::IVisualFeatureFrontend *frontend) override;
    void SetVisualFeatureInputSizeLimit(int maxWidth, int maxHeight) override;
    void SetStereoVoLoopClosure(bool enabled, float scale = 1.20f, float relaxation = 1.40f) override;
    void SetStereoVoPerFrameAcceleration(std::string acceleration) override;
    void StepBackend() override;
    void Stop() override;
    bool ShutdownAndSaveTrajectoryEuRoC(const std::string &path) override;
    Core::Ports::SlamOutput Process(const Core::Ports::SlamInputBatch &input, bool extractFeatures,
                                    bool extractPointCloud) override;

  private:
    friend class SlamEngineAccess;

    void StabilizeOutputPose(Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec, int trackingState);
    void MaintainRealtimePoseContinuity(Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
                                        int trackingState);
    void GateRealtimePoseQuality(Core::Ports::SlamOutput &out, double timestampSec);
    double StablePoseDeltaTime(double timestampSec) const;
    double SmoothedPoseDeltaTime(double timestampSec) const;
    bool AcceptStableRealtimePose(Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
                                  double dt, int trackingState);
    bool AcceptRealtimeBootstrapPose(Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec,
                                     int trackingState, bool rawIdentity);
    bool PredictRealtimePose(Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec, double dt,
                             int trackingState, bool rawIdentity);
    bool HandleRealtimeMapBridge(Core::Ports::SlamOutput &out);
    bool ApplyRealtimeMapContinuity(Core::Ports::SlamOutput &out);
    bool ApplyRealtimeResetGuard(Core::Ports::SlamOutput &out);
    bool ShouldApplyRealtimeFeatureGate(const Core::Ports::SlamOutput &out) const;
    void ApplyRealtimeInnovationGate(Core::Ports::SlamOutput &out, double timestampSec, float maxStep);
    void ApplyRealtimeStepGate(Core::Ports::SlamOutput &out, float maxStep);
    void StoreSmoothedPose(const Core::Ports::PoseEstimate &pose, double timestampSec);
    bool UseAlphaBetaSmoother() const;
    void PredictInvalidSmoothedPose(Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec);
    void PredictIdentitySmoothedPose(Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec, double dt);
    Core::Ports::PoseEstimate PredictSmoothedPose(double dt) const;
    void ClampInnovationVector(float &innovationX, float &innovationY, float &innovationZ, float maxInnovation) const;
    void LimitSmoothedStep(Core::Ports::PoseEstimate &pose, float maxStep) const;
    void UpdateAlphaBetaVelocity(float innovationX, float innovationY, float innovationZ, double dt, float beta,
                                 float maxSpeed);
    void ApplyAlphaBetaSmoother(Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec, double dt);
    Core::Ports::PoseEstimate PredictGuardedSmoothedPose(double dt, float maxGuardStep, float maxSpeed);
    void UpdateGuardMeasuredVelocity(const Core::Ports::PoseEstimate &pose, double dt, float maxSpeed);
    void ApplyGuardSmoother(Core::Ports::PoseEstimate &pose, bool &poseValid, double timestampSec, double dt,
                            int trackingState);
    void ResetRealtimeOutputAlignment();
    void ResetOutputSmoother();

    std::unique_ptr<Core::Ports::ISlamTrackingBackend> m_trackingBackend;
    std::unique_ptr<SlamModeSharedState> m_modeState;
    SlamInputMode m_inputMode{SlamInputMode::Stereo};
    bool m_useImu{false};
    FeatureFrontend m_featureFrontend{FeatureFrontend::Orb};
    std::unique_ptr<SlamModeStrategy> m_modeStrategy;
    Core::Ports::PoseEstimate m_lastStablePose{};
    bool m_haveLastStablePose{false};
    double m_lastStableTimestampSec{0.0};
    float m_stableVelX{0.0f};
    float m_stableVelY{0.0f};
    float m_stableVelZ{0.0f};
    Core::Ports::PoseEstimate m_smoothedOutputPose{};
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

} // namespace SmartDrone::Adapters::Slam
