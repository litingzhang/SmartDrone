#include "adapters/slam/klt_slam_engine.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <utility>

#include "adapters/slam/klt_continuous_frontend.h"
#include "adapters/slam/klt_loop_closure_backend.h"
#include "adapters/slam/klt_mode_utils.h"
#include "adapters/slam/klt_per_frame_frontend.h"
#include "adapters/slam/klt_pose_estimator.h"
#include "adapters/slam/klt_track_manager.h"
#include "adapters/slam/slam_engine_factory.h"
#include "adapters/slam/slam_output_utils.h"
#include "adapters/slam/slam_pose_utils.h"

namespace smartdrone::adapters::slam {

namespace {

ControlledSlamEngine
CreateKltSlamEngine(const SlamEngineFactoryConfig &config) {
  auto engine = std::make_unique<KltSlamEngine>(config.settingsPath);
  ControlledSlamEngine out{};
  out.control = engine.get();
  out.engine = std::move(engine);
  return out;
}

const SlamEngineFactoryRegistrar kKltSlamEngineRegistrar(SlamBackend::Klt,
                                                         CreateKltSlamEngine);

} // namespace

KltSlamEngine::KltSlamEngine(std::string settingsPath)
    : m_state(std::make_unique<SlamModeSharedState>()),
      m_settingsPath(std::move(settingsPath)) {}

KltSlamEngine::~KltSlamEngine() = default;

bool KltSlamEngine::Start() {
  if (m_state == nullptr) {
    m_state = std::make_unique<SlamModeSharedState>();
  }
  m_state->LoadStereoCalibration(m_settingsPath);
  m_state->ResetTrackingState();
  return true;
}

void KltSlamEngine::Stop() {
  if (m_state != nullptr) {
    m_state->ResetTrackingState();
  }
}

void KltSlamEngine::SetOperationMode(core::domain::SlamOperationMode mode) {
  (void)mode;
}

void KltSlamEngine::SetFeatureFrontend(FeatureFrontend frontend) {
  if (frontend != FeatureFrontend::LK &&
      frontend != FeatureFrontend::LkGfttPerFrame) {
    frontend = FeatureFrontend::LkGfttPerFrame;
  }
  if (m_frontend != frontend && m_state != nullptr) {
    m_state->ResetTrackingState();
  }
  m_frontend = frontend;
}

void KltSlamEngine::SetVisualFeatureFrontend(
    core::ports::IVisualFeatureFrontend *frontend) {
  if (m_state != nullptr) {
    m_state->m_visualFeatureFrontend = frontend;
  }
}

void KltSlamEngine::SetVisualFeatureInputSizeLimit(int maxWidth,
                                                   int maxHeight) {
  if (m_state != nullptr) {
    m_state->m_visualFeatureInputMaxWidth = std::max(0, maxWidth);
    m_state->m_visualFeatureInputMaxHeight = std::max(0, maxHeight);
  }
}

void KltSlamEngine::SetStereoVoLoopClosure(bool enabled, float scale,
                                           float relaxation) {
  if (m_state == nullptr) {
    return;
  }
  m_state->m_lkLoop.enabled = enabled;
  m_state->m_lkLoop.scale = std::clamp(scale, 0.25f, 4.0f);
  m_state->m_lkLoop.relaxation = std::clamp(relaxation, 0.0f, 4.0f);
}

void KltSlamEngine::SetStereoVoPerFrameAcceleration(std::string acceleration) {
  if (m_state == nullptr) {
    return;
  }
  std::transform(
      acceleration.begin(), acceleration.end(), acceleration.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (acceleration.empty()) {
    acceleration = "auto";
  }
  if (m_state->m_lkPerFrameAcceleration == acceleration) {
    return;
  }
  m_state->m_lkPerFrameAcceleration = std::move(acceleration);
  m_state->m_lkPerFrameVpi.reset();
  m_state->m_lkPerFrameAccelLogged = false;
}

core::ports::SlamOutput
KltSlamEngine::Process(const core::ports::SlamInputBatch &input,
                       bool extractFeatures, bool extractPointCloud) {
  (void)extractPointCloud;
  if (m_frontend == FeatureFrontend::LK) {
    return ProcessContinuousKlt(input, extractFeatures);
  }
  return ProcessPerFrameKlt(input, extractFeatures);
}

core::ports::SlamOutput
KltSlamEngine::ProcessContinuousKlt(const core::ports::SlamInputBatch &input,
                                    bool extractFeatures) {
  SlamModeSharedState &state = *m_state;
  core::ports::SlamOutput out = MakeOkSlamOutput(input);
  state.ResetVisualFeatureStats();

  KltContinuousFrontendResult frontend = RunKltContinuousFrontend(
      state, input.stereo.left.gray, input.stereo.right.gray);
  if (!frontend.valid) {
    MarkSlamOutputPoseLost(out, core::ports::kSlamTrackingLost);
    return out;
  }

  if (!frontend.havePreviousFrame) {
    RefreshLkStereoSeedsIfNeeded(state, frontend.leftRect, frontend.rightRect,
                                 input.frameId, true);
    state.m_lkTracks =
        SelectLkTracksGridBalanced(state.m_lkTracks, frontend.leftRect.size());
    state.m_lkPrevLeft = frontend.leftRect.clone();
    state.m_lkPrevRight = frontend.rightRect.clone();
    state.m_lkTwc = Sophus::SE3f();
    ResetKltLoopClosureState(state);
    state.m_lkHavePrev = true;
    state.m_lkFrameCount = 1;
    if (extractFeatures) {
      CopyLkTrackFeaturesToOutput(state.m_lkTracks, out);
    }
  } else {
    std::vector<cv::Point3f> objectPoints =
        std::move(frontend.observations.pnp.objectPoints);
    std::vector<cv::Point2f> imagePoints =
        std::move(frontend.observations.pnp.imagePoints);
    std::vector<LkStereoTrack> trackedTracks =
        std::move(frontend.observations.trackedTracks);

    const KltPnpCameraIntrinsics camera{state.m_lkFx, state.m_lkFy,
                                        state.m_lkCx, state.m_lkCy};
    const KltPnpPoseEstimateResult poseEstimate =
        EstimateKltPnpPoseDelta(objectPoints, imagePoints,
                                MakeKltContinuousPnpPoseEstimatorOptions(
                                    camera, frontend.horizontalLateralFlow),
                                state.VisualPnpPoseBackend());
    int inlierCount = poseEstimate.inlierCount;
    std::vector<LkStereoTrack> inlierTracks =
        KeepLkTracksByIndices(trackedTracks, poseEstimate.inlierIndices);
    if (!inlierTracks.empty()) {
      trackedTracks = std::move(inlierTracks);
    }
    if (poseEstimate.poseUpdated) {
      state.m_lkTwc = state.m_lkTwc * poseEstimate.deltaTwc;
    }

    out.matchesInliers = inlierCount;
    out.trackedMapPointCount = static_cast<uint32_t>(inlierCount);
    out.localMapPointCount = static_cast<uint32_t>(objectPoints.size());
    UpdateLkTracksAfterPoseEstimate(state, frontend.leftRect,
                                    frontend.rightRect, input.frameId,
                                    std::move(trackedTracks), inlierCount);
    if (extractFeatures) {
      CopyLkTrackFeaturesToOutput(state.m_lkTracks, out);
    }
    state.m_lkPrevLeft = frontend.leftRect.clone();
    state.m_lkPrevRight = frontend.rightRect.clone();
    ++state.m_lkFrameCount;
  }

  const Sophus::SE3f outputTwc = ApplyKltLoopClosure(
      state, frontend.leftRect, input.frameId, state.m_lkTwc);
  state.CopyVisualFeatureStatsToOutput(out);
  out.pose = PoseFromTwc(outputTwc);
  return out;
}

core::ports::SlamOutput
KltSlamEngine::ProcessPerFrameKlt(const core::ports::SlamInputBatch &input,
                                  bool extractFeatures) {
  SlamModeSharedState &state = *m_state;
  core::ports::SlamOutput out = MakeOkSlamOutput(input);
  state.ResetVisualFeatureStats();

  KltPerFrameFrontendResult frontend = RunKltPerFrameFrontend(
      state, input.stereo.left.gray, input.stereo.right.gray);
  out.inputPrepareMs = frontend.inputPrepareMs;
  out.lkRectifyMs = frontend.rectifyMs;
  out.lkDisparityMs = frontend.disparityMs;
  out.lkGfttMs = frontend.gfttMs;
  out.lkFlowMs = frontend.flowMs;
  out.lkCandidateMs = frontend.candidateMs;

  if (!frontend.valid) {
    MarkSlamOutputPoseLost(out, core::ports::kSlamTrackingLost);
    return out;
  }

  if (!state.m_lkHavePrev) {
    state.m_lkTwc = Sophus::SE3f();
    UpdateKltPerFrameReferenceFrame(state, frontend);
    state.m_lkHavePrev = true;
    state.m_lkFrameCount = 1;
  } else {
    std::vector<cv::Point3f> objectPoints =
        std::move(frontend.observations.objectPoints);
    std::vector<cv::Point2f> imagePoints =
        std::move(frontend.observations.imagePoints);

    int inlierCount = 0;
    bool poseUpdated = false;
    const auto pnpStartTp = std::chrono::steady_clock::now();
    {
      const KltPnpCameraIntrinsics camera{state.m_lkFx, state.m_lkFy,
                                          state.m_lkCx, state.m_lkCy};
      const KltPnpPoseEstimateResult poseEstimate = EstimateKltPnpPoseDelta(
          objectPoints, imagePoints,
          MakeKltPerFramePnpPoseEstimatorOptions(
              camera, frontend.preferAcceleratedPnpDefaults),
          state.VisualPnpPoseBackend());
      inlierCount = poseEstimate.inlierCount;
      if (poseEstimate.poseUpdated) {
        if (frontend.useKeyframeReference) {
          state.m_lkTwc =
              state.m_lkPerFrameReferenceTwc * poseEstimate.deltaTwc;
        } else {
          state.m_lkTwc = state.m_lkTwc * poseEstimate.deltaTwc;
        }
        poseUpdated = true;
      }
    }
    const auto pnpEndTp = std::chrono::steady_clock::now();
    out.lkPnpMs =
        std::chrono::duration<double, std::milli>(pnpEndTp - pnpStartTp)
            .count();
    out.frontendMs = out.lkDisparityMs + out.lkGfttMs + out.lkFlowMs +
                     out.lkCandidateMs + out.lkPnpMs;

    out.matchesInliers = inlierCount;
    out.trackedMapPointCount = static_cast<uint32_t>(inlierCount);
    out.localMapPointCount = static_cast<uint32_t>(objectPoints.size());
    if (extractFeatures) {
      out.leftFeatures = std::move(frontend.currentLeftPoints);
    }
    const auto updateStartTp = std::chrono::steady_clock::now();
    if (ShouldRefreshKltPerFrameReference(state, frontend, inlierCount)) {
      UpdateKltPerFrameReferenceFrame(state, frontend);
    }
    out.lkUpdateMs = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - updateStartTp)
                         .count();
    if (!poseUpdated) {
      MarkSlamOutputPoseLost(out, core::ports::kSlamTrackingLost);
    }
    ++state.m_lkFrameCount;
  }

  out.pose = PoseFromTwc(state.m_lkTwc);
  return out;
}

} // namespace smartdrone::adapters::slam
