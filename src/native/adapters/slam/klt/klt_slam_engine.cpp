#include "adapters/slam/klt/klt_slam_engine.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <utility>

#include "adapters/slam/klt/klt_continuous_frontend.h"
#include "adapters/slam/klt/klt_loop_closure_backend.h"
#include "adapters/slam/klt/klt_mode_utils.h"
#include "adapters/slam/klt/klt_per_frame_frontend.h"
#include "adapters/slam/klt/klt_pose_estimator.h"
#include "adapters/slam/klt/klt_track_manager.h"
#include "adapters/slam/engine/slam_engine_factory.h"
#include "adapters/slam/engine/slam_output_utils.h"
#include "adapters/slam/engine/slam_pose_utils.h"

namespace SmartDrone::Adapters::Slam {

namespace {

ControlledSlamEngine
CreateKltSlamEngine(const SlamEngineFactoryConfig &config)
{
    auto engine = std::make_unique<KltSlamEngine>(config.settingsPath);
    ControlledSlamEngine out{};
    out.control = engine.get();
    out.engine = std::move(engine);
    return out;
}

struct ContinuousKltFrameRequest {
    SlamModeSharedState *state{nullptr};
    KltContinuousFrontendResult *frontend{nullptr};
    uint64_t frameId{0};
    bool extractFeatures{false};
    bool extractPointCloud{false};
    Core::Ports::SlamOutput *out{nullptr};
};

struct PerFrameKltTrackingRequest {
    SlamModeSharedState *state{nullptr};
    KltPerFrameFrontendResult *frontend{nullptr};
    bool extractFeatures{false};
    bool extractPointCloud{false};
    Core::Ports::SlamOutput *out{nullptr};
};

struct ExportKltPointCloudRequest {
    const SlamModeSharedState *state{nullptr};
    const Sophus::SE3f *referenceTwc{nullptr};
    const std::vector<cv::Point3f> *objectPoints{nullptr};
    const std::vector<int> *inlierIndices{nullptr};
    bool extractPointCloud{false};
    Core::Ports::SlamOutput *out{nullptr};
};

void CopyKltFrontendTimings(const KltPerFrameFrontendResult &frontend,
                            Core::Ports::SlamOutput &out)
{
    out.inputPrepareMs = frontend.inputPrepareMs;
    out.lkRectifyMs = frontend.rectifyMs;
    out.lkDisparityMs = frontend.disparityMs;
    out.lkGfttMs = frontend.gfttMs;
    out.lkFlowMs = frontend.flowMs;
    out.lkCandidateMs = frontend.candidateMs;
}

bool CanUseKltPointCloud(const SlamModeSharedState &state)
{
    return state.m_lkFx > 0.0f && state.m_lkFy > 0.0f &&
           state.m_lkBaseline > 0.0f;
}

void AppendLocalPoint(const Sophus::SE3f &twc, const cv::Point3f &point,
                      Core::Ports::SlamOutput &out)
{
    const Eigen::Vector3f world =
        twc * Eigen::Vector3f(point.x, point.y, point.z);
    if (!std::isfinite(world.x()) || !std::isfinite(world.y()) ||
        !std::isfinite(world.z())) {
        return;
    }
    out.pointCloudXyz.push_back(world.x());
    out.pointCloudXyz.push_back(world.y());
    out.pointCloudXyz.push_back(world.z());
}

void AppendLocalPointCloud(const Sophus::SE3f &twc,
                           const std::vector<cv::Point3f> &points,
                           Core::Ports::SlamOutput &out)
{
    out.pointCloudXyz.reserve(points.size() * 3);
    for (const cv::Point3f &point : points) {
        AppendLocalPoint(twc, point, out);
    }
}

std::vector<cv::Point3f> KeepPointsByIndices(
    const std::vector<cv::Point3f> &points, const std::vector<int> &indices)
{
    std::vector<cv::Point3f> selected;
    selected.reserve(indices.size());
    for (int index : indices) {
        if (index >= 0 && static_cast<size_t>(index) < points.size()) {
            selected.push_back(points[static_cast<size_t>(index)]);
        }
    }
    return selected;
}

void MaybeExportKltPointCloud(const ExportKltPointCloudRequest &request)
{
    if (!request.extractPointCloud || !request.state || !request.referenceTwc ||
        !request.objectPoints || !request.out ||
        !CanUseKltPointCloud(*request.state) || request.objectPoints->empty()) {
        return;
    }
    AppendLocalPointCloud(*request.referenceTwc, *request.objectPoints,
                          *request.out);
}

void MaybeExportKltInlierPointCloud(const ExportKltPointCloudRequest &request)
{
    if (!request.inlierIndices || request.inlierIndices->empty() ||
        !request.objectPoints) {
        return;
    }
    const std::vector<cv::Point3f> inlierPoints =
        KeepPointsByIndices(*request.objectPoints, *request.inlierIndices);
    ExportKltPointCloudRequest inlierRequest = request;
    inlierRequest.objectPoints = &inlierPoints;
    MaybeExportKltPointCloud(inlierRequest);
}

void InitializeContinuousKltFrame(const ContinuousKltFrameRequest &request)
{
    SlamModeSharedState &state = *request.state;
    KltContinuousFrontendResult &frontend = *request.frontend;
    Core::Ports::SlamOutput &out = *request.out;
    RefreshLkStereoSeedsIfNeeded(state, frontend.leftRect, frontend.rightRect,
                                 request.frameId, true);
    state.m_lkTracks =
        SelectLkTracksGridBalanced(state.m_lkTracks, frontend.leftRect.size());
    state.m_lkPrevLeft = frontend.leftRect.clone();
    state.m_lkPrevRight = frontend.rightRect.clone();
    state.m_lkTwc = Sophus::SE3f();
    ResetKltLoopClosureState(state);
    state.m_lkHavePrev = true;
    state.m_lkFrameCount = 1;
    if (request.extractFeatures) {
        CopyLkTrackFeaturesToOutput(state.m_lkTracks, out);
    }
}

void ProcessContinuousKltTrackingFrame(
    const ContinuousKltFrameRequest &request)
{
    SlamModeSharedState &state = *request.state;
    KltContinuousFrontendResult &frontend = *request.frontend;
    Core::Ports::SlamOutput &out = *request.out;
    std::vector<cv::Point3f> objectPoints =
        std::move(frontend.observations.pnp.objectPoints);
    std::vector<cv::Point2f> imagePoints =
        std::move(frontend.observations.pnp.imagePoints);
    std::vector<LkStereoTrack> trackedTracks =
        std::move(frontend.observations.trackedTracks);
    const Sophus::SE3f previousTwc = state.m_lkTwc;

    const KltPnpCameraIntrinsics camera{state.m_lkFx, state.m_lkFy,
                                        state.m_lkCx, state.m_lkCy};
    const KltPnpPoseEstimateResult poseEstimate = EstimateKltPnpPoseDelta(
        objectPoints, imagePoints,
        MakeKltContinuousPnpPoseEstimatorOptions(
            camera, frontend.horizontalLateralFlow),
        state.VisualPnpPoseBackend());
    std::vector<LkStereoTrack> inlierTracks =
        KeepLkTracksByIndices(trackedTracks, poseEstimate.inlierIndices);
    if (!inlierTracks.empty()) {
        trackedTracks = std::move(inlierTracks);
    }
    if (poseEstimate.poseUpdated) {
        state.m_lkTwc = state.m_lkTwc * poseEstimate.deltaTwc;
    } else {
        MarkSlamOutputPoseLost(out, Core::Ports::SLAM_TRACKING_LOST);
    }

    out.matchesInliers = poseEstimate.inlierCount;
    out.trackedMapPointCount = static_cast<uint32_t>(poseEstimate.inlierCount);
    out.localMapPointCount = static_cast<uint32_t>(objectPoints.size());
    MaybeExportKltInlierPointCloud(
        {&state, &previousTwc, &objectPoints, &poseEstimate.inlierIndices,
         request.extractPointCloud, &out});
    UpdateLkTracksAfterPoseEstimate(
        {state, frontend.leftRect, frontend.rightRect, request.frameId,
         std::move(trackedTracks), poseEstimate.inlierCount});
    if (request.extractFeatures) {
        CopyLkTrackFeaturesToOutput(state.m_lkTracks, out);
    }
    state.m_lkPrevLeft = frontend.leftRect.clone();
    state.m_lkPrevRight = frontend.rightRect.clone();
    ++state.m_lkFrameCount;
}

void InitializePerFrameKltFrame(SlamModeSharedState &state,
                                const KltPerFrameFrontendResult &frontend)
{
    state.m_lkTwc = Sophus::SE3f();
    UpdateKltPerFrameReferenceFrame(state, frontend);
    state.m_lkHavePrev = true;
    state.m_lkFrameCount = 1;
}

KltPnpPoseEstimateResult EstimatePerFrameKltPose(
    SlamModeSharedState &state, const KltPerFrameFrontendResult &frontend,
    const std::vector<cv::Point3f> &objectPoints,
    const std::vector<cv::Point2f> &imagePoints)
{
    const KltPnpCameraIntrinsics camera{state.m_lkFx, state.m_lkFy,
                                        state.m_lkCx, state.m_lkCy};
    return EstimateKltPnpPoseDelta(
        objectPoints, imagePoints,
        MakeKltPerFramePnpPoseEstimatorOptions(
            camera, frontend.preferAcceleratedPnpDefaults),
        state.VisualPnpPoseBackend());
}

void ApplyPerFrameKltPose(SlamModeSharedState &state,
                          const KltPerFrameFrontendResult &frontend,
                          const KltPnpPoseEstimateResult &poseEstimate)
{
    if (!poseEstimate.poseUpdated) {
        return;
    }
    if (frontend.useKeyframeReference) {
        state.m_lkTwc = state.m_lkPerFrameReferenceTwc * poseEstimate.deltaTwc;
        return;
    }
    state.m_lkTwc = state.m_lkTwc * poseEstimate.deltaTwc;
}

void ProcessPerFrameKltTrackingFrame(
    const PerFrameKltTrackingRequest &request)
{
    SlamModeSharedState &state = *request.state;
    KltPerFrameFrontendResult &frontend = *request.frontend;
    Core::Ports::SlamOutput &out = *request.out;
    std::vector<cv::Point3f> objectPoints =
        std::move(frontend.observations.objectPoints);
    std::vector<cv::Point2f> imagePoints =
        std::move(frontend.observations.imagePoints);
    const Sophus::SE3f referenceTwc =
        frontend.useKeyframeReference ? state.m_lkPerFrameReferenceTwc
                                      : state.m_lkTwc;

    const auto pnpStartTp = std::chrono::steady_clock::now();
    const KltPnpPoseEstimateResult poseEstimate =
        EstimatePerFrameKltPose(state, frontend, objectPoints, imagePoints);
    ApplyPerFrameKltPose(state, frontend, poseEstimate);
    out.lkPnpMs = std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - pnpStartTp)
                      .count();
    out.frontendMs = out.lkDisparityMs + out.lkGfttMs + out.lkFlowMs +
                     out.lkCandidateMs + out.lkPnpMs;

    out.matchesInliers = poseEstimate.inlierCount;
    out.trackedMapPointCount = static_cast<uint32_t>(poseEstimate.inlierCount);
    out.localMapPointCount = static_cast<uint32_t>(objectPoints.size());
    MaybeExportKltInlierPointCloud(
        {&state, &referenceTwc, &objectPoints, &poseEstimate.inlierIndices,
         request.extractPointCloud, &out});
    if (request.extractFeatures) {
        out.leftFeatures = std::move(frontend.currentLeftPoints);
    }
    const auto updateStartTp = std::chrono::steady_clock::now();
    if (ShouldRefreshKltPerFrameReference(state, frontend,
                                          poseEstimate.inlierCount)) {
        UpdateKltPerFrameReferenceFrame(state, frontend);
    }
    out.lkUpdateMs = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - updateStartTp)
                         .count();
    if (!poseEstimate.poseUpdated) {
        MarkSlamOutputPoseLost(out, Core::Ports::SLAM_TRACKING_LOST);
    }
    ++state.m_lkFrameCount;
}

const SlamEngineFactoryRegistrar KLT_SLAM_ENGINE_REGISTRAR(
    SlamBackend::Klt, CreateKltSlamEngine);

} // namespace

KltSlamEngine::KltSlamEngine(std::string settingsPath)
    : m_state(std::make_unique<SlamModeSharedState>()),
      m_settingsPath(std::move(settingsPath))
{
}

KltSlamEngine::~KltSlamEngine() = default;

bool KltSlamEngine::Start()
{
    if (m_state == nullptr) {
        m_state = std::make_unique<SlamModeSharedState>();
    }
    m_state->LoadStereoCalibration(m_settingsPath);
    m_state->ResetTrackingState();
    return true;
}

void KltSlamEngine::Stop()
{
    if (m_state != nullptr) {
        m_state->ResetTrackingState();
    }
}

void KltSlamEngine::SetOperationMode(Core::Domain::SlamOperationMode mode)
{
    (void)mode;
}

void KltSlamEngine::SetFeatureFrontend(FeatureFrontend frontend)
{
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
    Core::Ports::IVisualFeatureFrontend *frontend)
{
    if (m_state != nullptr) {
        m_state->m_visualFeatureFrontend = frontend;
    }
}

void KltSlamEngine::SetVisualFeatureInputSizeLimit(int maxWidth,
                                                   int maxHeight)
{
    if (m_state != nullptr) {
        m_state->m_visualFeatureInputMaxWidth = std::max(0, maxWidth);
        m_state->m_visualFeatureInputMaxHeight = std::max(0, maxHeight);
    }
}

void KltSlamEngine::SetStereoVoLoopClosure(bool enabled, float scale,
                                           float relaxation)
{
    if (m_state == nullptr) {
        return;
    }
    m_state->m_lkLoop.enabled = enabled;
    m_state->m_lkLoop.scale = std::clamp(scale, 0.25f, 4.0f);
    m_state->m_lkLoop.relaxation = std::clamp(relaxation, 0.0f, 4.0f);
}

void KltSlamEngine::SetStereoVoPerFrameAcceleration(std::string acceleration)
{
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

Core::Ports::SlamOutput
KltSlamEngine::Process(const Core::Ports::SlamInputBatch &input,
                       bool extractFeatures, bool extractPointCloud)
{
    if (m_frontend == FeatureFrontend::LK) {
        return ProcessContinuousKlt(input, extractFeatures,
                                    extractPointCloud);
    }
    return ProcessPerFrameKlt(input, extractFeatures, extractPointCloud);
}

Core::Ports::SlamOutput
KltSlamEngine::ProcessContinuousKlt(const Core::Ports::SlamInputBatch &input,
                                    bool extractFeatures,
                                    bool extractPointCloud)
{
    SlamModeSharedState &state = *m_state;
    Core::Ports::SlamOutput out = MakeOkSlamOutput(input);
    state.ResetVisualFeatureStats();

    KltContinuousFrontendResult frontend = RunKltContinuousFrontend(
        state, input.stereo.left.gray, input.stereo.right.gray);
    if (!frontend.valid) {
        MarkSlamOutputPoseLost(out, Core::Ports::SLAM_TRACKING_LOST);
        return out;
    }

    if (!frontend.havePreviousFrame) {
        const ContinuousKltFrameRequest request{&state, &frontend, input.frameId,
                                                extractFeatures,
                                                extractPointCloud, &out};
        InitializeContinuousKltFrame(request);
    } else {
        const ContinuousKltFrameRequest request{&state, &frontend, input.frameId,
                                                extractFeatures,
                                                extractPointCloud, &out};
        ProcessContinuousKltTrackingFrame(request);
    }

    const Sophus::SE3f outputTwc = ApplyKltLoopClosure(
        state, frontend.leftRect, input.frameId, state.m_lkTwc);
    state.CopyVisualFeatureStatsToOutput(out);
    out.pose = PoseFromTwc(outputTwc);
    return out;
}

Core::Ports::SlamOutput
KltSlamEngine::ProcessPerFrameKlt(const Core::Ports::SlamInputBatch &input,
                                  bool extractFeatures,
                                  bool extractPointCloud)
{
    SlamModeSharedState &state = *m_state;
    Core::Ports::SlamOutput out = MakeOkSlamOutput(input);
    state.ResetVisualFeatureStats();

    KltPerFrameFrontendResult frontend = RunKltPerFrameFrontend(
        state, input.stereo.left.gray, input.stereo.right.gray);
    CopyKltFrontendTimings(frontend, out);

    if (!frontend.valid) {
        MarkSlamOutputPoseLost(out, Core::Ports::SLAM_TRACKING_LOST);
        return out;
    }

    if (!state.m_lkHavePrev) {
        InitializePerFrameKltFrame(state, frontend);
    } else {
        const PerFrameKltTrackingRequest request{&state, &frontend,
                                                 extractFeatures,
                                                 extractPointCloud, &out};
        ProcessPerFrameKltTrackingFrame(request);
    }

    out.pose = PoseFromTwc(state.m_lkTwc);
    return out;
}

} // namespace SmartDrone::Adapters::Slam
