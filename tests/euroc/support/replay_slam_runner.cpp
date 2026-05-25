#include "support/replay_slam_runner.h"

#include <algorithm>
#include <chrono>

namespace SmartDrone::Tests {

namespace {

double DurationMs(const std::chrono::steady_clock::time_point &start,
                  const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

} // namespace

ReplaySlamRunner::ReplaySlamRunner(
    SmartDrone::Core::Ports::ICameraProvider &camera,
    SmartDrone::Core::Ports::IImuProvider &imu,
    SmartDrone::Core::Ports::ISlamEngine &slamEngine,
    ReplaySlamRunnerConfig cfg)
    : m_camera(camera), m_imu(imu), m_slamEngine(slamEngine), m_cfg(cfg),
      m_pipeline(SmartDrone::Core::Application::PerceptionPipelineConfig{
          cfg.cameraFps, cfg.preferLatestFrame})
{
    m_slamControl =
        dynamic_cast<SmartDrone::Core::Ports::ISlamRuntimeControl *>(
            &m_slamEngine);
    m_backendStepEveryN = std::max(1, m_cfg.backendStepEveryN);
}

std::vector<ReplayPoseSample> ReplaySlamRunner::Run(
    size_t maxFrames,
    SmartDrone::Core::Application::FrameTimingTracker *timingTracker,
    const ReplayPoseSampleCallback &sampleCallback)
{
    std::vector<ReplayPoseSample> outputs;
    if (maxFrames > 0) {
        outputs.reserve(maxFrames);
    }
    if (!StartProviders()) {
        return outputs;
    }
    while (maxFrames == 0 || outputs.size() < maxFrames) {
        FrameRunResult result = RunFrame(timingTracker);
        if (!result.keepRunning) {
            break;
        }
        ReplayPoseSample &sample = result.sample;
        if (sampleCallback) {
            sampleCallback(sample);
        }
        outputs.push_back(sample);
    }
    StopProviders();
    return outputs;
}

bool ReplaySlamRunner::StartProviders()
{
    if (!m_camera.Start()) {
        return false;
    }
    if (m_cfg.useImu && !m_imu.Start()) {
        m_camera.Stop();
        return false;
    }
    if (m_slamEngine.Start()) {
        return true;
    }
    m_camera.Stop();
    if (m_cfg.useImu) {
        m_imu.Stop();
    }
    return false;
}

void ReplaySlamRunner::StopProviders()
{
    if (m_cfg.shutdownEngineOnFinish) {
        m_slamEngine.Stop();
    }
    if (m_cfg.useImu) {
        m_imu.Stop();
    }
    m_camera.Stop();
}

ReplaySlamRunner::FrameRunResult ReplaySlamRunner::RunFrame(
    SmartDrone::Core::Application::FrameTimingTracker *timingTracker)
{
    FrameRunContext context;
    context.timingTracker = timingTracker;
    if (!AcquireFrame(context)) {
        return {false, false, {}};
    }
    PrepareSlamInput(context);
    AttachImuWindow(context);
    ProcessSlamFrame(context);
    RunBackendStepIfNeeded(context);
    return {true, true, BuildReplayPoseSample(context)};
}

bool ReplaySlamRunner::AcquireFrame(FrameRunContext &context)
{
    context.acquireStart = std::chrono::steady_clock::now();
    const auto status = m_pipeline.AcquireNextStereoBatch(
        m_camera, m_cfg.slamInputFps, m_cfg.staleFrameThresholdMs,
        context.batch, context.timingTracker);
    context.acquireEnd = std::chrono::steady_clock::now();
    return status == SmartDrone::Core::Application::StereoAcquireStatus::Ok;
}

void ReplaySlamRunner::PrepareSlamInput(FrameRunContext &context)
{
    context.input.stereo = context.batch.stereo;
    context.input.frameId = context.batch.frameId;
    context.input.captureTimestampNs = context.batch.captureTimestampNs;
    context.input.frameTimeSec =
        static_cast<double>(context.batch.captureTimestampNs) * 1e-9;
}

void ReplaySlamRunner::AttachImuWindow(FrameRunContext &context)
{
    context.imuStart = std::chrono::steady_clock::now();
    if (m_cfg.useImu && m_lastFrameNs != 0) {
        context.imuWindow =
            m_imu.PopWindow(m_lastFrameNs, context.batch.captureTimestampNs);
        context.input.imu = context.imuWindow;
    }
    context.imuEnd = std::chrono::steady_clock::now();
    m_lastFrameNs = context.batch.captureTimestampNs;
}

void ReplaySlamRunner::ProcessSlamFrame(FrameRunContext &context)
{
    context.slamStart = std::chrono::steady_clock::now();
    context.output = m_slamEngine.Process(context.input, m_cfg.extractFeatures,
                                          m_cfg.extractPointCloud);
    context.slamEnd = std::chrono::steady_clock::now();
}

void ReplaySlamRunner::RunBackendStepIfNeeded(FrameRunContext &context)
{
    context.ranBackendStep =
        m_slamControl != nullptr &&
        context.output.frameId % static_cast<uint64_t>(m_backendStepEveryN) == 0U;
    context.backendStart = std::chrono::steady_clock::now();
    if (context.ranBackendStep) {
        m_slamControl->StepBackend();
    }
    context.backendEnd = std::chrono::steady_clock::now();
}

ReplayPoseSample ReplaySlamRunner::BuildReplayPoseSample(
    const FrameRunContext &context)
{
    const auto &output = context.output;
    ReplayPoseSample sample{};
    sample.frameId = output.frameId;
    sample.captureTimestampNs = output.captureTimestampNs;
    sample.trackingState = output.trackingState;
    sample.mapId = output.mapId;
    sample.poseValid = output.poseValid;
    sample.pose = output.pose;
    sample.imuSampleCount = context.imuWindow.size();
    CopyVisualFeatureFields(sample, output);
    CopyTimingFields(sample, context);
    CopyMappingFields(sample, output);
    sample.stereoFeatureInitFrameId = output.stereoFeatureInitFrameId;
    sample.stereoFeatureInjected = output.stereoFeatureInjected;
    sample.stereoFeatureBootstrap = output.stereoFeatureBootstrap;
    sample.stereoFeatureStabilizing = output.stereoFeatureStabilizing;
    sample.realtimePoseQualityGate = output.realtimePoseQualityGate;
    sample.rawPoseStepMeters = output.rawPoseStepMeters;
    sample.gatedPoseStepMeters = output.gatedPoseStepMeters;
    return sample;
}

void ReplaySlamRunner::CopyVisualFeatureFields(
    ReplayPoseSample &sample,
    const SmartDrone::Core::Ports::SlamOutput &output)
{
    sample.usedVisualFeatureFrontend = output.usedVisualFeatureFrontend;
    sample.visualFeatureRawLeftCount = output.visualFeatureRawLeftCount;
    sample.visualFeatureRawRightCount = output.visualFeatureRawRightCount;
    sample.visualFeatureMatchedStereoCount =
        output.visualFeatureMatchedStereoCount;
    sample.visualFeatureInjectedLeftCount =
        output.visualFeatureInjectedLeftCount;
    sample.visualFeatureInjectedRightCount =
        output.visualFeatureInjectedRightCount;
    sample.visualFeatureObservationHash = output.visualFeatureObservationHash;
    sample.visualFeatureMatchEveryN = output.visualFeatureMatchEveryN;
    sample.visualFeatureFrontendMs = output.visualFeatureFrontendMs;
    sample.visualFeatureStereoMatchMs = output.visualFeatureStereoMatchMs;
    sample.visualFeatureTotalMs = output.visualFeatureTotalMs;
}

void ReplaySlamRunner::CopyTimingFields(
    ReplayPoseSample &sample, const FrameRunContext &context)
{
    const auto &output = context.output;
    sample.replayAcquireMs =
        DurationMs(context.acquireStart, context.acquireEnd);
    sample.replayImuMs = DurationMs(context.imuStart, context.imuEnd);
    sample.slamTotalMs = DurationMs(context.slamStart, context.slamEnd);
    sample.slamBackendStepMs =
        context.ranBackendStep
            ? DurationMs(context.backendStart, context.backendEnd)
            : 0.0;
    sample.inputPrepareMs = output.inputPrepareMs;
    sample.frontendMs = output.frontendMs;
    sample.stereoPairMs = output.stereoPairMs;
    sample.featurePackMs = output.featurePackMs;
    sample.monoAugmentMs = output.monoAugmentMs;
    sample.lkRectifyMs = output.lkRectifyMs;
    sample.lkDisparityMs = output.lkDisparityMs;
    sample.lkGfttMs = output.lkGfttMs;
    sample.lkFlowMs = output.lkFlowMs;
    sample.lkCandidateMs = output.lkCandidateMs;
    sample.lkPnpMs = output.lkPnpMs;
    sample.lkUpdateMs = output.lkUpdateMs;
    sample.orbTrackMs = output.orbTrackMs;
    sample.orbExtractMs = output.orbExtractMs;
    sample.orbStereoMatchMs = output.orbStereoMatchMs;
}

void ReplaySlamRunner::CopyMappingFields(
    ReplayPoseSample &sample,
    const SmartDrone::Core::Ports::SlamOutput &output)
{
    sample.localMappingWaitMs = output.localMappingWaitMs;
    sample.localMappingWaitQueueBefore = output.localMappingWaitQueueBefore;
    sample.localMappingWaitQueueAfter = output.localMappingWaitQueueAfter;
    sample.localMappingWaitTimeoutMs = output.localMappingWaitTimeoutMs;
    sample.localMappingWaitRequested = output.localMappingWaitRequested;
    sample.localMappingWaitTimedOut = output.localMappingWaitTimedOut;
    sample.localMappingAcceptingBefore = output.localMappingAcceptingBefore;
    sample.localMappingAcceptingAfter = output.localMappingAcceptingAfter;
    sample.matchesInliers = output.matchesInliers;
    sample.trackedMapPointCount = output.trackedMapPointCount;
    sample.localMapPointCount = output.localMapPointCount;
    sample.localMapPointHash = output.localMapPointHash;
    sample.matchedMapPointHashBeforePoseOptimization =
        output.matchedMapPointHashBeforePoseOptimization;
    sample.trackedMapPointHash = output.trackedMapPointHash;
    sample.closeMapPointCount = output.closeMapPointCount;
    sample.orbFrameId = output.orbFrameId;
    sample.referenceKeyFrameId = output.referenceKeyFrameId;
    sample.lastKeyFrameId = output.lastKeyFrameId;
    sample.lastKeyFrameFrameId = output.lastKeyFrameFrameId;
    sample.keyFramesInMap = output.keyFramesInMap;
}

} // namespace SmartDrone::Tests
