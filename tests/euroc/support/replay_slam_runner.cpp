#include "support/replay_slam_runner.h"

#include <chrono>

namespace smartdrone::tests {

namespace {

double DurationMs(const std::chrono::steady_clock::time_point &start,
                  const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

} // namespace

ReplaySlamRunner::ReplaySlamRunner(smartdrone::core::ports::ICameraProvider &camera,
                                   smartdrone::core::ports::IImuProvider &imu,
                                   smartdrone::core::ports::ISlamEngine &slamEngine, ReplaySlamRunnerConfig cfg)
    : m_camera(camera), m_imu(imu), m_slamEngine(slamEngine), m_cfg(cfg),
      m_pipeline(smartdrone::core::application::PerceptionPipelineConfig{cfg.cameraFps, cfg.preferLatestFrame})
{
}

std::vector<ReplayPoseSample> ReplaySlamRunner::Run(
    size_t maxFrames, smartdrone::core::application::FrameTimingTracker *timingTracker)
{
    std::vector<ReplayPoseSample> outputs;
    if (maxFrames > 0) {
        outputs.reserve(maxFrames);
    }

    if (!m_camera.Start()) {
        return outputs;
    }
    if (m_cfg.useImu && !m_imu.Start()) {
        m_camera.Stop();
        return outputs;
    }
    if (!m_slamEngine.Start()) {
        m_camera.Stop();
        if (m_cfg.useImu) {
            m_imu.Stop();
        }
        return outputs;
    }

    while (maxFrames == 0 || outputs.size() < maxFrames) {
        smartdrone::core::application::StereoBatch batch{};
        const auto acquireStart = std::chrono::steady_clock::now();
        const auto acquireStatus = m_pipeline.AcquireNextStereoBatch(m_camera, m_cfg.slamInputFps, m_cfg.timeoutMs,
                                                                     batch, timingTracker);
        const auto acquireEnd = std::chrono::steady_clock::now();
        if (acquireStatus == smartdrone::core::application::StereoAcquireStatus::Timeout) {
            break;
        }
        if (acquireStatus == smartdrone::core::application::StereoAcquireStatus::CameraUnhealthy) {
            break;
        }
        if (acquireStatus == smartdrone::core::application::StereoAcquireStatus::DroppedByRateLimiter) {
            continue;
        }

        smartdrone::core::ports::SlamInputBatch input{};
        input.stereo = batch.stereo;
        input.frameId = batch.frameId;
        input.captureTimestampNs = batch.captureTimestampNs;
        input.frameTimeSec = static_cast<double>(batch.captureTimestampNs) * 1e-9;

        std::vector<smartdrone::core::ports::ImuReading> imuWindow;
        const auto imuStart = std::chrono::steady_clock::now();
        if (m_cfg.useImu && m_lastFrameNs != 0) {
            imuWindow = m_imu.PopWindow(m_lastFrameNs, batch.captureTimestampNs);
            input.imu = ToOrbImuPoints(imuWindow);
        }
        const auto imuEnd = std::chrono::steady_clock::now();
        m_lastFrameNs = batch.captureTimestampNs;

        const auto slamStart = std::chrono::steady_clock::now();
        const auto output = m_slamEngine.Process(input, m_cfg.extractFeatures, m_cfg.extractPointCloud);
        const auto slamEnd = std::chrono::steady_clock::now();
        ReplayPoseSample sample{};
        sample.frameId = output.frameId;
        sample.captureTimestampNs = output.captureTimestampNs;
        sample.trackingState = output.trackingState;
        sample.mapId = output.mapId;
        sample.poseValid = output.poseValid;
        sample.pose = output.pose;
        sample.imuSampleCount = imuWindow.size();
        sample.usedSuperPointFrontend = output.usedSuperPointFrontend;
        sample.superpointRawLeftCount = output.superpointRawLeftCount;
        sample.superpointRawRightCount = output.superpointRawRightCount;
        sample.superpointMatchedStereoCount = output.superpointMatchedStereoCount;
        sample.superpointInjectedLeftCount = output.superpointInjectedLeftCount;
        sample.superpointInjectedRightCount = output.superpointInjectedRightCount;
        sample.superpointLightGlueEveryN = output.superpointLightGlueEveryN;
        sample.superpointFrontendMs = output.superpointFrontendMs;
        sample.superpointStereoMatchMs = output.superpointStereoMatchMs;
        sample.superpointTotalMs = output.superpointTotalMs;
        sample.replayAcquireMs = DurationMs(acquireStart, acquireEnd);
        sample.replayImuMs = DurationMs(imuStart, imuEnd);
        sample.slamTotalMs = DurationMs(slamStart, slamEnd);
        sample.inputPrepareMs = output.inputPrepareMs;
        sample.frontendMs = output.frontendMs;
        sample.stereoPairMs = output.stereoPairMs;
        sample.externalPackMs = output.externalPackMs;
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
        sample.matchesInliers = output.matchesInliers;
        sample.trackedMapPointCount = output.trackedMapPointCount;
        sample.localMapPointCount = output.localMapPointCount;
        outputs.push_back(sample);
    }

    if (m_cfg.shutdownEngineOnFinish) {
        m_slamEngine.Stop();
    }
    if (m_cfg.useImu) {
        m_imu.Stop();
    }
    m_camera.Stop();
    return outputs;
}

std::vector<ORB_SLAM3::IMU::Point> ReplaySlamRunner::ToOrbImuPoints(
    const std::vector<smartdrone::core::ports::ImuReading> &readings)
{
    std::vector<ORB_SLAM3::IMU::Point> out;
    out.reserve(readings.size());
    for (const auto &reading : readings) {
        out.emplace_back(cv::Point3f(reading.ax, reading.ay, reading.az),
                         cv::Point3f(reading.gx, reading.gy, reading.gz),
                         static_cast<double>(reading.timestampNs) * 1e-9);
    }
    return out;
}

} // namespace smartdrone::tests
