#include "test_support/replay_slam_runner.h"

namespace smartdrone::tests {

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
        const auto acquireStatus = m_pipeline.AcquireNextStereoBatch(m_camera, m_cfg.slamInputFps, m_cfg.timeoutMs,
                                                                     batch, timingTracker);
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
        if (m_cfg.useImu && m_lastFrameNs != 0) {
            imuWindow = m_imu.PopWindow(m_lastFrameNs, batch.captureTimestampNs);
            input.imu = ToOrbImuPoints(imuWindow);
        }
        m_lastFrameNs = batch.captureTimestampNs;

        const auto output = m_slamEngine.Process(input, m_cfg.extractFeatures, m_cfg.extractPointCloud);
        outputs.push_back(
            {output.frameId, output.captureTimestampNs, output.trackingState, output.mapId, output.poseValid,
             output.pose, imuWindow.size()});
    }

    m_slamEngine.Stop();
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
