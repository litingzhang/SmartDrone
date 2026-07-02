#include "core/application/session/slam/slam_frame_port_set.h"

#include <cstdint>
#include <memory>

#include "core/application/session/stream/preview_output_port.h"
#include "core/application/session/slam/slam_settings_loader.h"
#include "core/ports/slam_session_telemetry.h"

namespace SmartDrone::Core::Application {
namespace {

constexpr uint64_t RANGE_SENSOR_MAX_AGE_US = 200000ULL;

PosePostprocessor::ReadRangeSensorFn
BuildRangeSensorReader(
    SmartDrone::Core::Ports::ISlamSessionTelemetryPort &telemetry)
{
    return [&telemetry](PosePostprocessor::RangeSensorSnapshot &snapshot) {
        SmartDrone::Core::Ports::SlamRangeSensor range{};
        if (!telemetry.GetDownwardRange(range, RANGE_SENSOR_MAX_AGE_US)) {
            return false;
        }
        snapshot.currentDistance = range.currentDistance;
        snapshot.signalQuality = range.signalQuality;
        return true;
    };
}

} // namespace

SlamFramePortSet::SlamFramePortSet(SlamFramePortSetConfig config)
    : m_cfg(config)
{
}

void SlamFramePortSet::Prepare()
{
    (void)InputPort();
    (void)BackendMaintenancePort();
    (void)TrackingPort();
    (void)PosePostprocessPort();
    (void)OutputPort();
}

SlamFrameInputPort &SlamFramePortSet::InputPort()
{
    if (!m_inputPort) {
        m_inputPort = std::make_unique<SlamFrameInputPort>(
            InputContext(), m_cfg.inputState, m_cfg.sharedState,
            m_cfg.outputState);
    }
    return *m_inputPort;
}

SlamBackendMaintenancePort &SlamFramePortSet::BackendMaintenancePort()
{
    if (!m_backendMaintenancePort) {
        m_backendMaintenancePort =
            std::make_unique<SlamBackendMaintenancePort>(
                m_cfg.slamBackendMaintenance);
    }
    return *m_backendMaintenancePort;
}

SlamFrameTrackingPort &SlamFramePortSet::TrackingPort()
{
    if (!m_trackingPort) {
        m_trackingPort =
            std::make_unique<SlamFrameTrackingPort>(TrackingContext());
    }
    return *m_trackingPort;
}

SlamFramePosePostprocessPort &SlamFramePortSet::PosePostprocessPort()
{
    if (!m_posePostprocessPort) {
        m_posePostprocessPort =
            std::make_unique<SlamFramePosePostprocessPort>(
                PosePostprocessContext(), m_cfg.posePostprocessState,
                m_cfg.sharedState);
    }
    return *m_posePostprocessPort;
}

SlamFrameOutputPort &SlamFramePortSet::OutputPort()
{
    if (!m_outputPort) {
        m_outputPort =
            std::make_unique<SlamFrameOutputPort>(OutputContext(),
                                                  m_cfg.outputState);
    }
    return *m_outputPort;
}

SlamFrameProcessingContext &SlamFramePortSet::InputContext()
{
    if (!m_inputContext) {
        m_inputContext = std::make_unique<SlamFrameProcessingContext>(
            SlamFrameProcessingContext{
                m_cfg.aliases, m_cfg.monoMode, m_cfg.useImu, m_cfg.tuning,
                m_cfg.livePose, &m_cfg.slamControl, m_cfg.cameraProvider,
                m_cfg.imuProvider, m_cfg.frameTimingTracker,
                m_cfg.perceptionPipeline, m_cfg.autoSlamModeController});
    }
    return *m_inputContext;
}

SlamFrameTrackingContext &SlamFramePortSet::TrackingContext()
{
    if (!m_trackingContext) {
        m_trackingContext = std::make_unique<SlamFrameTrackingContext>(
            SlamFrameTrackingContext{m_cfg.slamEngine, &m_cfg.slamControl,
                                     m_cfg.frameTimingTracker});
    }
    return *m_trackingContext;
}

SlamFramePosePostprocessContext &SlamFramePortSet::PosePostprocessContext()
{
    if (!m_posePostprocessContext) {
        m_posePostprocessContext =
            std::make_unique<SlamFramePosePostprocessContext>(
                SlamFramePosePostprocessContext{
                    m_cfg.monoMode, m_cfg.useImu, m_cfg.tuning,
                    m_cfg.livePose, BuildRangeSensorReader(m_cfg.telemetry),
                    &m_cfg.slamControl, m_cfg.posePostprocessor,
                    m_cfg.autoSlamModeController,
                    m_cfg.stereoBodyExtrinsics});
    }
    return *m_posePostprocessContext;
}

SlamFrameOutputContext &SlamFramePortSet::OutputContext()
{
    if (!m_outputContext) {
        m_outputContext = std::make_unique<SlamFrameOutputContext>(
            SlamFrameOutputContext{m_cfg.aliases, m_cfg.monoMode,
                                   m_cfg.tuning, m_cfg.livePose,
                                   m_cfg.posePublisher, m_cfg.previewOutput});
    }
    return *m_outputContext;
}

} // namespace SmartDrone::Core::Application
