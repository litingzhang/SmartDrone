#include "core/application/session/slam/slam_session_resource_lifecycle.h"

#include <iostream>
#include <utility>

#include "core/application/sensors/imu_sensor_poller.h"
#include "core/application/session/stream/preview_output_port.h"
#include "core/application/session/slam/slam_runtime_control_port.h"
#include "core/ports/camera_provider.h"
#include "core/ports/slam_engine.h"
#include "core/ports/visual_feature_frontend.h"

namespace SmartDrone::Core::Application {

SlamSessionResourceLifecycle::SlamSessionResourceLifecycle(
    SlamSessionResourceLifecycleConfig config)
    : m_config(std::move(config))
{
}

SlamSessionResourceLifecycle::~SlamSessionResourceLifecycle()
{
    Stop(false);
}

SlamSessionResourceStartResult SlamSessionResourceLifecycle::StartFrameResources()
{
    SlamSessionResourceStartResult result;
    StartVisualFeatureFrontend(result);
    if (!OpenUdp() || !StartImuPoller() || !OpenCamera()) {
        Stop(false);
        return result;
    }
    result.ok = true;
    return result;
}

void SlamSessionResourceLifecycle::Stop(bool logProgress)
{
    if (m_cameraOpen && m_config.cameraProvider != nullptr) {
        m_config.cameraProvider->Close();
        m_cameraOpen = false;
        if (logProgress) {
            std::cerr << "[session] slam camera closed\n";
        }
    }
    if (m_config.useImu && m_config.imuPoller != nullptr) {
        m_config.imuPoller->Stop();
        if (logProgress) {
            std::cerr << "[session] slam imu stopped\n";
        }
    }
    if (m_udpOpen && m_config.previewOutput != nullptr) {
        m_config.previewOutput->Close();
        m_udpOpen = false;
        if (logProgress) {
            std::cerr << "[session] slam udp closed\n";
        }
    }
    if (m_slamStarted) {
        m_config.slamEngine.Stop();
        m_slamStarted = false;
    }
    if (m_visualFeatureSession != nullptr) {
        m_visualFeatureSession->Stop();
        m_visualFeatureSession.reset();
    }
}

bool SlamSessionResourceLifecycle::StartSlamEngine()
{
    if (m_config.slamEngine.Start()) {
        m_slamStarted = true;
        return true;
    }
    return false;
}

void SlamSessionResourceLifecycle::StartVisualFeatureFrontend(
    SlamSessionResourceStartResult &result)
{
    if (!m_config.startVisualFrontend) {
        return;
    }
    auto startResult = m_config.startVisualFrontend();
    result.featureRouteAvailable = startResult.routeAvailable;
    result.featureClientMissing = startResult.clientMissing;
    result.featureStarted = startResult.started;
    result.featureRepoPath = startResult.repoPath;
    result.featureError = startResult.error;
    result.featureFrontend = startResult.frontend;
    if (!startResult.started) {
        return;
    }
    if (m_config.attachVisualFrontend) {
        m_config.attachVisualFrontend(startResult.frontend);
    }
    m_visualFeatureSession = std::move(startResult.session);
}

bool SlamSessionResourceLifecycle::OpenUdp()
{
    if (!m_config.udpEnabled) {
        return true;
    }
    if (m_config.previewOutput == nullptr) {
        return false;
    }
    if (m_config.previewOutput->Open(
            MakePreviewOutputOpenConfig(m_config.aliases),
            m_config.resolveUdpDestination)) {
        m_udpOpen = true;
        return true;
    }
    std::cerr << "[session] slam udp open failed\n";
    return false;
}

bool SlamSessionResourceLifecycle::StartImuPoller()
{
    if (!m_config.useImu) {
        return true;
    }
    return m_config.imuPoller != nullptr && m_config.imuPoller->Start();
}

bool SlamSessionResourceLifecycle::OpenCamera()
{
    if (m_config.cameraProvider != nullptr &&
        m_config.makeCameraOpenConfig &&
        m_config.cameraProvider->Open(
            m_config.makeCameraOpenConfig(m_config.aliases))) {
        m_cameraOpen = true;
        return true;
    }
    return false;
}

} // namespace SmartDrone::Core::Application
