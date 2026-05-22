#include "core/application/session/calib/calib_session_port_set.h"

#include <iostream>
#include <mutex>
#include <utility>

#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/application_runtime_factories.h"
#include "core/application/session/calib/calib_camera_input_port.h"
#include "core/application/session/calib/calib_imu_sample_port.h"
#include "core/application/session/calib/calib_preview_port.h"
#include "core/application/session/calib/calib_save_pacing_port.h"
#include "core/application/session/calib/calib_storage_port.h"

namespace SmartDrone::Core::Application {

class CalibSessionPortSet::Impl final {
  public:
    explicit Impl(CalibSessionPortSetConfig config)
        : m_aliases(config.aliases),
          m_root(std::move(config.root)),
          m_factories(config.factories)
    {
    }

    CalibSessionPortOpenResult Open()
    {
        m_storagePort.reset(new CalibStoragePort({m_root}));
        if (!m_storagePort->Open()) {
            return {CalibSessionPortOpenStatus::StorageFailed};
        }
        std::cerr << "[calib] out=" << m_storagePort->OutputRoot() << "\n";
        if (!OpenPreviewPort()) {
            return {CalibSessionPortOpenStatus::PreviewFailed};
        }
        if (!OpenCameraPort()) {
            return {CalibSessionPortOpenStatus::CameraFailed};
        }
        ConfigureDataPorts();
        return {CalibSessionPortOpenStatus::Ready};
    }

    bool ShouldFinishCapture(int maxFrames) const
    {
        return maxFrames > 0 && m_storagePort &&
               m_storagePort->SavedCount() >= maxFrames;
    }

    CalibFrameCaptureResult TryCaptureFrame(CalibStereoFrame &frame)
    {
        std::lock_guard<std::mutex> lock(m_cameraPortMu);
        if (!m_cameraInput || !m_cameraInput->Opened()) {
            return {CalibFrameCaptureStatus::SessionAbort};
        }
        return m_cameraInput->TryCaptureFrame(frame);
    }

    bool TryBuildSavePair(std::shared_ptr<CalibStereoFrame> frame,
                          CalibSavePair &savePair)
    {
        if (!m_savePacing) {
            return false;
        }
        return m_savePacing->TryBuildSavePair(std::move(frame), savePair);
    }

    bool WriteSavePair(const CalibSavePair &pair)
    {
        if (!m_storagePort) {
            return false;
        }
        return m_storagePort->WriteSavePair(pair);
    }

    bool EnqueuePreview(const CalibStereoFrame &frame)
    {
        std::lock_guard<std::mutex> lock(m_previewPortMu);
        if (m_previewPort) {
            return m_previewPort->Enqueue(frame);
        }
        return false;
    }

    CalibImuSampleResult StepImuSample()
    {
        std::lock_guard<std::mutex> lock(m_imuPortMu);
        if (!m_imuSamplePort) {
            return {CalibImuSampleStatus::Pending};
        }
        const CalibImuSamplePortResult result =
            m_imuSamplePort->ReadSample();
        return WriteReadyImuSample(result);
    }

    void StopAndFlush()
    {
        StopCamera();
        StopImu();
        StopPreview();
        FlushAndCloseStorage();
    }

    void LogFinalStatus() const
    {
        const int saved = m_storagePort ? m_storagePort->SavedCount() : 0;
        const std::string outRoot =
            m_storagePort ? m_storagePort->OutputRoot() : std::string{};
        std::cerr << "[calib] out=" << outRoot
                  << " saved=" << saved
                  << " imuOk=" << (m_imuOk.load() ? "true" : "false")
                  << "\n";
    }

  private:
    bool OpenPreviewPort()
    {
        m_previewPort.reset(new CalibPreviewPort(m_aliases, m_factories));
        return m_previewPort->Open();
    }

    bool OpenCameraPort()
    {
        m_cameraInput.reset(new CalibCameraInputPort(m_aliases, m_factories));
        return m_cameraInput->Open();
    }

    void ConfigureDataPorts()
    {
        m_savePacing.reset(new CalibSavePacingPort(
            {m_aliases.slamInputFps, m_aliases.fps,
             m_storagePort->Cam0Dir(),
             m_storagePort->Cam1Dir()}));
        m_imuSamplePort.reset(new CalibImuSamplePort(m_aliases));
    }

    CalibImuSampleResult WriteReadyImuSample(
        const CalibImuSamplePortResult &result)
    {
        if (result.status == CalibImuSamplePortStatus::Pending) {
            return {CalibImuSampleStatus::Pending};
        }
        if (result.status == CalibImuSamplePortStatus::Failed ||
            !WriteImuSample(result.sample)) {
            return {CalibImuSampleStatus::Failed};
        }
        return {CalibImuSampleStatus::Written};
    }

    bool WriteImuSample(const ImuSample &sample)
    {
        if (!m_storagePort || !m_storagePort->WriteImuSample(sample)) {
            return false;
        }
        m_imuOk.store(true, std::memory_order_relaxed);
        return true;
    }

    void StopCamera()
    {
        std::lock_guard<std::mutex> lock(m_cameraPortMu);
        if (!m_cameraInput) {
            return;
        }
        m_cameraInput->Stop();
        m_cameraInput.reset();
    }

    void StopImu()
    {
        std::lock_guard<std::mutex> lock(m_imuPortMu);
        if (m_imuSamplePort) {
            m_imuSamplePort->Stop();
        }
        m_imuSamplePort.reset();
    }

    void StopPreview()
    {
        std::lock_guard<std::mutex> lock(m_previewPortMu);
        if (!m_previewPort) {
            return;
        }
        m_previewPort->Close();
        m_previewPort.reset();
    }

    void FlushAndCloseStorage()
    {
        if (m_storagePort) {
            m_storagePort->FlushAndClose();
        }
    }

    MainRuntimeAliases m_aliases;
    std::string m_root;
    const ApplicationRuntimeFactories &m_factories;
    std::mutex m_cameraPortMu;
    std::mutex m_imuPortMu;
    std::mutex m_previewPortMu;
    std::unique_ptr<CalibCameraInputPort> m_cameraInput;
    std::unique_ptr<CalibImuSamplePort> m_imuSamplePort;
    std::unique_ptr<CalibPreviewPort> m_previewPort;
    std::unique_ptr<CalibSavePacingPort> m_savePacing;
    std::unique_ptr<CalibStoragePort> m_storagePort;
    std::atomic<bool> m_imuOk{false};
};

CalibSessionPortSet::CalibSessionPortSet(CalibSessionPortSetConfig config)
    : m_impl(new Impl(std::move(config)))
{
}

CalibSessionPortSet::~CalibSessionPortSet() = default;

CalibSessionPortOpenResult CalibSessionPortSet::Open()
{
    return m_impl->Open();
}

bool CalibSessionPortSet::ShouldFinishCapture(int maxFrames) const
{
    return m_impl->ShouldFinishCapture(maxFrames);
}

CalibFrameCaptureResult CalibSessionPortSet::TryCaptureFrame(
    CalibStereoFrame &frame)
{
    return m_impl->TryCaptureFrame(frame);
}

bool CalibSessionPortSet::TryBuildSavePair(
    std::shared_ptr<CalibStereoFrame> frame,
    CalibSavePair &savePair)
{
    return m_impl->TryBuildSavePair(std::move(frame), savePair);
}

bool CalibSessionPortSet::WriteSavePair(const CalibSavePair &pair)
{
    return m_impl->WriteSavePair(pair);
}

bool CalibSessionPortSet::EnqueuePreview(const CalibStereoFrame &frame)
{
    return m_impl->EnqueuePreview(frame);
}

CalibImuSampleResult CalibSessionPortSet::StepImuSample()
{
    return m_impl->StepImuSample();
}

void CalibSessionPortSet::StopAndFlush()
{
    m_impl->StopAndFlush();
}

void CalibSessionPortSet::LogFinalStatus() const
{
    m_impl->LogFinalStatus();
}

} // namespace SmartDrone::Core::Application
