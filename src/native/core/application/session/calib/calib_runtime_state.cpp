#include "core/application/session/calib/calib_runtime_state.h"

#include <iostream>
#include <utility>

#include "common/tlv/tlv_protocol.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/application_runtime_factories.h"
#include "core/application/runtime/runtime_aliases.h"
#include "core/application/session/calib/calib_session_port_set.h"
#include "core/application/state/live_pose_state.h"
#include "core/domain/runtime_mode.h"

namespace SmartDrone::Core::Application {
namespace {

using ControllerMode = SmartDrone::Core::Domain::RuntimeMode;

} // namespace

class CalibRuntimeState::Impl final {
  public:
    explicit Impl(CalibRuntimeStateConfig config)
        : m_cfg(config.cfg),
          m_stop(config.stop),
          m_livePose(config.livePose),
          m_factories(config.factories)
    {
    }

    ~Impl()
    {
        Finalize(false);
    }

    bool EnsureStarted()
    {
        if (m_started.load(std::memory_order_acquire)) {
            return true;
        }
        if (m_startFailed.load(std::memory_order_acquire)) {
            return false;
        }

        m_aliases = BuildRuntimeAliases(m_cfg.app);
        PrintStartupConfig(m_cfg.app, m_aliases, m_factories.cameraProvider,
                           ControllerMode::Calib);
        m_livePose.SetRuntimeMode(RUNTIME_MODE_CALIB);

        const CalibSessionPortOpenResult openResult = OpenPorts();
        if (openResult.status != CalibSessionPortOpenStatus::Ready) {
            return HandleOpenFailure(openResult.status);
        }
        m_started.store(true, std::memory_order_release);
        return true;
    }

    bool ShouldFinishCapture() const
    {
        const CalibSessionPortSet *ports = Ports();
        return ports && ports->ShouldFinishCapture(m_cfg.calib.maxFrames);
    }

    CalibFrameCaptureResult TryCaptureFrame(CalibStereoFrame &frame)
    {
        CalibSessionPortSet *ports = Ports();
        if (!ports) {
            return {CalibFrameCaptureStatus::SessionAbort};
        }
        return ports->TryCaptureFrame(frame);
    }

    bool TryBuildSavePair(std::shared_ptr<CalibStereoFrame> frame,
                          CalibSavePair &savePair)
    {
        CalibSessionPortSet *ports = Ports();
        if (!ports) {
            return false;
        }
        return ports->TryBuildSavePair(std::move(frame), savePair);
    }

    bool WriteSavePair(const CalibSavePair &pair)
    {
        CalibSessionPortSet *ports = Ports();
        if (!ports) {
            return false;
        }
        return ports->WriteSavePair(pair);
    }

    bool WriteImuSample(const ImuSample &sample)
    {
        CalibSessionPortSet *ports = Ports();
        if (!ports) {
            return false;
        }
        return ports->WriteImuSample(sample);
    }

    bool EnqueuePreview(const CalibStereoFrame &frame)
    {
        CalibSessionPortSet *ports = Ports();
        if (ports) {
            return ports->EnqueuePreview(frame);
        }
        return false;
    }

    CalibImuSampleResult StepImuSample()
    {
        CalibSessionPortSet *ports = Ports();
        if (!ports) {
            return {CalibImuSampleStatus::Pending};
        }
        return ports->StepImuSample();
    }

    void RequestStop()
    {
        m_stop.store(true);
    }

    bool FlushStorage()
    {
        CalibSessionPortSet *ports = Ports();
        if (!ports) {
            ports = m_ports.get();
        }
        if (!ports) {
            return false;
        }
        return ports->FlushAndCloseStorage();
    }

    void MarkStorageFlushed()
    {
        m_storageFlushed.store(true, std::memory_order_release);
    }

    bool StorageFlushed() const
    {
        return m_storageFlushed.load(std::memory_order_acquire);
    }

    bool Finalized() const
    {
        return m_finalized.load(std::memory_order_acquire);
    }

    void FinalizeAfterStorageFlushed(bool sessionOk)
    {
        bool expected = false;
        if (!m_finalized.compare_exchange_strong(expected, true,
                                                 std::memory_order_acq_rel)) {
            return;
        }
        FinalizeNonStoragePorts(sessionOk);
    }

    void Finalize(bool sessionOk)
    {
        if (Finalized()) {
            return;
        }
        const bool storageOk = FlushStorage();
        MarkStorageFlushed();
        sessionOk = sessionOk && storageOk;
        FinalizeAfterStorageFlushed(sessionOk);
    }

  private:
    CalibSessionPortSet *Ports()
    {
        return m_portsView.load(std::memory_order_acquire);
    }

    const CalibSessionPortSet *Ports() const
    {
        return m_portsView.load(std::memory_order_acquire);
    }

    CalibSessionPortOpenResult OpenPorts()
    {
        m_ports.reset(
            new CalibSessionPortSet(
                {m_aliases, m_cfg.calib.root, m_factories}));
        CalibSessionPortOpenResult result = m_ports->Open();
        if (result.status == CalibSessionPortOpenStatus::Ready) {
            m_portsView.store(m_ports.get(), std::memory_order_release);
        }
        return result;
    }

    bool HandleOpenFailure(CalibSessionPortOpenStatus status)
    {
        m_startFailed.store(true, std::memory_order_release);
        if (status == CalibSessionPortOpenStatus::StorageFailed) {
            return false;
        }
        m_stop.store(true);
        Finalize(false);
        return false;
    }

    void FinalizeNonStoragePorts(bool sessionOk)
    {
        m_stop.store(true);
        m_portsView.store(nullptr, std::memory_order_release);
        if (m_ports) {
            m_ports->StopPorts();
            m_ports->LogFinalStatus();
        }
        m_livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
        std::cerr << "[session] calib exit ok="
                  << (sessionOk ? "true" : "false") << "\n";
    }

    UnifiedConfig m_cfg;
    std::atomic<bool> &m_stop;
    LivePoseState &m_livePose;
    const ApplicationRuntimeFactories &m_factories;
    MainRuntimeAliases m_aliases{};
    std::unique_ptr<CalibSessionPortSet> m_ports;
    std::atomic<CalibSessionPortSet *> m_portsView{nullptr};
    std::atomic<bool> m_storageFlushed{false};
    std::atomic<bool> m_started{false};
    std::atomic<bool> m_startFailed{false};
    std::atomic<bool> m_finalized{false};
};

CalibRuntimeState::CalibRuntimeState(CalibRuntimeStateConfig config)
    : m_impl(new Impl(std::move(config)))
{
}

CalibRuntimeState::~CalibRuntimeState() = default;

bool CalibRuntimeState::EnsureStarted()
{
    return m_impl->EnsureStarted();
}

bool CalibRuntimeState::ShouldFinishCapture() const
{
    return m_impl->ShouldFinishCapture();
}

CalibFrameCaptureResult CalibRuntimeState::TryCaptureFrame(
    CalibStereoFrame &frame)
{
    return m_impl->TryCaptureFrame(frame);
}

bool CalibRuntimeState::TryBuildSavePair(
    std::shared_ptr<CalibStereoFrame> frame,
    CalibSavePair &savePair)
{
    return m_impl->TryBuildSavePair(std::move(frame), savePair);
}

bool CalibRuntimeState::WriteSavePair(const CalibSavePair &pair)
{
    return m_impl->WriteSavePair(pair);
}

bool CalibRuntimeState::WriteImuSample(const ImuSample &sample)
{
    return m_impl->WriteImuSample(sample);
}

bool CalibRuntimeState::EnqueuePreview(const CalibStereoFrame &frame)
{
    return m_impl->EnqueuePreview(frame);
}

CalibImuSampleResult CalibRuntimeState::StepImuSample()
{
    return m_impl->StepImuSample();
}

void CalibRuntimeState::RequestStop()
{
    m_impl->RequestStop();
}

bool CalibRuntimeState::FlushStorage()
{
    return m_impl->FlushStorage();
}

void CalibRuntimeState::MarkStorageFlushed()
{
    m_impl->MarkStorageFlushed();
}

bool CalibRuntimeState::StorageFlushed() const
{
    return m_impl->StorageFlushed();
}

bool CalibRuntimeState::Finalized() const
{
    return m_impl->Finalized();
}

void CalibRuntimeState::FinalizeAfterStorageFlushed(bool sessionOk)
{
    m_impl->FinalizeAfterStorageFlushed(sessionOk);
}

void CalibRuntimeState::Finalize(bool sessionOk)
{
    m_impl->Finalize(sessionOk);
}

} // namespace SmartDrone::Core::Application
