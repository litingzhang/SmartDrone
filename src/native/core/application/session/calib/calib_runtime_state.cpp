#include "core/application/session/calib/calib_runtime_state.h"

#include <algorithm>
#include <iostream>
#include <mutex>
#include <utility>

#include "common/tlv/tlv_protocol.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/runtime_aliases.h"
#include "core/application/session/calib/calib_session_port_set.h"
#include "core/application/state/live_pose_state.h"
#include "core/domain/runtime_mode.h"

namespace smartdrone::core::application {
namespace {

using ControllerMode = smartdrone::core::domain::RuntimeMode;

} // namespace

class CalibRuntimeState::Impl final {
  public:
    explicit Impl(CalibRuntimeStateConfig config)
        : m_cfg(config.cfg),
          m_stop(config.stop),
          m_livePose(config.livePose)
    {
    }

    ~Impl()
    {
        Finalize(false);
    }

    bool EnsureStarted()
    {
        std::lock_guard<std::mutex> lock(m_mu);
        if (m_started) {
            return true;
        }
        if (m_startFailed) {
            return false;
        }

        m_aliases = BuildRuntimeAliases(m_cfg.app);
        PrintStartupConfig(m_cfg.app, m_aliases, ControllerMode::Calib);
        m_livePose.SetRuntimeMode(RUNTIME_MODE_CALIB);

        const CalibSessionPortOpenResult openResult = OpenPorts();
        if (openResult.status != CalibSessionPortOpenStatus::Ready) {
            return HandleOpenFailure(openResult.status);
        }
        m_started = true;
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

    bool Finalized() const
    {
        std::lock_guard<std::mutex> lock(m_mu);
        return m_finalized;
    }

    void Finalize(bool sessionOk)
    {
        std::lock_guard<std::mutex> lock(m_mu);
        FinalizeLocked(sessionOk);
    }

  private:
    CalibSessionPortSet *Ports()
    {
        std::lock_guard<std::mutex> lock(m_mu);
        return m_ports.get();
    }

    const CalibSessionPortSet *Ports() const
    {
        std::lock_guard<std::mutex> lock(m_mu);
        return m_ports.get();
    }

    CalibSessionPortOpenResult OpenPorts()
    {
        m_ports.reset(
            new CalibSessionPortSet({m_aliases, m_cfg.calib.root}));
        return m_ports->Open();
    }

    bool HandleOpenFailure(CalibSessionPortOpenStatus status)
    {
        m_startFailed = true;
        if (status == CalibSessionPortOpenStatus::StorageFailed) {
            return false;
        }
        m_stop.store(true);
        FinalizeLocked(false);
        return false;
    }

    void FinalizeLocked(bool sessionOk)
    {
        if (m_finalized) {
            return;
        }
        m_finalized = true;
        m_stop.store(true);
        if (m_ports) {
            m_ports->StopAndFlush();
            m_ports->LogFinalStatus();
        }
        m_livePose.SetRuntimeMode(RUNTIME_MODE_IDLE);
        std::cerr << "[session] calib exit ok="
                  << (sessionOk ? "true" : "false") << "\n";
    }

    UnifiedConfig m_cfg;
    std::atomic<bool> &m_stop;
    LivePoseState &m_livePose;
    mutable std::mutex m_mu;
    MainRuntimeAliases m_aliases{};
    std::unique_ptr<CalibSessionPortSet> m_ports;
    bool m_started{false};
    bool m_startFailed{false};
    bool m_finalized{false};
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

bool CalibRuntimeState::EnqueuePreview(const CalibStereoFrame &frame)
{
    return m_impl->EnqueuePreview(frame);
}

CalibImuSampleResult CalibRuntimeState::StepImuSample()
{
    return m_impl->StepImuSample();
}

bool CalibRuntimeState::Finalized() const
{
    return m_impl->Finalized();
}

void CalibRuntimeState::Finalize(bool sessionOk)
{
    m_impl->Finalize(sessionOk);
}

} // namespace smartdrone::core::application
