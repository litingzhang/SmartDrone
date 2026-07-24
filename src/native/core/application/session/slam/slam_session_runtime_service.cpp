#include "core/application/session/slam/slam_session_runtime_service.h"

#include <atomic>
#include <memory>
#include <utility>

#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/application_runtime_factories.h"
#include "core/application/session/slam/slam_session_processing_port.h"
#include "core/application/session/slam/slam_session_runtime.h"
#include "core/application/state/live_pose_state.h"
#include "core/ports/pose_publisher.h"
#include "core/ports/slam_session_telemetry.h"

namespace SmartDrone::Core::Application {
namespace {

template <typename Flag>
class AtomicBoolResetGuard {
  public:
    explicit AtomicBoolResetGuard(Flag &flag)
        : m_flag(flag)
    {
    }

    ~AtomicBoolResetGuard()
    {
        m_flag.store(false, std::memory_order_release);
    }

  private:
    Flag &m_flag;
};

} // namespace

class SlamSessionRuntimeService::Impl final {
  public:
    explicit Impl(SlamSessionRuntimeServiceConfig config);
    ~Impl();

    bool EnsureStarted();
    bool SetCameraFrameReadyCallback(
        SmartDrone::Core::Ports::CameraFrameReadyCallback callback);
    bool StartFailed() const;
    bool Stopped() const;
    std::uint64_t SessionId() const;
    void Stop();
    bool ShutdownAndSaveTrajectoryEuRoC(const std::string &path);

    bool StepImuPoll();
    bool ImuReady() const;
    void RequestBackendStop();
    bool BackendStopped() const;
    SlamTaskStepResult StepBackend();
    SlamPrepareFrameResult AcquireAndPrepareFrame(std::uint64_t sessionId);
    SlamTrackFrameResult TrackPreparedFrame(
        std::uint64_t sessionId,
        std::shared_ptr<ISlamPreparedFramePayload> frame);
    SlamPublishFrameResult PostprocessTrackedFrame(
        std::uint64_t sessionId,
        std::shared_ptr<ISlamTrackedFramePayload> frame);
    SlamTaskStepResult EmitPointCloud(
        std::uint64_t sessionId,
        ISlamPublishedFramePayload &frame);
    SlamTaskStepResult EmitDfx(std::uint64_t sessionId,
                               ISlamPublishedFramePayload &frame);
    SlamTaskStepResult EmitUdp(std::uint64_t sessionId,
                               ISlamPublishedFramePayload &frame);
    SlamTaskStepResult FlushPreview(std::uint64_t sessionId,
                                    ISlamPublishedFramePayload &frame);
    SlamTaskStepResult EmitMavlink(std::uint64_t sessionId,
                                   ISlamPublishedFramePayload &frame);
    SlamTaskStepResult EmitLivePose(std::uint64_t sessionId,
                                    ISlamPublishedFramePayload &frame);

  private:
    struct RuntimeState {
        std::shared_ptr<SlamSessionRuntime> runtime;
        std::uint64_t sessionId{0};
        bool started{false};
        bool startFailed{false};
    };

    std::shared_ptr<const RuntimeState> LoadRuntimeState() const;
    void StoreRuntimeState(std::shared_ptr<const RuntimeState> state);
    bool TryAcquireStartSlot(std::shared_ptr<const RuntimeState> &state);
    std::shared_ptr<SlamSessionRuntime> CreateRuntime() const;
    bool StartRuntime(const std::shared_ptr<SlamSessionRuntime> &runtime,
                      std::uint64_t sessionId);
    std::shared_ptr<SlamSessionRuntime> Runtime() const;
    std::shared_ptr<SlamSessionRuntime> Runtime(std::uint64_t sessionId) const;
    static SlamTaskStepResult MissingRuntimeResult();

    UnifiedConfig m_cfg;
    LiveRuntimeTuning &m_tuning;
    SmartDrone::Core::Ports::ISlamSessionTelemetryPort &m_telemetry;
    SmartDrone::Core::Ports::IPosePublisher &m_posePublisher;
    std::atomic<bool> &m_stop;
    LivePoseState &m_livePose;
    std::atomic<bool> &m_runningFlag;
    const ApplicationRuntimeFactories &m_factories;
    std::string m_finalEurocTrajectory;
    std::unique_ptr<SlamSessionProcessingPort> m_processingPort;
    std::shared_ptr<SmartDrone::Core::Ports::CameraFrameReadyCallback>
        m_frameReadyCallback;
    std::shared_ptr<const RuntimeState> m_runtimeState;
    std::atomic<bool> m_starting{false};
    std::atomic<bool> m_stopped{true};
    std::atomic<bool> m_stopRequested{false};
};

SlamSessionRuntimeService::Impl::Impl(
    SlamSessionRuntimeServiceConfig config)
    : m_cfg(config.cfg),
      m_tuning(config.tuning),
      m_telemetry(config.telemetry),
      m_posePublisher(config.posePublisher),
      m_stop(config.stop),
      m_livePose(config.livePose),
      m_runningFlag(config.runningFlag),
      m_factories(config.factories),
      m_finalEurocTrajectory(std::move(config.finalEurocTrajectory)),
      m_processingPort(std::make_unique<SlamSessionProcessingPort>())
{
    std::atomic_store_explicit(&m_runtimeState,
                               std::make_shared<const RuntimeState>(),
                               std::memory_order_release);
}

SlamSessionRuntimeService::Impl::~Impl()
{
    Stop();
}

bool SlamSessionRuntimeService::Impl::EnsureStarted()
{
    if (m_stopRequested.load(std::memory_order_acquire)) {
        return false;
    }
    std::shared_ptr<const RuntimeState> current = LoadRuntimeState();
    if (current->started) {
        return true;
    }
    if (current->startFailed) {
        return false;
    }
    if (!TryAcquireStartSlot(current)) {
        return LoadRuntimeState()->started;
    }
    AtomicBoolResetGuard<std::atomic<bool>> startingGuard(m_starting);
    current = LoadRuntimeState();
    if (current->started) {
        return true;
    }
    if (current->startFailed) {
        return false;
    }

    const bool started = StartRuntime(CreateRuntime(), current->sessionId);
    m_starting.store(false, std::memory_order_release);
    if (m_stopRequested.load(std::memory_order_acquire)) {
        Stop();
        return false;
    }
    return started;
}

bool SlamSessionRuntimeService::Impl::TryAcquireStartSlot(
    std::shared_ptr<const RuntimeState> &state)
{
    bool expected = false;
    if (!m_starting.compare_exchange_strong(expected, true,
                                            std::memory_order_acq_rel)) {
        state = LoadRuntimeState();
        return false;
    }
    return true;
}

std::shared_ptr<SlamSessionRuntime>
SlamSessionRuntimeService::Impl::CreateRuntime() const
{
    const auto frameReadyCallback = std::atomic_load_explicit(
        &m_frameReadyCallback, std::memory_order_acquire);
    return std::make_shared<SlamSessionRuntime>(SlamSessionRuntimeConfig{
        m_cfg,
        m_tuning,
        m_telemetry,
        m_posePublisher,
        m_livePose,
        m_stop,
        m_runningFlag,
        m_factories,
        frameReadyCallback
            ? *frameReadyCallback
            : SmartDrone::Core::Ports::CameraFrameReadyCallback{},
    });
}

bool SlamSessionRuntimeService::Impl::SetCameraFrameReadyCallback(
    SmartDrone::Core::Ports::CameraFrameReadyCallback callback)
{
    if (m_starting.load(std::memory_order_acquire) ||
        LoadRuntimeState()->started) {
        return false;
    }
    std::atomic_store_explicit(
        &m_frameReadyCallback,
        std::make_shared<SmartDrone::Core::Ports::CameraFrameReadyCallback>(
            std::move(callback)),
        std::memory_order_release);
    return true;
}

bool SlamSessionRuntimeService::Impl::StartRuntime(
    const std::shared_ptr<SlamSessionRuntime> &runtime,
    std::uint64_t sessionId)
{
    m_stopped.store(false, std::memory_order_release);
    if (!runtime->Start()) {
        StoreRuntimeState(std::make_shared<const RuntimeState>(
            RuntimeState{nullptr, sessionId, false, true}));
        m_stopped.store(true, std::memory_order_release);
        return false;
    }
    runtime->PrepareFramePorts();

    StoreRuntimeState(std::make_shared<const RuntimeState>(
        RuntimeState{runtime, sessionId + 1, true, false}));
    return true;
}

bool SlamSessionRuntimeService::Impl::StartFailed() const
{
    return LoadRuntimeState()->startFailed;
}

bool SlamSessionRuntimeService::Impl::Stopped() const
{
    return !m_starting.load(std::memory_order_acquire) &&
           m_stopped.load(std::memory_order_acquire);
}

std::uint64_t SlamSessionRuntimeService::Impl::SessionId() const
{
    std::shared_ptr<const RuntimeState> state = LoadRuntimeState();
    return state->started ? state->sessionId : 0;
}

void SlamSessionRuntimeService::Impl::Stop()
{
    m_stopRequested.store(true, std::memory_order_release);
    if (m_starting.load(std::memory_order_acquire)) {
        return;
    }
    std::shared_ptr<const RuntimeState> state = LoadRuntimeState();
    StoreRuntimeState(std::make_shared<const RuntimeState>(
        RuntimeState{nullptr, state->sessionId, false, state->startFailed}));
    std::shared_ptr<SlamSessionRuntime> runtime = state->runtime;
    if (runtime) {
        if (!m_finalEurocTrajectory.empty()) {
            runtime->ShutdownAndSaveTrajectoryEuRoC(m_finalEurocTrajectory);
        }
        runtime->Stop();
    }
    m_stopped.store(true, std::memory_order_release);
}

bool SlamSessionRuntimeService::Impl::ShutdownAndSaveTrajectoryEuRoC(
    const std::string &path)
{
    auto runtime = Runtime();
    return runtime != nullptr &&
           runtime->ShutdownAndSaveTrajectoryEuRoC(path);
}

bool SlamSessionRuntimeService::Impl::StepImuPoll()
{
    auto runtime = Runtime();
    if (!runtime) {
        return true;
    }
    return runtime->StepImuPoll();
}

bool SlamSessionRuntimeService::Impl::ImuReady() const
{
    auto runtime = Runtime();
    return runtime ? runtime->ImuReady() : false;
}

void SlamSessionRuntimeService::Impl::RequestBackendStop()
{
    auto runtime = Runtime();
    if (runtime) {
        runtime->RequestBackendStop();
    }
}

bool SlamSessionRuntimeService::Impl::BackendStopped() const
{
    auto runtime = Runtime();
    return !runtime || runtime->BackendStopped();
}

SlamTaskStepResult SlamSessionRuntimeService::Impl::StepBackend()
{
    auto runtime = Runtime();
    if (!runtime) {
        return MissingRuntimeResult();
    }
    return m_processingPort->StepBackend(*runtime);
}

SlamPrepareFrameResult SlamSessionRuntimeService::Impl::AcquireAndPrepareFrame(
    std::uint64_t sessionId)
{
    SlamPrepareFrameResult output;
    auto runtime = Runtime(sessionId);
    if (!runtime) {
        return output;
    }
    return m_processingPort->AcquireAndPrepareFrame(*runtime);
}

SlamTrackFrameResult SlamSessionRuntimeService::Impl::TrackPreparedFrame(
    std::uint64_t sessionId,
    std::shared_ptr<ISlamPreparedFramePayload> frame)
{
    SlamTrackFrameResult result;
    auto runtime = Runtime(sessionId);
    if (runtime && frame) {
        result = m_processingPort->TrackPreparedFrame(*runtime,
                                                      std::move(frame));
    }
    return result;
}

SlamPublishFrameResult
SlamSessionRuntimeService::Impl::PostprocessTrackedFrame(
    std::uint64_t sessionId,
    std::shared_ptr<ISlamTrackedFramePayload> frame)
{
    SlamPublishFrameResult output;
    auto runtime = Runtime(sessionId);
    if (!runtime || !frame) {
        return output;
    }
    return m_processingPort->PostprocessTrackedFrame(*runtime,
                                                     std::move(frame));
}

SlamTaskStepResult SlamSessionRuntimeService::Impl::EmitPointCloud(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    auto runtime = Runtime(sessionId);
    if (!runtime) {
        return MissingRuntimeResult();
    }
    return m_processingPort->EmitPointCloud(*runtime, frame);
}

SlamTaskStepResult SlamSessionRuntimeService::Impl::EmitDfx(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    auto runtime = Runtime(sessionId);
    if (!runtime) {
        return MissingRuntimeResult();
    }
    return m_processingPort->EmitDfx(*runtime, frame);
}

SlamTaskStepResult SlamSessionRuntimeService::Impl::EmitUdp(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    auto runtime = Runtime(sessionId);
    if (!runtime) {
        return MissingRuntimeResult();
    }
    return m_processingPort->EmitUdp(*runtime, frame);
}

SlamTaskStepResult SlamSessionRuntimeService::Impl::FlushPreview(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    auto runtime = Runtime(sessionId);
    if (!runtime) {
        return MissingRuntimeResult();
    }
    return m_processingPort->FlushPreview(*runtime, frame);
}

SlamTaskStepResult SlamSessionRuntimeService::Impl::EmitMavlink(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    auto runtime = Runtime(sessionId);
    if (!runtime) {
        return MissingRuntimeResult();
    }
    return m_processingPort->EmitMavlink(*runtime, frame);
}

SlamTaskStepResult SlamSessionRuntimeService::Impl::EmitLivePose(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    auto runtime = Runtime(sessionId);
    if (!runtime) {
        return MissingRuntimeResult();
    }
    return m_processingPort->EmitLivePose(*runtime, frame);
}

std::shared_ptr<const SlamSessionRuntimeService::Impl::RuntimeState>
SlamSessionRuntimeService::Impl::LoadRuntimeState() const
{
    return std::atomic_load_explicit(&m_runtimeState,
                                     std::memory_order_acquire);
}

void SlamSessionRuntimeService::Impl::StoreRuntimeState(
    std::shared_ptr<const RuntimeState> state)
{
    std::atomic_store_explicit(&m_runtimeState, std::move(state),
                               std::memory_order_release);
}

std::shared_ptr<SlamSessionRuntime>
SlamSessionRuntimeService::Impl::Runtime() const
{
    return LoadRuntimeState()->runtime;
}

std::shared_ptr<SlamSessionRuntime> SlamSessionRuntimeService::Impl::Runtime(
    std::uint64_t sessionId) const
{
    std::shared_ptr<const RuntimeState> state = LoadRuntimeState();
    if (!state->started || sessionId == 0 || sessionId != state->sessionId) {
        return nullptr;
    }
    return state->runtime;
}

SlamTaskStepResult SlamSessionRuntimeService::Impl::MissingRuntimeResult()
{
    return {false, false, false};
}

SlamSessionRuntimeService::SlamSessionRuntimeService(
    SlamSessionRuntimeServiceConfig config)
    : m_impl(std::make_unique<Impl>(std::move(config)))
{
}

SlamSessionRuntimeService::~SlamSessionRuntimeService() = default;

bool SlamSessionRuntimeService::EnsureStarted()
{
    return m_impl->EnsureStarted();
}

bool SlamSessionRuntimeService::SetCameraFrameReadyCallback(
    SmartDrone::Core::Ports::CameraFrameReadyCallback callback)
{
    return m_impl->SetCameraFrameReadyCallback(std::move(callback));
}

bool SlamSessionRuntimeService::StartFailed() const
{
    return m_impl->StartFailed();
}

bool SlamSessionRuntimeService::Stopped() const
{
    return m_impl->Stopped();
}

std::uint64_t SlamSessionRuntimeService::SessionId() const
{
    return m_impl->SessionId();
}

void SlamSessionRuntimeService::Stop()
{
    m_impl->Stop();
}

bool SlamSessionRuntimeService::ShutdownAndSaveTrajectoryEuRoC(
    const std::string &path)
{
    return m_impl->ShutdownAndSaveTrajectoryEuRoC(path);
}

bool SlamSessionRuntimeService::StepImuPoll()
{
    return m_impl->StepImuPoll();
}

bool SlamSessionRuntimeService::ImuReady() const
{
    return m_impl->ImuReady();
}

void SlamSessionRuntimeService::RequestBackendStop()
{
    m_impl->RequestBackendStop();
}

bool SlamSessionRuntimeService::BackendStopped() const
{
    return m_impl->BackendStopped();
}

SlamTaskStepResult SlamSessionRuntimeService::StepBackend()
{
    return m_impl->StepBackend();
}

SlamPrepareFrameResult SlamSessionRuntimeService::AcquireAndPrepareFrame(
    std::uint64_t sessionId)
{
    return m_impl->AcquireAndPrepareFrame(sessionId);
}

SlamTrackFrameResult SlamSessionRuntimeService::TrackPreparedFrame(
    std::uint64_t sessionId,
    std::shared_ptr<ISlamPreparedFramePayload> frame)
{
    return m_impl->TrackPreparedFrame(sessionId, std::move(frame));
}

SlamPublishFrameResult SlamSessionRuntimeService::PostprocessTrackedFrame(
    std::uint64_t sessionId,
    std::shared_ptr<ISlamTrackedFramePayload> frame)
{
    return m_impl->PostprocessTrackedFrame(sessionId, std::move(frame));
}

SlamTaskStepResult SlamSessionRuntimeService::EmitPointCloud(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    return m_impl->EmitPointCloud(sessionId, frame);
}

SlamTaskStepResult SlamSessionRuntimeService::EmitDfx(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    return m_impl->EmitDfx(sessionId, frame);
}

SlamTaskStepResult SlamSessionRuntimeService::EmitUdp(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    return m_impl->EmitUdp(sessionId, frame);
}

SlamTaskStepResult SlamSessionRuntimeService::FlushPreview(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    return m_impl->FlushPreview(sessionId, frame);
}

SlamTaskStepResult SlamSessionRuntimeService::EmitMavlink(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    return m_impl->EmitMavlink(sessionId, frame);
}

SlamTaskStepResult SlamSessionRuntimeService::EmitLivePose(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    return m_impl->EmitLivePose(sessionId, frame);
}

} // namespace SmartDrone::Core::Application
