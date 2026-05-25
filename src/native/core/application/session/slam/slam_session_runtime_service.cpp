#include "core/application/session/slam/slam_session_runtime_service.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <utility>

#include "common/environment.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/application_runtime_factories.h"
#include "core/application/session/slam/slam_session_processing_port.h"
#include "core/application/session/slam/slam_session_runtime.h"
#include "core/application/state/live_pose_state.h"
#include "core/ports/pose_publisher.h"
#include "core/ports/slam_session_telemetry.h"

namespace SmartDrone::Core::Application {
namespace {

std::uint64_t EpgBackendTickMinIntervalMs()
{
    return static_cast<std::uint64_t>(SmartDrone::Common::EnvIntValueClamped(
        "SMART_DRONE_EPG_BACKEND_TICK_MIN_INTERVAL_MS", 50, 0, 1000));
}

std::uint64_t EpgBackendTickTrackingCooldownMs()
{
    return static_cast<std::uint64_t>(SmartDrone::Common::EnvIntValueClamped(
        "SMART_DRONE_EPG_BACKEND_TICK_TRACKING_COOLDOWN_MS", 50, 0, 1000));
}

std::uint64_t SteadyNowMs()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

bool WithinCooldown(std::uint64_t nowMs,
                    std::uint64_t lastStepMs,
                    std::uint64_t minIntervalMs)
{
    return minIntervalMs > 0 && lastStepMs > 0 &&
           nowMs - lastStepMs < minIntervalMs;
}

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
    bool StartFailed() const;
    bool Stopped() const;
    std::uint64_t SessionId() const;
    void Stop();

    bool StepImuPoll();
    bool ImuReady() const;
    SlamTaskStepResult StepBackend();
    SlamTaskStepResult StepBackendIfIdle();
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
    std::unique_ptr<SlamSessionProcessingPort> m_processingPort;
    std::shared_ptr<const RuntimeState> m_runtimeState;
    std::atomic<bool> m_starting{false};
    std::atomic<bool> m_stopped{true};
    std::atomic<bool> m_backendBusy{false};
    std::atomic<std::uint64_t> m_lastBackendStepMs{0};
    std::atomic<std::uint64_t> m_lastTrackingStepMs{0};
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

    return StartRuntime(CreateRuntime(), current->sessionId);
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
    return std::make_shared<SlamSessionRuntime>(SlamSessionRuntimeConfig{
        m_cfg,
        m_tuning,
        m_telemetry,
        m_posePublisher,
        m_livePose,
        m_stop,
        m_runningFlag,
        m_factories,
    });
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
    return m_stopped.load(std::memory_order_acquire);
}

std::uint64_t SlamSessionRuntimeService::Impl::SessionId() const
{
    std::shared_ptr<const RuntimeState> state = LoadRuntimeState();
    return state->started ? state->sessionId : 0;
}

void SlamSessionRuntimeService::Impl::Stop()
{
    std::shared_ptr<const RuntimeState> state = LoadRuntimeState();
    StoreRuntimeState(std::make_shared<const RuntimeState>(
        RuntimeState{nullptr, state->sessionId, false, state->startFailed}));
    std::shared_ptr<SlamSessionRuntime> runtime = state->runtime;
    if (runtime) {
        runtime->Stop();
    }
    m_stopped.store(true, std::memory_order_release);
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

SlamTaskStepResult SlamSessionRuntimeService::Impl::StepBackend()
{
    auto runtime = Runtime();
    if (!runtime) {
        return MissingRuntimeResult();
    }
    return m_processingPort->StepBackend(*runtime);
}

SlamTaskStepResult SlamSessionRuntimeService::Impl::StepBackendIfIdle()
{
    auto runtime = Runtime();
    if (!runtime) {
        return {};
    }
    const std::uint64_t nowMs = SteadyNowMs();
    const std::uint64_t minIntervalMs = EpgBackendTickMinIntervalMs();
    const std::uint64_t trackingCooldownMs =
        EpgBackendTickTrackingCooldownMs();
    const std::uint64_t lastBackendStepMs =
        m_lastBackendStepMs.load(std::memory_order_relaxed);
    const std::uint64_t lastTrackingStepMs =
        m_lastTrackingStepMs.load(std::memory_order_relaxed);
    if (WithinCooldown(nowMs, lastBackendStepMs, minIntervalMs) ||
        WithinCooldown(nowMs, lastTrackingStepMs, trackingCooldownMs)) {
        return {true, true, false};
    }

    bool expected = false;
    if (!m_backendBusy.compare_exchange_strong(expected, true,
                                               std::memory_order_acq_rel)) {
        return {true, true, false};
    }
    AtomicBoolResetGuard<std::atomic<bool>> busyGuard(m_backendBusy);
    SlamTaskStepResult result = m_processingPort->StepBackend(*runtime);
    m_lastBackendStepMs.store(SteadyNowMs(), std::memory_order_relaxed);
    return result;
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
    if (result.sessionAvailable) {
        m_lastTrackingStepMs.store(SteadyNowMs(), std::memory_order_relaxed);
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

bool SlamSessionRuntimeService::StepImuPoll()
{
    return m_impl->StepImuPoll();
}

bool SlamSessionRuntimeService::ImuReady() const
{
    return m_impl->ImuReady();
}

SlamTaskStepResult SlamSessionRuntimeService::StepBackend()
{
    return m_impl->StepBackend();
}

SlamTaskStepResult SlamSessionRuntimeService::StepBackendIfIdle()
{
    return m_impl->StepBackendIfIdle();
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
