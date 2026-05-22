#include "core/application/session/slam/slam_session_runtime_service.h"

#include <atomic>
#include <chrono>
#include <mutex>
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

template <typename Result, typename Operation>
Result RunLockedRuntimeOperation(
    const std::shared_ptr<SlamSessionRuntime> &runtime,
    std::mutex &mutex,
    Operation operation)
{
    Result output;
    if (!runtime) {
        return output;
    }

    const auto waitBegin = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex);
    const auto waitEnd = std::chrono::steady_clock::now();
    output = operation(*runtime);
    output.resourceWaitUs = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            waitEnd - waitBegin)
            .count());
    return output;
}

template <typename Result, typename Payload, typename Operation>
Result RunLockedPayloadOperation(
    const std::shared_ptr<SlamSessionRuntime> &runtime,
    std::shared_ptr<Payload> payload,
    std::mutex &mutex,
    Operation operation)
{
    Result output;
    if (!runtime || !payload) {
        return output;
    }

    const auto waitBegin = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex);
    const auto waitEnd = std::chrono::steady_clock::now();
    output = operation(*runtime, std::move(payload));
    output.resourceWaitUs = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            waitEnd - waitBegin)
            .count());
    return output;
}

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
    SlamTaskStepResult EmitMavlink(std::uint64_t sessionId,
                                   ISlamPublishedFramePayload &frame);
    SlamTaskStepResult EmitLivePose(std::uint64_t sessionId,
                                    ISlamPublishedFramePayload &frame);

  private:
    std::shared_ptr<SlamSessionRuntime> Runtime() const;
    std::shared_ptr<SlamSessionRuntime> Runtime(std::uint64_t sessionId) const;

    UnifiedConfig m_cfg;
    LiveRuntimeTuning &m_tuning;
    SmartDrone::Core::Ports::ISlamSessionTelemetryPort &m_telemetry;
    SmartDrone::Core::Ports::IPosePublisher &m_posePublisher;
    std::atomic<bool> &m_stop;
    LivePoseState &m_livePose;
    std::atomic<bool> &m_runningFlag;
    const ApplicationRuntimeFactories &m_factories;
    mutable std::mutex m_mu;
    mutable std::mutex m_inputStageMu;
    mutable std::mutex m_trackingStageMu;
    mutable std::mutex m_posePostprocessStageMu;
    mutable std::mutex m_pointCloudOutputMu;
    mutable std::mutex m_dfxOutputMu;
    mutable std::mutex m_livePoseOutputMu;
    mutable std::mutex m_mavlinkOutputMu;
    mutable std::mutex m_udpOutputMu;
    std::unique_ptr<SlamSessionProcessingPort> m_processingPort;
    std::shared_ptr<SlamSessionRuntime> m_runtime;
    std::atomic<bool> m_stopped{true};
    std::uint64_t m_sessionId{0};
    bool m_started{false};
    bool m_startFailed{false};
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
}

SlamSessionRuntimeService::Impl::~Impl()
{
    Stop();
}

bool SlamSessionRuntimeService::Impl::EnsureStarted()
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (m_started) {
        return true;
    }
    if (m_startFailed) {
        return false;
    }

    m_runtime = std::make_shared<SlamSessionRuntime>(SlamSessionRuntimeConfig{
        m_cfg,
        m_tuning,
        m_telemetry,
        m_posePublisher,
        m_livePose,
        m_stop,
        m_runningFlag,
        m_factories,
    });
    m_stopped.store(false, std::memory_order_release);
    if (!m_runtime->Start()) {
        m_runtime.reset();
        m_startFailed = true;
        m_stopped.store(true, std::memory_order_release);
        return false;
    }
    m_runtime->PrepareFramePorts();

    m_started = true;
    ++m_sessionId;
    return true;
}

bool SlamSessionRuntimeService::Impl::StartFailed() const
{
    std::lock_guard<std::mutex> lock(m_mu);
    return m_startFailed;
}

bool SlamSessionRuntimeService::Impl::Stopped() const
{
    return m_stopped.load(std::memory_order_acquire);
}

std::uint64_t SlamSessionRuntimeService::Impl::SessionId() const
{
    std::lock_guard<std::mutex> lock(m_mu);
    return m_started ? m_sessionId : 0;
}

void SlamSessionRuntimeService::Impl::Stop()
{
    std::shared_ptr<SlamSessionRuntime> runtime;
    {
        std::lock_guard<std::mutex> lock(m_mu);
        runtime = std::move(m_runtime);
        m_started = false;
    }
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
    return RunLockedRuntimeOperation<SlamTaskStepResult>(
        Runtime(), m_trackingStageMu,
        [this](SlamSessionRuntime &runtime) {
            return m_processingPort->StepBackend(runtime);
        });
}

SlamPrepareFrameResult SlamSessionRuntimeService::Impl::AcquireAndPrepareFrame(
    std::uint64_t sessionId)
{
    return RunLockedRuntimeOperation<SlamPrepareFrameResult>(
        Runtime(sessionId), m_inputStageMu,
        [this](SlamSessionRuntime &runtime) {
            return m_processingPort->AcquireAndPrepareFrame(runtime);
        });
}

SlamTrackFrameResult SlamSessionRuntimeService::Impl::TrackPreparedFrame(
    std::uint64_t sessionId,
    std::shared_ptr<ISlamPreparedFramePayload> frame)
{
    return RunLockedPayloadOperation<SlamTrackFrameResult>(
        Runtime(sessionId), std::move(frame), m_trackingStageMu,
        [this](SlamSessionRuntime &runtime,
               std::shared_ptr<ISlamPreparedFramePayload> payload) {
            return m_processingPort->TrackPreparedFrame(runtime,
                                                        std::move(payload));
        });
}

SlamPublishFrameResult
SlamSessionRuntimeService::Impl::PostprocessTrackedFrame(
    std::uint64_t sessionId,
    std::shared_ptr<ISlamTrackedFramePayload> frame)
{
    return RunLockedPayloadOperation<SlamPublishFrameResult>(
        Runtime(sessionId), std::move(frame), m_posePostprocessStageMu,
        [this](SlamSessionRuntime &runtime,
               std::shared_ptr<ISlamTrackedFramePayload> payload) {
            return m_processingPort->PostprocessTrackedFrame(
                runtime, std::move(payload));
        });
}

SlamTaskStepResult SlamSessionRuntimeService::Impl::EmitPointCloud(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    return RunLockedRuntimeOperation<SlamTaskStepResult>(
        Runtime(sessionId), m_pointCloudOutputMu,
        [this, &frame](SlamSessionRuntime &runtime) {
            return m_processingPort->EmitPointCloud(runtime, frame);
        });
}

SlamTaskStepResult SlamSessionRuntimeService::Impl::EmitDfx(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    return RunLockedRuntimeOperation<SlamTaskStepResult>(
        Runtime(sessionId), m_dfxOutputMu,
        [this, &frame](SlamSessionRuntime &runtime) {
            return m_processingPort->EmitDfx(runtime, frame);
        });
}

SlamTaskStepResult SlamSessionRuntimeService::Impl::EmitUdp(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    return RunLockedRuntimeOperation<SlamTaskStepResult>(
        Runtime(sessionId), m_udpOutputMu,
        [this, &frame](SlamSessionRuntime &runtime) {
            return m_processingPort->EmitUdp(runtime, frame);
        });
}

SlamTaskStepResult SlamSessionRuntimeService::Impl::EmitMavlink(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    return RunLockedRuntimeOperation<SlamTaskStepResult>(
        Runtime(sessionId), m_mavlinkOutputMu,
        [this, &frame](SlamSessionRuntime &runtime) {
            return m_processingPort->EmitMavlink(runtime, frame);
        });
}

SlamTaskStepResult SlamSessionRuntimeService::Impl::EmitLivePose(
    std::uint64_t sessionId,
    ISlamPublishedFramePayload &frame)
{
    return RunLockedRuntimeOperation<SlamTaskStepResult>(
        Runtime(sessionId), m_livePoseOutputMu,
        [this, &frame](SlamSessionRuntime &runtime) {
            return m_processingPort->EmitLivePose(runtime, frame);
        });
}

std::shared_ptr<SlamSessionRuntime>
SlamSessionRuntimeService::Impl::Runtime() const
{
    std::lock_guard<std::mutex> lock(m_mu);
    return m_runtime;
}

std::shared_ptr<SlamSessionRuntime> SlamSessionRuntimeService::Impl::Runtime(
    std::uint64_t sessionId) const
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (!m_started || sessionId == 0 || sessionId != m_sessionId) {
        return nullptr;
    }
    return m_runtime;
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
