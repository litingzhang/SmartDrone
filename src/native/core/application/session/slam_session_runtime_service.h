#pragma once

#include <cstdint>
#include <memory>
#include <mutex>

#include "core/application/session/epg_messages.h"
#include "core/application/session/slam_frame_processor.h"
#include "core/application/session/slam_session_graph_service.h"
#include "core/application/session/slam_session_runtime.h"

namespace smartdrone::core::application {

struct SlamTaskStepResult {
    bool sessionAvailable{false};
    bool sessionOk{true};
    bool abortRequested{false};
};

struct SlamPrepareFrameResult : SlamTaskStepResult {
    std::shared_ptr<SlamPreparedFramePayload> frame;
};

struct SlamTrackFrameResult : SlamTaskStepResult {
    std::shared_ptr<SlamTrackedFramePayload> frame;
};

struct SlamPublishFrameResult : SlamTaskStepResult {
    std::shared_ptr<SlamPublishedFramePayload> frame;
};

class SlamSessionRuntimeService final {
  public:
    explicit SlamSessionRuntimeService(SlamSessionGraphRuntimeConfig config);
    ~SlamSessionRuntimeService();

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
        std::shared_ptr<SlamPreparedFramePayload> frame);
    SlamPublishFrameResult PostprocessTrackedFrame(
        std::uint64_t sessionId,
        std::shared_ptr<SlamTrackedFramePayload> frame);
    SlamTaskStepResult EmitPointCloud(
        std::uint64_t sessionId,
        SlamPublishedFramePayload &frame);
    SlamTaskStepResult EmitDfx(std::uint64_t sessionId,
                               SlamPublishedFramePayload &frame);
    SlamTaskStepResult EmitUdp(std::uint64_t sessionId,
                               SlamPublishedFramePayload &frame);
    SlamTaskStepResult EmitMavlink(std::uint64_t sessionId,
                                   SlamPublishedFramePayload &frame);
    SlamTaskStepResult EmitLivePose(std::uint64_t sessionId,
                                    SlamPublishedFramePayload &frame);

  private:
    std::shared_ptr<SlamSessionRuntime> Runtime() const;
    std::shared_ptr<SlamSessionRuntime> Runtime(std::uint64_t sessionId) const;
    SlamTaskStepResult MakeStepResult(const SlamSessionRuntime &runtime,
                                      SlamFrameProcessor::StepResult result)
        const;

    UnifiedConfig m_cfg;
    LiveRuntimeTuning &m_tuning;
    smartdrone::core::ports::ISlamSessionTelemetryPort &m_telemetry;
    smartdrone::core::ports::IPosePublisher &m_posePublisher;
    std::atomic<bool> &m_stop;
    LivePoseState &m_livePose;
    std::atomic<bool> &m_runningFlag;
    mutable std::mutex m_mu;
    mutable std::mutex m_processorMu;
    std::shared_ptr<SlamSessionRuntime> m_runtime;
    std::atomic<bool> m_stopped{true};
    std::uint64_t m_sessionId{0};
    bool m_started{false};
    bool m_startFailed{false};
};

} // namespace smartdrone::core::application
