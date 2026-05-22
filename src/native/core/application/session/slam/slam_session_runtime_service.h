#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include "core/application/session/epg/slam_session_task_results.h"

namespace SmartDrone::Core::Ports {
class IPosePublisher;
class ISlamSessionTelemetryPort;
} // namespace SmartDrone::Core::Ports

namespace SmartDrone::Core::Application {

struct ApplicationRuntimeFactories;
struct LivePoseState;
struct LiveRuntimeTuning;
struct UnifiedConfig;

struct SlamSessionRuntimeServiceConfig {
    const UnifiedConfig &cfg;
    LiveRuntimeTuning &tuning;
    SmartDrone::Core::Ports::ISlamSessionTelemetryPort &telemetry;
    SmartDrone::Core::Ports::IPosePublisher &posePublisher;
    std::atomic<bool> &stop;
    LivePoseState &livePose;
    std::atomic<bool> &runningFlag;
    const ApplicationRuntimeFactories &factories;
};

class SlamSessionRuntimeService final {
  public:
    explicit SlamSessionRuntimeService(SlamSessionRuntimeServiceConfig config);
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
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Core::Application
