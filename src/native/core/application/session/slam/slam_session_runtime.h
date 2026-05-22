#pragma once

#include <atomic>
#include <memory>

#include "core/application/session/slam/slam_frame_step_result.h"

namespace SmartDrone::Core::Ports {

class IPosePublisher;
class ISlamSessionTelemetryPort;

} // namespace SmartDrone::Core::Ports

namespace SmartDrone::Core::Application {

struct ApplicationRuntimeFactories;
struct LivePoseState;
struct LiveRuntimeTuning;
struct SlamPreparedFrameData;
struct SlamPublishedFrameData;
struct SlamTrackedFrameData;
struct UnifiedConfig;

struct SlamSessionRuntimeConfig {
    const UnifiedConfig &cfg;
    LiveRuntimeTuning &tuning;
    SmartDrone::Core::Ports::ISlamSessionTelemetryPort &telemetry;
    SmartDrone::Core::Ports::IPosePublisher &posePublisher;
    LivePoseState &livePose;
    std::atomic<bool> &stop;
    std::atomic<bool> &runningFlag;
    const ApplicationRuntimeFactories &factories;
};

class SlamSessionRuntime {
  public:
    explicit SlamSessionRuntime(SlamSessionRuntimeConfig config);
    ~SlamSessionRuntime();

    bool Start();
    void PrepareFramePorts();
    void Stop();
    bool StepImuPoll();
    bool ImuReady() const;
    SlamFrameStageResult StepBackend();
    SlamFrameStageResult AcquireAndPrepareFrame(SlamPreparedFrameData &frame);
    SlamFrameStageResult TrackPreparedFrame(
        std::shared_ptr<SlamPreparedFrameData> frame,
        SlamTrackedFrameData &tracked);
    SlamFrameStageResult PostprocessTrackedFrame(
        std::shared_ptr<SlamTrackedFrameData> tracked,
        SlamPublishedFrameData &published);
    SlamFrameStageResult EmitPointCloud(SlamPublishedFrameData &published);
    SlamFrameStageResult EmitDfx(SlamPublishedFrameData &published);
    SlamFrameStageResult EmitUdp(SlamPublishedFrameData &published);
    SlamFrameStageResult EmitMavlink(SlamPublishedFrameData &published);
    SlamFrameStageResult EmitLivePose(SlamPublishedFrameData &published);

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Core::Application
