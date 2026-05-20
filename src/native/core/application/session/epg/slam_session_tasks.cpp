#include "core/application/session/epg/slam_session_tasks.h"

#include <array>
#include <cstdint>
#include <utility>

#include "core/application/session/epg/messages/slam_epg_messages.h"
#include "core/application/session/slam/slam_session_runtime_service.h"
#include "core/application/session/epg/slam_session_task_utils.h"

namespace smartdrone::core::application {
namespace {

constexpr epg::PortId STATUS_OUTPUT_PORT = 1;
constexpr epg::PortId POSE_POSTPROCESS_STATUS_OUTPUT_PORT = 4;
constexpr epg::PortId DFX_STATUS_OUTPUT_PORT = 0;
constexpr std::array<epg::PortId, 4> PUBLISHED_FRAME_FAN_OUT_PORTS{
    0, 1, 2, 3};
constexpr std::array<epg::PortId, 9> SLAM_MONITOR_STATUS_INPUT_PORTS{
    0, 1, 2, 3, 4, 5, 6, 7, 8,
};

bool ShouldRunTask(const std::atomic<bool> &runningFlag,
                   const std::atomic<bool> &stop)
{
    return runningFlag.load() && !stop.load();
}

using PublishedOutputFn = SlamTaskStepResult (
    SlamSessionRuntimeService::*)(std::uint64_t,
                                  ISlamPublishedFramePayload &);

void PushSlamStatus(epg::TaskContext &context, bool sessionOk,
                    bool abortRequested,
                    epg::PortId port)
{
    auto status = context.Make<SlamStatus>();
    status->sessionOk = sessionOk;
    status->abortRequested = abortRequested;
    context.Push(port, std::move(status));
}

void PushSlamPublishedFrame(
    epg::TaskContext &context,
    epg::PortId port,
    std::uint64_t sessionId,
    const std::shared_ptr<ISlamPublishedFramePayload> &frame)
{
    auto published = context.Make<SlamPublishedFrame>();
    published->sessionId = sessionId;
    published->frame = frame;
    context.Push(port, std::move(published));
}

bool StepResultAllowsContinue(epg::TaskContext &context,
                              const SlamTaskStepResult &result,
                              epg::PortId statusPort)
{
    if (!result.sessionAvailable) {
        return false;
    }
    if (result.abortRequested) {
        PushSlamStatus(context, result.sessionOk, true, statusPort);
        return false;
    }
    return true;
}

template <typename MessageType, typename FramePayload>
void PushSlamStageFrame(epg::TaskContext &context, std::uint64_t sessionId,
                        FramePayload frame)
{
    auto message = context.Make<MessageType>();
    message->sessionId = sessionId;
    message->frame = std::move(frame);
    context.Push(0, std::move(message));
}

bool ValidPublishedFrame(const std::shared_ptr<SlamPublishedFrame> &frame)
{
    return frame && frame->sessionId != 0 && frame->frame;
}

bool SamePublishedFrame(const std::shared_ptr<SlamPublishedFrame> &lhs,
                        const std::shared_ptr<SlamPublishedFrame> &rhs)
{
    return ValidPublishedFrame(lhs) && ValidPublishedFrame(rhs) &&
           lhs->sessionId == rhs->sessionId && lhs->frame == rhs->frame;
}

bool AllPendingFramesReady(
    const std::array<std::shared_ptr<SlamPublishedFrame>, 4> &frames)
{
    return SamePublishedFrame(frames[0], frames[1]) &&
           SamePublishedFrame(frames[0], frames[2]) &&
           SamePublishedFrame(frames[0], frames[3]);
}

void ResetPendingFrames(
    std::array<std::shared_ptr<SlamPublishedFrame>, 4> &frames)
{
    for (auto &frame : frames) {
        frame.reset();
    }
}

void RunPublishedOutputTask(epg::TaskContext &context,
                            SlamSessionRuntimeService &service,
                            PublishedOutputFn outputFn)
{
    const auto published = context.TryPopLatest<SlamPublishedFrame>(0);
    if (!published || published->sessionId == 0 || !published->frame) {
        return;
    }

    const SlamTaskStepResult result =
        (service.*outputFn)(published->sessionId, *published->frame);
    if (!StepResultAllowsContinue(context, result, STATUS_OUTPUT_PORT)) {
        return;
    }

    context.Push(0, published);
    PushSlamStatus(context, result.sessionOk, false, STATUS_OUTPUT_PORT);
}

} // namespace

SlamResourceTask::SlamResourceTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)), m_stop(stop), m_runningFlag(runningFlag)
{
}

void SlamResourceTask::OnTick(epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        m_service->Stop();
        return;
    }
    if (m_readyEmitted) {
        return;
    }
    if (!m_service->EnsureStarted()) {
        m_stop.store(true);
        return;
    }
    auto ready = context.Make<SlamResourceReady>();
    ready->ready = true;
    if (context.Push(0, std::move(ready))) {
        m_readyEmitted = true;
    }
}

SlamClockTask::SlamClockTask(std::atomic<bool> &stop,
                             std::atomic<bool> &runningFlag)
    : m_stop(stop), m_runningFlag(runningFlag)
{
}

void SlamClockTask::OnTick(epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    auto tick = context.Make<SlamTick>();
    tick->sequence = ++m_sequence;
    context.Push(0, std::move(tick));
}

SlamImuPollTask::SlamImuPollTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)), m_stop(stop), m_runningFlag(runningFlag)
{
}

void SlamImuPollTask::OnTick(epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    if (!m_service->StepImuPoll()) {
        m_stop.store(true);
        return;
    }
    if (!m_service->ImuReady() || context.OutputSize(0) > 0) {
        return;
    }

    auto ready = context.Make<SlamImuReady>();
    ready->ready = true;
    context.Push(0, std::move(ready));
}

SlamBackendTickTask::SlamBackendTickTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)), m_stop(stop), m_runningFlag(runningFlag)
{
}

void SlamBackendTickTask::OnTick(epg::TaskContext &context)
{
    (void)context;
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    const SlamTaskStepResult result = m_service->StepBackend();
    if (result.abortRequested) {
        m_stop.store(true);
    }
}

SlamImuGateTask::SlamImuGateTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag,
    LiveRuntimeTuning &tuning,
    int cameraFps)
    : m_service(std::move(service)),
      m_stop(stop),
      m_runningFlag(runningFlag),
      m_tuning(tuning),
      m_cameraFps(cameraFps)
{
}

void SlamImuGateTask::OnTick(epg::TaskContext &context)
{
    if (auto ready = context.TryPopLatest<SlamImuReady>(0)) {
        m_imuReady = ready->ready;
    }
    const auto tick = context.TryPopLatest<SlamTick>(1);
    if (m_service->StartFailed()) {
        PushSlamStatus(context, false, true, STATUS_OUTPUT_PORT);
        return;
    }
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    if (!m_imuReady || !tick || context.OutputSize(0) > 0) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto minInterval =
        SlamInputInterval(m_tuning.slamInputFps.load(std::memory_order_relaxed),
                          m_cameraFps);
    if (m_lastFrameReadyTime.time_since_epoch().count() != 0 &&
        now - m_lastFrameReadyTime < minInterval) {
        return;
    }

    const std::uint64_t sessionId = m_service->SessionId();
    if (sessionId == 0) {
        return;
    }
    auto frameReady = context.Make<SlamFrameReady>();
    frameReady->sessionId = sessionId;
    if (context.Push(0, std::move(frameReady))) {
        m_lastFrameReadyTime = now;
    }
}

SlamAcquireTask::SlamAcquireTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)),
      m_stop(stop),
      m_runningFlag(runningFlag)
{
}

void SlamAcquireTask::OnTick(epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    const auto frameReady = context.TryPopLatest<SlamFrameReady>(0);
    if (!frameReady || frameReady->sessionId == 0 ||
        context.OutputSize(0) > 0) {
        return;
    }

    SlamPrepareFrameResult result =
        m_service->AcquireAndPrepareFrame(frameReady->sessionId);
    if (!StepResultAllowsContinue(context, result, STATUS_OUTPUT_PORT)) {
        return;
    }
    if (!result.frame) {
        return;
    }

    PushSlamStageFrame<SlamPreparedFrame>(context, frameReady->sessionId,
                                          std::move(result.frame));
}

SlamTrackingTask::SlamTrackingTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)),
      m_stop(stop),
      m_runningFlag(runningFlag)
{
}

void SlamTrackingTask::OnTick(epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    const auto prepared = context.TryPopLatest<SlamPreparedFrame>(0);
    if (!prepared || prepared->sessionId == 0 || !prepared->frame) {
        return;
    }

    SlamTrackFrameResult result =
        m_service->TrackPreparedFrame(prepared->sessionId,
                                      std::move(prepared->frame));
    if (!StepResultAllowsContinue(context, result, STATUS_OUTPUT_PORT)) {
        return;
    }
    if (!result.frame) {
        return;
    }

    PushSlamStageFrame<SlamTrackedFrame>(context, prepared->sessionId,
                                         std::move(result.frame));
}

SlamPosePostprocessTask::SlamPosePostprocessTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)),
      m_stop(stop),
      m_runningFlag(runningFlag)
{
}

void SlamPosePostprocessTask::OnTick(epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    const auto tracked = context.TryPopLatest<SlamTrackedFrame>(0);
    if (!tracked || tracked->sessionId == 0 || !tracked->frame) {
        return;
    }

    SlamPublishFrameResult result =
        m_service->PostprocessTrackedFrame(tracked->sessionId,
                                           std::move(tracked->frame));
    if (!StepResultAllowsContinue(context, result,
                                  POSE_POSTPROCESS_STATUS_OUTPUT_PORT)) {
        return;
    }
    if (!result.frame) {
        return;
    }

    for (const epg::PortId port : PUBLISHED_FRAME_FAN_OUT_PORTS) {
        PushSlamPublishedFrame(context, port, tracked->sessionId,
                               result.frame);
    }
}

SlamPointCloudTask::SlamPointCloudTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)),
      m_stop(stop),
      m_runningFlag(runningFlag)
{
}

void SlamPointCloudTask::OnTick(epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    RunPublishedOutputTask(context, *m_service,
                           &SlamSessionRuntimeService::EmitPointCloud);
}

SlamDfxTask::SlamDfxTask(std::shared_ptr<SlamSessionRuntimeService> service,
                         std::atomic<bool> &stop,
                         std::atomic<bool> &runningFlag)
    : m_service(std::move(service)),
      m_stop(stop),
      m_runningFlag(runningFlag)
{
}

void SlamDfxTask::OnTick(epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    for (epg::PortId port = 0; port < m_pendingFrames.size(); ++port) {
        if (auto frame = context.TryPopLatest<SlamPublishedFrame>(port)) {
            m_pendingFrames[port] = std::move(frame);
        }
    }
    if (!AllPendingFramesReady(m_pendingFrames)) {
        return;
    }
    const auto published = m_pendingFrames[0];
    ResetPendingFrames(m_pendingFrames);

    SlamTaskStepResult result =
        m_service->EmitDfx(published->sessionId, *published->frame);
    if (!StepResultAllowsContinue(context, result, DFX_STATUS_OUTPUT_PORT)) {
        return;
    }
    PushSlamStatus(context, result.sessionOk, false, DFX_STATUS_OUTPUT_PORT);
}

SlamUdpTask::SlamUdpTask(std::shared_ptr<SlamSessionRuntimeService> service,
                         std::atomic<bool> &stop,
                         std::atomic<bool> &runningFlag)
    : m_service(std::move(service)), m_stop(stop), m_runningFlag(runningFlag)
{
}

void SlamUdpTask::OnTick(epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    RunPublishedOutputTask(context, *m_service,
                           &SlamSessionRuntimeService::EmitUdp);
}

SlamMavlinkTask::SlamMavlinkTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)), m_stop(stop), m_runningFlag(runningFlag)
{
}

void SlamMavlinkTask::OnTick(epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    RunPublishedOutputTask(context, *m_service,
                           &SlamSessionRuntimeService::EmitMavlink);
}

SlamLivePoseTask::SlamLivePoseTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)), m_stop(stop), m_runningFlag(runningFlag)
{
}

void SlamLivePoseTask::OnTick(epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    RunPublishedOutputTask(context, *m_service,
                           &SlamSessionRuntimeService::EmitLivePose);
}

SlamMonitorTask::SlamMonitorTask(std::atomic<bool> &stop,
                                 std::atomic<bool> &sessionOk)
    : m_stop(stop), m_sessionOk(sessionOk)
{
}

void SlamMonitorTask::OnTick(epg::TaskContext &context)
{
    for (const epg::PortId port : SLAM_MONITOR_STATUS_INPUT_PORTS) {
        while (auto status = context.TryPop<SlamStatus>(port)) {
            m_sessionOk.store(status->sessionOk, std::memory_order_relaxed);
            if (status->abortRequested) {
                m_stop.store(true);
            }
        }
    }
}

EPG_REGISTER_TASK_TYPE(SlamResourceTask, "SlamResourceTask")
EPG_REGISTER_TASK_TYPE(SlamClockTask, "SlamClockTask")
EPG_REGISTER_TASK_TYPE(SlamImuPollTask, "SlamImuPollTask")
EPG_REGISTER_TASK_TYPE(SlamBackendTickTask, "SlamBackendTickTask")
EPG_REGISTER_TASK_TYPE(SlamImuGateTask, "SlamImuGateTask")
EPG_REGISTER_TASK_TYPE(SlamAcquireTask, "SlamAcquireTask")
EPG_REGISTER_TASK_TYPE(SlamTrackingTask, "SlamTrackingTask")
EPG_REGISTER_TASK_TYPE(SlamPosePostprocessTask, "SlamPosePostprocessTask")
EPG_REGISTER_TASK_TYPE(SlamPointCloudTask, "SlamPointCloudTask")
EPG_REGISTER_TASK_TYPE(SlamDfxTask, "SlamDfxTask")
EPG_REGISTER_TASK_TYPE(SlamUdpTask, "SlamUdpTask")
EPG_REGISTER_TASK_TYPE(SlamMavlinkTask, "SlamMavlinkTask")
EPG_REGISTER_TASK_TYPE(SlamLivePoseTask, "SlamLivePoseTask")
EPG_REGISTER_TASK_TYPE(SlamMonitorTask, "SlamMonitorTask")

} // namespace smartdrone::core::application
