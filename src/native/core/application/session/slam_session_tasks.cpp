#include "core/application/session/slam_session_tasks.h"

#include <utility>

#include "core/application/session/epg_messages.h"
#include "core/application/session/slam_session_task_utils.h"

namespace smartdrone::core::application {
namespace {

void PushSlamStatus(epg::TaskContext &context, bool sessionOk,
                    bool abortRequested)
{
    auto status = context.Make<SlamStatus>();
    status->sessionOk = sessionOk;
    status->abortRequested = abortRequested;
    context.Push(1, std::move(status));
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
    if (!m_runningFlag.load() || m_stop.load()) {
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
    if (!m_runningFlag.load() || m_stop.load()) {
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
    if (!m_runningFlag.load() || m_stop.load()) {
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
    if (!m_runningFlag.load() || m_stop.load()) {
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
        PushStatus(context, false, true);
        return;
    }
    if (!m_runningFlag.load() || m_stop.load()) {
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

void SlamImuGateTask::PushStatus(epg::TaskContext &context, bool sessionOk,
                                 bool abortRequested)
{
    PushSlamStatus(context, sessionOk, abortRequested);
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
    if (!m_runningFlag.load() || m_stop.load()) {
        return;
    }
    const auto frameReady = context.TryPopLatest<SlamFrameReady>(0);
    if (!frameReady || frameReady->sessionId == 0 ||
        context.OutputSize(0) > 0) {
        return;
    }

    SlamPrepareFrameResult result =
        m_service->AcquireAndPrepareFrame(frameReady->sessionId);
    if (!result.sessionAvailable) {
        return;
    }
    if (result.abortRequested) {
        PushStatus(context, result.sessionOk, true);
        return;
    }
    if (!result.frame) {
        return;
    }

    auto prepared = context.Make<SlamPreparedFrame>();
    prepared->sessionId = frameReady->sessionId;
    prepared->frame = std::move(result.frame);
    context.Push(0, std::move(prepared));
}

void SlamAcquireTask::PushStatus(epg::TaskContext &context, bool sessionOk,
                                 bool abortRequested)
{
    PushSlamStatus(context, sessionOk, abortRequested);
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
    if (!m_runningFlag.load() || m_stop.load()) {
        return;
    }
    const auto prepared = context.TryPopLatest<SlamPreparedFrame>(0);
    if (!prepared || prepared->sessionId == 0 || !prepared->frame) {
        return;
    }

    SlamTrackFrameResult result =
        m_service->TrackPreparedFrame(prepared->sessionId,
                                      std::move(prepared->frame));
    if (!result.sessionAvailable) {
        return;
    }
    if (result.abortRequested) {
        PushStatus(context, result.sessionOk, true);
        return;
    }
    if (!result.frame) {
        return;
    }

    auto tracked = context.Make<SlamTrackedFrame>();
    tracked->sessionId = prepared->sessionId;
    tracked->frame = std::move(result.frame);
    context.Push(0, std::move(tracked));
}

void SlamTrackingTask::PushStatus(epg::TaskContext &context, bool sessionOk,
                                  bool abortRequested)
{
    PushSlamStatus(context, sessionOk, abortRequested);
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
    if (!m_runningFlag.load() || m_stop.load()) {
        return;
    }
    const auto tracked = context.TryPopLatest<SlamTrackedFrame>(0);
    if (!tracked || tracked->sessionId == 0 || !tracked->frame) {
        return;
    }

    SlamPublishFrameResult result =
        m_service->PostprocessTrackedFrame(tracked->sessionId,
                                           std::move(tracked->frame));
    if (!result.sessionAvailable) {
        return;
    }
    if (result.abortRequested) {
        PushStatus(context, result.sessionOk, true);
        return;
    }
    if (!result.frame) {
        return;
    }

    auto published = context.Make<SlamPublishedFrame>();
    published->sessionId = tracked->sessionId;
    published->frame = std::move(result.frame);
    context.Push(0, std::move(published));
}

void SlamPosePostprocessTask::PushStatus(epg::TaskContext &context,
                                         bool sessionOk,
                                         bool abortRequested)
{
    PushSlamStatus(context, sessionOk, abortRequested);
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
    if (!m_runningFlag.load() || m_stop.load()) {
        return;
    }
    const auto published = context.TryPopLatest<SlamPublishedFrame>(0);
    if (!published || published->sessionId == 0 || !published->frame) {
        return;
    }

    SlamTaskStepResult result =
        m_service->EmitPointCloud(published->sessionId, *published->frame);
    if (!result.sessionAvailable) {
        return;
    }
    if (result.abortRequested) {
        PushStatus(context, result.sessionOk, true);
        return;
    }

    context.Push(0, published);
}

void SlamPointCloudTask::PushStatus(epg::TaskContext &context, bool sessionOk,
                                    bool abortRequested)
{
    PushSlamStatus(context, sessionOk, abortRequested);
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
    if (!m_runningFlag.load() || m_stop.load()) {
        return;
    }
    const auto published = context.TryPopLatest<SlamPublishedFrame>(0);
    if (!published || published->sessionId == 0 || !published->frame) {
        return;
    }

    SlamTaskStepResult result =
        m_service->EmitDfx(published->sessionId, *published->frame);
    if (!result.sessionAvailable) {
        return;
    }
    PushStatus(context, result.sessionOk, result.abortRequested);
}

void SlamDfxTask::PushStatus(epg::TaskContext &context, bool sessionOk,
                             bool abortRequested)
{
    PushSlamStatus(context, sessionOk, abortRequested);
}

SlamUdpTask::SlamUdpTask(std::shared_ptr<SlamSessionRuntimeService> service,
                         std::atomic<bool> &stop,
                         std::atomic<bool> &runningFlag)
    : m_service(std::move(service)), m_stop(stop), m_runningFlag(runningFlag)
{
}

void SlamUdpTask::OnTick(epg::TaskContext &context)
{
    if (!m_runningFlag.load() || m_stop.load()) {
        return;
    }
    const auto published = context.TryPopLatest<SlamPublishedFrame>(0);
    if (!published || published->sessionId == 0 || !published->frame) {
        return;
    }

    SlamTaskStepResult result =
        m_service->EmitUdp(published->sessionId, *published->frame);
    if (!result.sessionAvailable) {
        return;
    }
    if (result.abortRequested) {
        PushStatus(context, result.sessionOk, true);
        return;
    }

    context.Push(0, published);
}

void SlamUdpTask::PushStatus(epg::TaskContext &context, bool sessionOk,
                             bool abortRequested)
{
    PushSlamStatus(context, sessionOk, abortRequested);
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
    if (!m_runningFlag.load() || m_stop.load()) {
        return;
    }
    const auto published = context.TryPopLatest<SlamPublishedFrame>(0);
    if (!published || published->sessionId == 0 || !published->frame) {
        return;
    }

    SlamTaskStepResult result =
        m_service->EmitMavlink(published->sessionId, *published->frame);
    if (!result.sessionAvailable) {
        return;
    }
    if (result.abortRequested) {
        PushStatus(context, result.sessionOk, true);
        return;
    }

    context.Push(0, published);
}

void SlamMavlinkTask::PushStatus(epg::TaskContext &context, bool sessionOk,
                                 bool abortRequested)
{
    PushSlamStatus(context, sessionOk, abortRequested);
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
    if (!m_runningFlag.load() || m_stop.load()) {
        return;
    }
    const auto published = context.TryPopLatest<SlamPublishedFrame>(0);
    if (!published || published->sessionId == 0 || !published->frame) {
        return;
    }

    SlamTaskStepResult result =
        m_service->EmitLivePose(published->sessionId, *published->frame);
    if (!result.sessionAvailable) {
        return;
    }
    if (result.abortRequested) {
        PushStatus(context, result.sessionOk, true);
        return;
    }

    context.Push(0, published);
}

void SlamLivePoseTask::PushStatus(epg::TaskContext &context, bool sessionOk,
                                  bool abortRequested)
{
    PushSlamStatus(context, sessionOk, abortRequested);
}

SlamMonitorTask::SlamMonitorTask(std::atomic<bool> &stop,
                                 std::atomic<bool> &sessionOk)
    : m_stop(stop), m_sessionOk(sessionOk)
{
}

void SlamMonitorTask::OnTick(epg::TaskContext &context)
{
    while (auto status = context.TryPop<SlamStatus>(0)) {
        m_sessionOk.store(status->sessionOk, std::memory_order_relaxed);
        if (status->abortRequested) {
            m_stop.store(true);
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
