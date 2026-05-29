#include "core/application/session/epg/slam_session_tasks.h"

#include <array>
#include <utility>

#include "core/application/session/epg/messages/slam_epg_messages.h"
#include "core/application/session/slam/slam_session_runtime_service.h"

namespace SmartDrone::Core::Application {
namespace {

constexpr Epg::PortId STATUS_OUTPUT_PORT = 1;
constexpr Epg::PortId POSE_POSTPROCESS_STATUS_OUTPUT_PORT = 5;
constexpr Epg::PortId DFX_STATUS_OUTPUT_PORT = 0;
constexpr Epg::PortId PREVIEW_READY_OUTPUT_PORT = 0;
constexpr std::array<Epg::PortId, 5> PUBLISHED_FRAME_OUTPUT_PORTS{0, 1, 2, 3,
                                                                   4};
constexpr std::array<Epg::PortId, 10> SLAM_MONITOR_STATUS_INPUT_PORTS{
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
};

bool ShouldRunTask(const std::atomic<bool> &runningFlag,
                   const std::atomic<bool> &stop)
{
    return runningFlag.load() && !stop.load();
}

using PublishedOutputFn = SlamTaskStepResult (
    SlamSessionRuntimeService::*)(std::uint64_t,
                                  ISlamPublishedFramePayload &);

void PushSlamStatus(Epg::TaskContext &context, bool sessionOk,
                    bool abortRequested,
                    Epg::PortId port)
{
    auto status = context.Make<SlamStatus>();
    status->sessionOk = sessionOk;
    status->abortRequested = abortRequested;
    context.Push(port, std::move(status));
}

void PushSlamPublishedFrame(
    Epg::TaskContext &context,
    Epg::PortId port,
    std::uint64_t sessionId,
    const std::shared_ptr<ISlamPublishedFramePayload> &frame)
{
    auto published = context.Make<SlamPublishedFrame>();
    published->sessionId = sessionId;
    published->frame = frame;
    context.Push(port, std::move(published));
}

void PushSlamPreviewReady(
    Epg::TaskContext &context,
    Epg::PortId port,
    std::uint64_t sessionId,
    const std::shared_ptr<ISlamPublishedFramePayload> &frame)
{
    auto ready = context.Make<SlamPreviewReady>();
    ready->sessionId = sessionId;
    ready->frame = frame;
    context.Push(port, std::move(ready));
}

bool StepResultAllowsContinue(Epg::TaskContext &context,
                              const SlamTaskStepResult &result,
                              Epg::PortId statusPort)
{
    context.ReportResourceWait(result.resourceWaitUs);
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
void PushSlamStageFrame(Epg::TaskContext &context, Epg::PortId port,
                        std::uint64_t sessionId, FramePayload frame)
{
    auto message = context.Make<MessageType>();
    message->sessionId = sessionId;
    message->frame = std::move(frame);
    context.Push(port, std::move(message));
}

void RunTrackingStageTask(Epg::TaskContext &context,
                          SlamSessionRuntimeService &service)
{
    const auto prepared = context.TryPopLatest<SlamPreparedFrame>(0);
    if (!prepared || prepared->sessionId == 0 || !prepared->frame) {
        return;
    }

    SlamTrackFrameResult result =
        service.TrackPreparedFrame(prepared->sessionId,
                                   std::move(prepared->frame));
    if (!StepResultAllowsContinue(context, result, STATUS_OUTPUT_PORT)) {
        return;
    }
    if (!result.frame) {
        return;
    }

    PushSlamStageFrame<SlamTrackedFrame>(
        context, 0, prepared->sessionId, std::move(result.frame));
}

bool ValidPublishedFrame(const std::shared_ptr<SlamPublishedFrame> &frame)
{
    return frame && frame->sessionId != 0 && frame->frame;
}

void RunPublishedOutputTask(Epg::TaskContext &context,
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

    context.Push(0, std::move(published));
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

void SlamResourceTask::OnTick(Epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        if (!m_backendStopRequested) {
            m_service->RequestBackendStop();
            m_backendStopRequested = true;
        }
        if (!m_service->BackendStopped()) {
            return;
        }
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

void SlamClockTask::OnTick(Epg::TaskContext &context)
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

void SlamImuPollTask::OnTick(Epg::TaskContext &context)
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

void SlamBackendTickTask::OnTick(Epg::TaskContext &context)
{
    (void)context;
    if (!ShouldRunTask(m_runningFlag, m_stop) &&
        m_service->BackendStopped()) {
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
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)),
      m_stop(stop),
      m_runningFlag(runningFlag)
{
}

void SlamImuGateTask::OnTick(Epg::TaskContext &context)
{
    if (auto ready = context.TryPopLatest<SlamImuReady>(0)) {
        m_imuReady = ready->ready;
    }
    if (m_service->StartFailed()) {
        PushSlamStatus(context, false, true, STATUS_OUTPUT_PORT);
        return;
    }
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    if (!m_imuReady || context.OutputSize(0) > 0) {
        return;
    }

    const auto tick = context.TryPopLatest<SlamTick>(1);
    if (!tick) {
        return;
    }

    const std::uint64_t sessionId = m_service->SessionId();
    if (sessionId == 0) {
        return;
    }
    auto frameReady = context.Make<SlamFrameReady>();
    frameReady->sessionId = sessionId;
    context.Push(0, std::move(frameReady));
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

void SlamAcquireTask::OnTick(Epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    if (context.OutputSize(0) > 0) {
        return;
    }
    const auto frameReady = context.TryPopLatest<SlamFrameReady>(0);
    if (!frameReady || frameReady->sessionId == 0) {
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

    PushSlamStageFrame<SlamPreparedFrame>(
        context, 0, frameReady->sessionId, std::move(result.frame));
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

void SlamTrackingTask::OnTick(Epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    RunTrackingStageTask(context, *m_service);
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

void SlamPosePostprocessTask::OnTick(Epg::TaskContext &context)
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

    for (const auto port : PUBLISHED_FRAME_OUTPUT_PORTS) {
        PushSlamPublishedFrame(context, port, tracked->sessionId, result.frame);
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

void SlamPointCloudTask::OnTick(Epg::TaskContext &context)
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

void SlamDfxTask::OnTick(Epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    const auto published = context.TryPopLatest<SlamPublishedFrame>(0);
    if (!ValidPublishedFrame(published)) {
        return;
    }

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

void SlamUdpTask::OnTick(Epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    const auto published = context.TryPopLatest<SlamPublishedFrame>(0);
    if (!published || published->sessionId == 0 || !published->frame) {
        return;
    }

    const SlamTaskStepResult result =
        m_service->EmitUdp(published->sessionId, *published->frame);
    if (!StepResultAllowsContinue(context, result, STATUS_OUTPUT_PORT)) {
        return;
    }

    PushSlamPreviewReady(context, PREVIEW_READY_OUTPUT_PORT,
                         published->sessionId, published->frame);
    PushSlamStatus(context, result.sessionOk, false, STATUS_OUTPUT_PORT);
}

SlamPreviewTxTask::SlamPreviewTxTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)), m_stop(stop), m_runningFlag(runningFlag)
{
}

void SlamPreviewTxTask::OnTick(Epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    const auto preview = context.TryPopLatest<SlamPreviewReady>(0);
    if (!preview || preview->sessionId == 0 || !preview->frame) {
        return;
    }

    const SlamTaskStepResult result =
        m_service->FlushPreview(preview->sessionId, *preview->frame);
    if (!StepResultAllowsContinue(context, result, STATUS_OUTPUT_PORT)) {
        return;
    }

    PushSlamStatus(context, result.sessionOk, false, STATUS_OUTPUT_PORT);
}

SlamMavlinkTask::SlamMavlinkTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)), m_stop(stop), m_runningFlag(runningFlag)
{
}

void SlamMavlinkTask::OnTick(Epg::TaskContext &context)
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

void SlamLivePoseTask::OnTick(Epg::TaskContext &context)
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

void SlamMonitorTask::OnTick(Epg::TaskContext &context)
{
    for (const Epg::PortId port : SLAM_MONITOR_STATUS_INPUT_PORTS) {
        while (auto status = context.TryPop<SlamStatus>(port)) {
            m_sessionOk.store(status->sessionOk, std::memory_order_relaxed);
            if (status->abortRequested) {
                m_stop.store(true);
            }
        }
    }
}

const bool SLAM_RESOURCE_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamResourceTask>(
        "SlamResourceTask");
const bool SLAM_CLOCK_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamClockTask>(
        "SlamClockTask");
const bool SLAM_IMU_POLL_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamImuPollTask>(
        "SlamImuPollTask");
const bool SLAM_BACKEND_TICK_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamBackendTickTask>(
        "SlamBackendTickTask");
const bool SLAM_IMU_GATE_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamImuGateTask>(
        "SlamImuGateTask");
const bool SLAM_ACQUIRE_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamAcquireTask>(
        "SlamAcquireTask");
const bool SLAM_TRACKING_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamTrackingTask>(
        "SlamTrackingTask");
const bool SLAM_POSE_POSTPROCESS_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamPosePostprocessTask>(
        "SlamPosePostprocessTask");
const bool SLAM_POINT_CLOUD_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamPointCloudTask>(
        "SlamPointCloudTask");
const bool SLAM_DFX_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamDfxTask>(
        "SlamDfxTask");
const bool SLAM_UDP_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamUdpTask>(
        "SlamUdpTask");
const bool SLAM_PREVIEW_TX_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamPreviewTxTask>(
        "SlamPreviewTxTask");
const bool SLAM_MAVLINK_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamMavlinkTask>(
        "SlamMavlinkTask");
const bool SLAM_LIVE_POSE_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamLivePoseTask>(
        "SlamLivePoseTask");
const bool SLAM_MONITOR_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamMonitorTask>(
        "SlamMonitorTask");

} // namespace SmartDrone::Core::Application
