#include "core/application/session/epg/slam_session_tasks.h"

#include <array>
#include <cstdint>
#include <utility>

#include "core/application/config/runtime_app_types.h"
#include "core/application/session/epg/messages/slam_epg_messages.h"
#include "core/application/session/slam/slam_session_runtime_service.h"

namespace SmartDrone::Core::Application {
namespace {

constexpr Epg::PortId STATUS_OUTPUT_PORT = 1;
constexpr Epg::PortId POSE_POSTPROCESS_STATUS_OUTPUT_PORT = 4;
constexpr Epg::PortId DFX_STATUS_OUTPUT_PORT = 0;
constexpr Epg::PortId PREVIEW_READY_OUTPUT_PORT = 0;
constexpr std::array<Epg::PortId, 4> TRACKED_FRAME_INPUT_PORTS{0, 1, 2, 3};
constexpr std::array<Epg::PortId, 13> SLAM_MONITOR_STATUS_INPUT_PORTS{
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
    10,
    11,
    12,
};

enum class SlamTrackingRoute : std::uint8_t {
    Klt,
    Dpvo,
    Orb,
    VisualFeature,
};

bool ShouldRunTask(const std::atomic<bool> &runningFlag,
                   const std::atomic<bool> &stop)
{
    return runningFlag.load() && !stop.load();
}

using PublishedOutputFn = SlamTaskStepResult (
    SlamSessionRuntimeService::*)(std::uint64_t,
                                  ISlamPublishedFramePayload &);

bool IsVisualFeatureRouteFrontend(FeatureFrontend frontend)
{
    return frontend == FeatureFrontend::SuperPointLightGlue ||
           frontend == FeatureFrontend::XFeatLightGlue;
}

SlamTrackingRoute ResolveTrackingRoute(const SlamPreparedFrame &prepared)
{
    const auto frame = prepared.frame ? prepared.frame->Frame() : nullptr;
    if (!frame) {
        return SlamTrackingRoute::Klt;
    }
    if (frame->slamBackend == SlamBackend::DpvoTensorRt) {
        return SlamTrackingRoute::Dpvo;
    }
    if (frame->slamBackend != SlamBackend::OrbSlam3) {
        return SlamTrackingRoute::Klt;
    }
    return IsVisualFeatureRouteFrontend(frame->featureFrontend)
               ? SlamTrackingRoute::VisualFeature
               : SlamTrackingRoute::Orb;
}

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

template <typename MessageType>
void PushSlamRouteFrame(Epg::TaskContext &context, Epg::PortId port,
                        SlamPreparedFrame &prepared)
{
    auto message = context.Make<MessageType>();
    message->sessionId = prepared.sessionId;
    message->frame = std::move(prepared.frame);
    context.Push(port, std::move(message));
}

template <typename MessageType>
void RunTrackingStageTask(Epg::TaskContext &context,
                          SlamSessionRuntimeService &service)
{
    const auto prepared = context.TryPopLatest<MessageType>(0);
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

std::shared_ptr<SlamTrackedFrame> PopNextTrackedFrame(
    Epg::TaskContext &context)
{
    for (const Epg::PortId port : TRACKED_FRAME_INPUT_PORTS) {
        if (!context.InputExists(port)) {
            continue;
        }
        if (auto frame = context.TryPopLatest<SlamTrackedFrame>(port)) {
            return frame;
        }
    }
    return {};
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
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    const SlamTaskStepResult result = m_service->StepBackendIfIdle();
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

SlamTrackingRouteTask::SlamTrackingRouteTask(
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_stop(stop),
      m_runningFlag(runningFlag)
{
}

void SlamTrackingRouteTask::OnTick(Epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    auto prepared = context.TryPopLatest<SlamPreparedFrame>(0);
    if (!prepared || prepared->sessionId == 0 || !prepared->frame) {
        return;
    }

    switch (ResolveTrackingRoute(*prepared)) {
    case SlamTrackingRoute::Dpvo:
        PushSlamRouteFrame<SlamDpvoPreparedFrame>(context, 1, *prepared);
        break;
    case SlamTrackingRoute::Orb:
        PushSlamRouteFrame<SlamOrbPreparedFrame>(context, 2, *prepared);
        break;
    case SlamTrackingRoute::VisualFeature:
        PushSlamRouteFrame<SlamVisualFeaturePreparedFrame>(context, 3,
                                                           *prepared);
        break;
    case SlamTrackingRoute::Klt:
    default:
        PushSlamRouteFrame<SlamKltPreparedFrame>(context, 0, *prepared);
        break;
    }
}

SlamKltTrackingTask::SlamKltTrackingTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)),
      m_stop(stop),
      m_runningFlag(runningFlag)
{
}

void SlamKltTrackingTask::OnTick(Epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    RunTrackingStageTask<SlamKltPreparedFrame>(context, *m_service);
}

SlamDpvoTrackingTask::SlamDpvoTrackingTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)),
      m_stop(stop),
      m_runningFlag(runningFlag)
{
}

void SlamDpvoTrackingTask::OnTick(Epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    RunTrackingStageTask<SlamDpvoPreparedFrame>(context, *m_service);
}

SlamOrbTrackingTask::SlamOrbTrackingTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)),
      m_stop(stop),
      m_runningFlag(runningFlag)
{
}

void SlamOrbTrackingTask::OnTick(Epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    RunTrackingStageTask<SlamOrbPreparedFrame>(context, *m_service);
}

SlamVisualFeatureTrackingTask::SlamVisualFeatureTrackingTask(
    std::shared_ptr<SlamSessionRuntimeService> service,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_service(std::move(service)),
      m_stop(stop),
      m_runningFlag(runningFlag)
{
}

void SlamVisualFeatureTrackingTask::OnTick(Epg::TaskContext &context)
{
    if (!ShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    RunTrackingStageTask<SlamVisualFeaturePreparedFrame>(context, *m_service);
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
    const auto tracked = PopNextTrackedFrame(context);
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

    PushSlamPublishedFrame(context, 0, tracked->sessionId, result.frame);
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
    context.Push(1, std::move(published));
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

    PushSlamPublishedFrame(context, 0, preview->sessionId, preview->frame);
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
const bool SLAM_TRACKING_ROUTE_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamTrackingRouteTask>(
        "SlamTrackingRouteTask");
const bool SLAM_KLT_TRACKING_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamKltTrackingTask>(
        "SlamKltTrackingTask");
const bool SLAM_DPVO_TRACKING_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamDpvoTrackingTask>(
        "SlamDpvoTrackingTask");
const bool SLAM_ORB_TRACKING_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<SlamOrbTrackingTask>(
        "SlamOrbTrackingTask");
const bool SLAM_VISUAL_FEATURE_TRACKING_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<
        SlamVisualFeatureTrackingTask>("SlamVisualFeatureTrackingTask");
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
