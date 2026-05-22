#include "core/application/session/epg/calib_session_tasks.h"

#include <utility>

#include "core/application/session/epg/messages/calib_epg_messages.h"

namespace SmartDrone::Core::Application {
namespace {

bool CalibShouldRunTask(const std::atomic<bool> &runningFlag,
                        const std::atomic<bool> &stop)
{
    return runningFlag.load() && !stop.load();
}

} // namespace

CalibResourceTask::CalibResourceTask(
    std::shared_ptr<CalibRuntimeState> state,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_state(std::move(state)), m_stop(stop), m_runningFlag(runningFlag)
{
}

void CalibResourceTask::OnTick(Epg::TaskContext &context)
{
    if (!CalibShouldRunTask(m_runningFlag, m_stop)) {
        EmitStopRequest(context, false);
        return;
    }
    if (m_emitted) {
        return;
    }
    if (!m_state->EnsureStarted()) {
        m_stop.store(true);
        EmitStopRequest(context, false);
        return;
    }
    auto ready = context.Make<CalibResourceReady>();
    ready->ready = true;
    const bool cameraOk = context.Push(0, ready);
    const bool imuOk = context.Push(1, std::move(ready));
    m_emitted = cameraOk || imuOk;
}

void CalibResourceTask::EmitStopRequest(Epg::TaskContext &context,
                                        bool sessionOk)
{
    if (m_stopEmitted) {
        return;
    }
    m_stopEmitted = true;
    auto stop = context.Make<CalibStopRequest>();
    stop->sessionOk = sessionOk;
    context.Push(2, std::move(stop));
}

const bool CALIB_RESOURCE_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<CalibResourceTask>(
        "CalibResourceTask");

CalibClockTask::CalibClockTask(std::atomic<bool> &stop,
                               std::atomic<bool> &runningFlag)
    : m_stop(stop), m_runningFlag(runningFlag)
{
}

void CalibClockTask::OnTick(Epg::TaskContext &context)
{
    if (!CalibShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    auto tick = context.Make<CalibTick>();
    tick->sequence = ++m_sequence;
    context.Push(0, std::move(tick));
}

const bool CALIB_CLOCK_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<CalibClockTask>(
        "CalibClockTask");

CalibCameraAcquireTask::CalibCameraAcquireTask(
    std::shared_ptr<CalibRuntimeState> state,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_state(std::move(state)), m_stop(stop), m_runningFlag(runningFlag)
{
}

void CalibCameraAcquireTask::OnTick(Epg::TaskContext &context)
{
    if (auto ready = context.TryPopLatest<CalibResourceReady>(0)) {
        m_ready = ready->ready;
    }
    const auto tick = context.TryPopLatest<CalibTick>(1);
    if (!m_ready || !tick || !CalibShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    if (m_state->ShouldFinishCapture()) {
        EmitDone(context, true);
        return;
    }

    auto frame = context.Make<CalibStereoFrame>();
    const CalibFrameCaptureResult result = m_state->TryCaptureFrame(*frame);
    if (!HandleCaptureResult(context, result.status)) {
        return;
    }

    context.Push(0, frame);
    context.Push(1, frame);
}

void CalibCameraAcquireTask::EmitDone(Epg::TaskContext &context,
                                      bool sessionOk)
{
    if (m_doneEmitted) {
        return;
    }
    m_doneEmitted = true;
    auto done = context.Make<CalibCaptureDone>();
    done->sessionOk = sessionOk;
    context.Push(2, std::move(done));
}

bool CalibCameraAcquireTask::HandleCaptureResult(
    Epg::TaskContext &context,
    CalibFrameCaptureStatus status)
{
    if (status == CalibFrameCaptureStatus::Captured) {
        return true;
    }
    if (status == CalibFrameCaptureStatus::SessionAbort) {
        EmitDone(context, false);
    }
    return false;
}

const bool CALIB_CAMERA_ACQUIRE_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<CalibCameraAcquireTask>(
        "CalibCameraAcquireTask");

CalibPacingFilterTask::CalibPacingFilterTask(
    std::shared_ptr<CalibRuntimeState> state)
    : m_state(std::move(state))
{
}

void CalibPacingFilterTask::OnTick(Epg::TaskContext &context)
{
    while (auto frame = context.TryPop<CalibStereoFrame>(0)) {
        auto save = context.Make<CalibSavePair>();
        if (!m_state->TryBuildSavePair(std::move(frame), *save)) {
            continue;
        }
        context.Push(0, std::move(save));
    }
}

const bool CALIB_PACING_FILTER_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<CalibPacingFilterTask>(
        "CalibPacingFilterTask");

CalibStorageWriteTask::CalibStorageWriteTask(
    std::shared_ptr<CalibRuntimeState> state)
    : m_state(std::move(state))
{
}

void CalibStorageWriteTask::OnTick(Epg::TaskContext &context)
{
    while (auto save = context.TryPop<CalibSavePair>(0)) {
        auto status = context.Make<CalibStorageStatus>();
        status->ok = m_state->WriteSavePair(*save);
        context.Push(0, std::move(status));
    }
}

const bool CALIB_STORAGE_WRITE_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<CalibStorageWriteTask>(
        "CalibStorageWriteTask");

CalibUdpPreviewTask::CalibUdpPreviewTask(
    std::shared_ptr<CalibRuntimeState> state)
    : m_state(std::move(state))
{
}

void CalibUdpPreviewTask::OnTick(Epg::TaskContext &context)
{
    while (auto frame = context.TryPopLatest<CalibStereoFrame>(0)) {
        auto status = context.Make<CalibPreviewStatus>();
        status->ok = m_state->EnqueuePreview(*frame);
        context.Push(0, std::move(status));
    }
}

const bool CALIB_UDP_PREVIEW_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<CalibUdpPreviewTask>(
        "CalibUdpPreviewTask");

CalibImuWriterTask::CalibImuWriterTask(
    std::shared_ptr<CalibRuntimeState> state,
    std::atomic<bool> &stop,
    std::atomic<bool> &runningFlag)
    : m_state(std::move(state)), m_stop(stop), m_runningFlag(runningFlag)
{
}

void CalibImuWriterTask::OnTick(Epg::TaskContext &context)
{
    if (auto ready = context.TryPopLatest<CalibResourceReady>(0)) {
        m_ready = ready->ready;
    }
    if (!m_ready || !CalibShouldRunTask(m_runningFlag, m_stop)) {
        return;
    }
    const CalibImuSampleResult result = m_state->StepImuSample();
    PushResult(context, result.status);
}

void CalibImuWriterTask::PushResult(Epg::TaskContext &context,
                                    CalibImuSampleStatus status)
{
    if (status == CalibImuSampleStatus::Pending) {
        return;
    }
    auto imuStatus = context.Make<CalibImuStatus>();
    imuStatus->ok = status == CalibImuSampleStatus::Written;
    context.Push(0, std::move(imuStatus));
}

const bool CALIB_IMU_WRITER_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<CalibImuWriterTask>(
        "CalibImuWriterTask");

CalibCompletionTask::CalibCompletionTask(
    std::shared_ptr<CalibRuntimeState> state)
    : m_state(std::move(state))
{
}

void CalibCompletionTask::OnTick(Epg::TaskContext &context)
{
    while (auto status = context.TryPop<CalibStorageStatus>(1)) {
        if (!status->ok) {
            m_sessionOk = false;
        }
    }
    while (auto status = context.TryPop<CalibImuStatus>(2)) {
        if (!status->ok) {
            m_sessionOk = false;
        }
    }
    while (auto status = context.TryPop<CalibPreviewStatus>(3)) {
        if (!status->ok) {
            m_sessionOk = false;
        }
    }
    while (auto done = context.TryPop<CalibCaptureDone>(0)) {
        m_sessionOk = m_sessionOk && done->sessionOk;
        EmitFlush(context);
    }
    while (auto stop = context.TryPop<CalibStopRequest>(4)) {
        m_sessionOk = m_sessionOk && stop->sessionOk;
        EmitFlush(context);
    }
    if (!m_flushEmitted && m_state->ShouldFinishCapture()) {
        EmitFlush(context);
    }
}

void CalibCompletionTask::EmitFlush(Epg::TaskContext &context)
{
    if (m_flushEmitted) {
        return;
    }
    m_flushEmitted = true;
    auto flush = context.Make<CalibFlushRequest>();
    flush->sessionOk = m_sessionOk;
    context.Push(0, std::move(flush));
}

const bool CALIB_COMPLETION_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<CalibCompletionTask>(
        "CalibCompletionTask");

CalibFlushSyncTask::CalibFlushSyncTask(
    std::shared_ptr<CalibRuntimeState> state,
    std::atomic<bool> &completed,
    std::atomic<bool> &sessionOk)
    : m_state(std::move(state)), m_completed(completed), m_sessionOk(sessionOk)
{
}

void CalibFlushSyncTask::OnTick(Epg::TaskContext &context)
{
    if (m_completed.load(std::memory_order_relaxed)) {
        return;
    }
    if (auto flush = context.TryPopLatest<CalibFlushRequest>(0)) {
        m_state->Finalize(flush->sessionOk);
        m_sessionOk.store(flush->sessionOk, std::memory_order_relaxed);
        auto status = context.Make<CalibStatus>();
        status->sessionOk = flush->sessionOk;
        status->completed = true;
        context.Push(0, std::move(status));
    }
}

const bool CALIB_FLUSH_SYNC_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<CalibFlushSyncTask>(
        "CalibFlushSyncTask");

CalibMonitorTask::CalibMonitorTask(std::atomic<bool> &sessionOk,
                                   std::atomic<bool> &completed)
    : m_sessionOk(sessionOk), m_completed(completed)
{
}

void CalibMonitorTask::OnTick(Epg::TaskContext &context)
{
    while (auto status = context.TryPop<CalibStatus>(0)) {
        m_sessionOk.store(status->sessionOk, std::memory_order_relaxed);
        if (status->completed) {
            m_completed.store(true, std::memory_order_relaxed);
        }
    }
}

const bool CALIB_MONITOR_TASK_REGISTERED =
    Epg::TypeCatalog::Global().RegisterTaskType<CalibMonitorTask>(
        "CalibMonitorTask");

} // namespace SmartDrone::Core::Application
