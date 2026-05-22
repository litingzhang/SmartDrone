#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include "common/epg/epg.h"
#include "core/application/session/calib/calib_runtime_state.h"

namespace SmartDrone::core::application {

class CalibResourceTask final : public Epg::ITask {
  public:
    CalibResourceTask(std::shared_ptr<CalibRuntimeState> state,
                      std::atomic<bool> &stop,
                      std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    void EmitStopRequest(Epg::TaskContext &context, bool sessionOk);

    std::shared_ptr<CalibRuntimeState> m_state;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    bool m_emitted{false};
    bool m_stopEmitted{false};
};

class CalibClockTask final : public Epg::ITask {
  public:
    CalibClockTask(std::atomic<bool> &stop, std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    std::uint64_t m_sequence{0};
};

class CalibCameraAcquireTask final : public Epg::ITask {
  public:
    CalibCameraAcquireTask(std::shared_ptr<CalibRuntimeState> state,
                           std::atomic<bool> &stop,
                           std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    void EmitDone(Epg::TaskContext &context, bool sessionOk);
    bool HandleCaptureResult(Epg::TaskContext &context,
                             CalibFrameCaptureStatus status);

    std::shared_ptr<CalibRuntimeState> m_state;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    bool m_ready{false};
    bool m_doneEmitted{false};
};

class CalibPacingFilterTask final : public Epg::ITask {
  public:
    explicit CalibPacingFilterTask(std::shared_ptr<CalibRuntimeState> state);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<CalibRuntimeState> m_state;
};

class CalibStorageWriteTask final : public Epg::ITask {
  public:
    explicit CalibStorageWriteTask(std::shared_ptr<CalibRuntimeState> state);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<CalibRuntimeState> m_state;
};

class CalibUdpPreviewTask final : public Epg::ITask {
  public:
    explicit CalibUdpPreviewTask(std::shared_ptr<CalibRuntimeState> state);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<CalibRuntimeState> m_state;
};

class CalibImuWriterTask final : public Epg::ITask {
  public:
    CalibImuWriterTask(std::shared_ptr<CalibRuntimeState> state,
                       std::atomic<bool> &stop,
                       std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    void PushResult(Epg::TaskContext &context,
                    CalibImuSampleStatus status);

    std::shared_ptr<CalibRuntimeState> m_state;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    bool m_ready{false};
};

class CalibCompletionTask final : public Epg::ITask {
  public:
    explicit CalibCompletionTask(std::shared_ptr<CalibRuntimeState> state);
    void OnTick(Epg::TaskContext &context) override;

  private:
    void EmitFlush(Epg::TaskContext &context);

    std::shared_ptr<CalibRuntimeState> m_state;
    bool m_sessionOk{true};
    bool m_flushEmitted{false};
};

class CalibFlushSyncTask final : public Epg::ITask {
  public:
    CalibFlushSyncTask(std::shared_ptr<CalibRuntimeState> state,
                       std::atomic<bool> &completed,
                       std::atomic<bool> &sessionOk);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<CalibRuntimeState> m_state;
    std::atomic<bool> &m_completed;
    std::atomic<bool> &m_sessionOk;
};

class CalibMonitorTask final : public Epg::ITask {
  public:
    CalibMonitorTask(std::atomic<bool> &sessionOk,
                     std::atomic<bool> &completed);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::atomic<bool> &m_sessionOk;
    std::atomic<bool> &m_completed;
};

} // namespace SmartDrone::core::application
