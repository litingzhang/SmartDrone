#pragma once

#include <array>
#include <atomic>
#include <memory>

#include "common/epg/epg.h"

namespace SmartDrone::Core::Application {

class SlamSessionRuntimeService;
struct SlamPublishedFrame;

class SlamResourceTask final : public Epg::ITask {
  public:
    SlamResourceTask(std::shared_ptr<SlamSessionRuntimeService> service,
                     std::atomic<bool> &stop,
                     std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    bool m_readyEmitted{false};
    bool m_backendStopRequested{false};
};

class SlamClockTask final : public Epg::ITask {
  public:
    SlamClockTask(std::atomic<bool> &stop, std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    std::uint64_t m_sequence{0};
};

class SlamImuPollTask final : public Epg::ITask {
  public:
    SlamImuPollTask(std::shared_ptr<SlamSessionRuntimeService> service,
                    std::atomic<bool> &stop,
                    std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamBackendTickTask final : public Epg::ITask {
  public:
    SlamBackendTickTask(std::shared_ptr<SlamSessionRuntimeService> service,
                        std::atomic<bool> &stop,
                        std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamImuGateTask final : public Epg::ITask {
  public:
    SlamImuGateTask(std::shared_ptr<SlamSessionRuntimeService> service,
                    std::atomic<bool> &stop,
                    std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    bool m_imuReady{false};
};

class SlamAcquireTask final : public Epg::ITask {
  public:
    SlamAcquireTask(std::shared_ptr<SlamSessionRuntimeService> service,
                    std::atomic<bool> &stop,
                    std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamTrackingTask final : public Epg::ITask {
  public:
    SlamTrackingTask(std::shared_ptr<SlamSessionRuntimeService> service,
                     std::atomic<bool> &stop,
                     std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamPosePostprocessTask final : public Epg::ITask {
  public:
    SlamPosePostprocessTask(std::shared_ptr<SlamSessionRuntimeService> service,
                            std::atomic<bool> &stop,
                            std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamPointCloudTask final : public Epg::ITask {
  public:
    SlamPointCloudTask(std::shared_ptr<SlamSessionRuntimeService> service,
                       std::atomic<bool> &stop,
                       std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamDfxTask final : public Epg::ITask {
  public:
    SlamDfxTask(std::shared_ptr<SlamSessionRuntimeService> service,
                std::atomic<bool> &stop,
                std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamUdpTask final : public Epg::ITask {
  public:
    SlamUdpTask(std::shared_ptr<SlamSessionRuntimeService> service,
                std::atomic<bool> &stop,
                std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamPreviewTxTask final : public Epg::ITask {
  public:
    SlamPreviewTxTask(std::shared_ptr<SlamSessionRuntimeService> service,
                      std::atomic<bool> &stop,
                      std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamMavlinkTask final : public Epg::ITask {
  public:
    SlamMavlinkTask(std::shared_ptr<SlamSessionRuntimeService> service,
                    std::atomic<bool> &stop,
                    std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamLivePoseTask final : public Epg::ITask {
  public:
    SlamLivePoseTask(std::shared_ptr<SlamSessionRuntimeService> service,
                     std::atomic<bool> &stop,
                     std::atomic<bool> &runningFlag);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamMonitorTask final : public Epg::ITask {
  public:
    SlamMonitorTask(std::atomic<bool> &stop, std::atomic<bool> &sessionOk);
    void OnTick(Epg::TaskContext &context) override;

  private:
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_sessionOk;
};

} // namespace SmartDrone::Core::Application
