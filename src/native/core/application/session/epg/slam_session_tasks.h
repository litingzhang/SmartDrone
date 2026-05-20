#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <memory>

#include "common/epg/epg.h"

namespace smartdrone::core::application {

struct LiveRuntimeTuning;
class SlamSessionRuntimeService;
struct SlamPublishedFrame;

class SlamResourceTask final : public epg::ITask {
  public:
    SlamResourceTask(std::shared_ptr<SlamSessionRuntimeService> service,
                     std::atomic<bool> &stop,
                     std::atomic<bool> &runningFlag);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    bool m_readyEmitted{false};
};

class SlamClockTask final : public epg::ITask {
  public:
    SlamClockTask(std::atomic<bool> &stop, std::atomic<bool> &runningFlag);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    std::uint64_t m_sequence{0};
};

class SlamImuPollTask final : public epg::ITask {
  public:
    SlamImuPollTask(std::shared_ptr<SlamSessionRuntimeService> service,
                    std::atomic<bool> &stop,
                    std::atomic<bool> &runningFlag);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamBackendTickTask final : public epg::ITask {
  public:
    SlamBackendTickTask(std::shared_ptr<SlamSessionRuntimeService> service,
                        std::atomic<bool> &stop,
                        std::atomic<bool> &runningFlag);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamImuGateTask final : public epg::ITask {
  public:
    SlamImuGateTask(std::shared_ptr<SlamSessionRuntimeService> service,
                    std::atomic<bool> &stop,
                    std::atomic<bool> &runningFlag,
                    LiveRuntimeTuning &tuning,
                    int cameraFps);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    LiveRuntimeTuning &m_tuning;
    int m_cameraFps{};
    std::chrono::steady_clock::time_point m_lastFrameReadyTime{};
    bool m_imuReady{false};
};

class SlamAcquireTask final : public epg::ITask {
  public:
    SlamAcquireTask(std::shared_ptr<SlamSessionRuntimeService> service,
                    std::atomic<bool> &stop,
                    std::atomic<bool> &runningFlag);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamTrackingTask final : public epg::ITask {
  public:
    SlamTrackingTask(std::shared_ptr<SlamSessionRuntimeService> service,
                     std::atomic<bool> &stop,
                     std::atomic<bool> &runningFlag);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamPosePostprocessTask final : public epg::ITask {
  public:
    SlamPosePostprocessTask(std::shared_ptr<SlamSessionRuntimeService> service,
                            std::atomic<bool> &stop,
                            std::atomic<bool> &runningFlag);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamPointCloudTask final : public epg::ITask {
  public:
    SlamPointCloudTask(std::shared_ptr<SlamSessionRuntimeService> service,
                       std::atomic<bool> &stop,
                       std::atomic<bool> &runningFlag);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamDfxTask final : public epg::ITask {
  public:
    SlamDfxTask(std::shared_ptr<SlamSessionRuntimeService> service,
                std::atomic<bool> &stop,
                std::atomic<bool> &runningFlag);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
    std::array<std::shared_ptr<SlamPublishedFrame>, 4> m_pendingFrames{};
};

class SlamUdpTask final : public epg::ITask {
  public:
    SlamUdpTask(std::shared_ptr<SlamSessionRuntimeService> service,
                std::atomic<bool> &stop,
                std::atomic<bool> &runningFlag);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamMavlinkTask final : public epg::ITask {
  public:
    SlamMavlinkTask(std::shared_ptr<SlamSessionRuntimeService> service,
                    std::atomic<bool> &stop,
                    std::atomic<bool> &runningFlag);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamLivePoseTask final : public epg::ITask {
  public:
    SlamLivePoseTask(std::shared_ptr<SlamSessionRuntimeService> service,
                     std::atomic<bool> &stop,
                     std::atomic<bool> &runningFlag);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::shared_ptr<SlamSessionRuntimeService> m_service;
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_runningFlag;
};

class SlamMonitorTask final : public epg::ITask {
  public:
    SlamMonitorTask(std::atomic<bool> &stop, std::atomic<bool> &sessionOk);
    void OnTick(epg::TaskContext &context) override;

  private:
    std::atomic<bool> &m_stop;
    std::atomic<bool> &m_sessionOk;
};

} // namespace smartdrone::core::application
