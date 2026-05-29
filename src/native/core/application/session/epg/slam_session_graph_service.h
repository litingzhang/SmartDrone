#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "core/application/session/epg/session_graph_runtime.h"

namespace SmartDrone::Core::Ports {
class IPosePublisher;
class ISlamSessionTelemetryPort;
} // namespace SmartDrone::Core::Ports

namespace SmartDrone::Core::Application {

struct ApplicationRuntimeFactories;
class LivePoseState;
struct LiveRuntimeTuning;
struct UnifiedConfig;

struct SlamSessionGraphRuntimeConfig {
    const UnifiedConfig &cfg;
    LiveRuntimeTuning &tuning;
    SmartDrone::Core::Ports::ISlamSessionTelemetryPort &telemetry;
    SmartDrone::Core::Ports::IPosePublisher &posePublisher;
    std::atomic<bool> &stop;
    LivePoseState &livePose;
    std::atomic<bool> &runningFlag;
    const ApplicationRuntimeFactories &factories;
    std::string finalEurocTrajectory;
};

class SlamSessionGraphRuntime final : public ISessionGraphRuntime {
  public:
    explicit SlamSessionGraphRuntime(SlamSessionGraphRuntimeConfig config);
    ~SlamSessionGraphRuntime();

    bool Start() override;
    void Step() override;
    void RequestStop() override;
    void Stop() override;
    bool Done() override;
    bool Ok() const override;

  private:
    class Impl;

    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Core::Application
