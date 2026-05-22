#pragma once

#include <atomic>
#include <memory>

#include "core/application/session/epg/session_graph_runtime.h"

namespace SmartDrone::core::ports {
class IPosePublisher;
class ISlamSessionTelemetryPort;
} // namespace SmartDrone::core::ports

namespace SmartDrone::core::application {

struct ApplicationRuntimeFactories;
struct LivePoseState;
struct LiveRuntimeTuning;
struct UnifiedConfig;

struct SlamSessionGraphRuntimeConfig {
    const UnifiedConfig &cfg;
    LiveRuntimeTuning &tuning;
    SmartDrone::core::ports::ISlamSessionTelemetryPort &telemetry;
    SmartDrone::core::ports::IPosePublisher &posePublisher;
    std::atomic<bool> &stop;
    LivePoseState &livePose;
    std::atomic<bool> &runningFlag;
    const ApplicationRuntimeFactories &factories;
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

} // namespace SmartDrone::core::application
