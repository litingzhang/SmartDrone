#pragma once

#include <atomic>
#include <memory>

#include "core/application/config/runtime_app_types.h"
#include "core/application/session/session_graph_runtime.h"
#include "core/application/state/live_pose_state.h"
#include "core/ports/pose_publisher.h"
#include "core/ports/slam_session_telemetry.h"

namespace smartdrone::core::application {

struct SlamSessionGraphRuntimeConfig {
    UnifiedConfig cfg;
    LiveRuntimeTuning &tuning;
    smartdrone::core::ports::ISlamSessionTelemetryPort &telemetry;
    smartdrone::core::ports::IPosePublisher &posePublisher;
    std::atomic<bool> &stop;
    LivePoseState &livePose;
    std::atomic<bool> &runningFlag;
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

} // namespace smartdrone::core::application
