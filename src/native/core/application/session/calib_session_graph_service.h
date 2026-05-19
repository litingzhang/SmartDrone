#pragma once

#include <atomic>
#include <memory>

#include "core/application/config/runtime_app_types.h"
#include "core/application/session/session_graph_runtime.h"
#include "core/application/state/live_pose_state.h"

namespace smartdrone::core::application {

struct CalibSessionGraphRuntimeConfig {
    UnifiedConfig cfg;
    std::atomic<bool> &stop;
    LivePoseState &livePose;
    std::atomic<bool> &runningFlag;
};

class CalibSessionGraphRuntime final : public ISessionGraphRuntime {
  public:
    explicit CalibSessionGraphRuntime(CalibSessionGraphRuntimeConfig config);
    ~CalibSessionGraphRuntime();

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
