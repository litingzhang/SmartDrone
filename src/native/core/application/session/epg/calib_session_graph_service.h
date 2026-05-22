#pragma once

#include <atomic>
#include <memory>

#include "core/application/session/epg/session_graph_runtime.h"

namespace SmartDrone::core::application {

struct ApplicationRuntimeFactories;
struct LivePoseState;
struct UnifiedConfig;

struct CalibSessionGraphRuntimeConfig {
    const UnifiedConfig &cfg;
    std::atomic<bool> &stop;
    LivePoseState &livePose;
    std::atomic<bool> &runningFlag;
    const ApplicationRuntimeFactories &factories;
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

} // namespace SmartDrone::core::application
