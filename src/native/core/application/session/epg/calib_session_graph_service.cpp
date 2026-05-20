#include "core/application/session/epg/calib_session_graph_service.h"

#include <atomic>
#include <memory>
#include <utility>

#include "common/epg/epg.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/runtime/epg_dfx_snapshot.h"
#include "core/application/runtime/epg_graph_lifecycle.h"
#include "core/application/session/calib/calib_runtime_state.h"
#include "core/application/epg/epg_registry.h"
#include "core/application/state/live_pose_state.h"
#include "core/application/session/epg/calib_session_task_factory.h"

namespace smartdrone::core::application {

class CalibSessionGraphRuntime::Impl final {
  public:
    explicit Impl(CalibSessionGraphRuntimeConfig config)
        : m_cfg(config.cfg),
          m_stop(config.stop),
          m_livePose(config.livePose),
          m_runningFlag(config.runningFlag),
          m_lifecycle(EpgGraphLifecycleConfig{
              m_stop,
              [this]() { return ResourcesStopped(); },
              {},
              [this]() { ResetResources(); },
          })
    {
    }

    ~Impl() { Stop(); }

    bool Start()
    {
        if (m_lifecycle.HasGraph()) {
            return true;
        }
        m_lifecycle.ResetForStart();
        m_completed.store(false, std::memory_order_relaxed);
        m_sessionOk.store(true, std::memory_order_relaxed);
        m_state = std::make_shared<CalibRuntimeState>(RuntimeStateConfig());
        auto graphRef = std::make_shared<EpgGraphRef>();
        RegisterEpgTypes(m_registry, EpgDomain::CalibSession,
                         MakeCalibGraphTaskFactoryResolver({
                             m_state,
                             m_stop,
                             m_runningFlag,
                             m_sessionOk,
                             m_completed,
                             graphRef,
                         }));
        auto graph = std::make_unique<epg::EventPipelineGraph>(m_registry);
        graphRef->graph = graph.get();
        graph->Configure(CompileEpgConfig(EpgDomain::CalibSession, m_registry));
        graph->Start();
        m_lifecycle.AttachGraph(std::move(graph));
        return true;
    }

    void Step()
    {
        if (m_lifecycle.Done()) {
            return;
        }
        if (m_lifecycle.StopRequested()) {
            m_lifecycle.StepStop();
            return;
        }
        if (!m_lifecycle.HasGraph()) {
            return;
        }
        if (ShouldFinish()) {
            RequestStop();
        }
    }

    void RequestStop()
    {
        m_lifecycle.RequestStop();
    }

    void Stop()
    {
        if (m_lifecycle.Done()) {
            return;
        }
        if (m_lifecycle.HasGraph()) {
            m_lifecycle.StopSynchronously();
        }
    }

    bool Done()
    {
        m_lifecycle.StepStop();
        return m_lifecycle.Done();
    }
    bool Ok() const { return m_sessionOk.load(std::memory_order_relaxed); }

  private:
    CalibRuntimeStateConfig RuntimeStateConfig()
    {
        return {
            m_cfg,
            m_stop,
            m_livePose,
        };
    }

    bool ShouldFinish() const
    {
        return !m_runningFlag.load() || m_stop.load() || m_completed.load(std::memory_order_relaxed);
    }

    bool ResourcesStopped() const
    {
        return m_state && m_state->Finalized();
    }

    void ResetResources()
    {
        m_state.reset();
    }

    UnifiedConfig m_cfg;
    std::atomic<bool> &m_stop;
    LivePoseState &m_livePose;
    std::atomic<bool> &m_runningFlag;
    epg::Registry m_registry;
    std::shared_ptr<CalibRuntimeState> m_state;
    EpgGraphLifecycle m_lifecycle;
    std::atomic<bool> m_sessionOk{true};
    std::atomic<bool> m_completed{false};
};

CalibSessionGraphRuntime::CalibSessionGraphRuntime(CalibSessionGraphRuntimeConfig config)
    : m_impl(new Impl(std::move(config)))
{
}

CalibSessionGraphRuntime::~CalibSessionGraphRuntime() = default;

bool CalibSessionGraphRuntime::Start() { return m_impl->Start(); }

void CalibSessionGraphRuntime::Step() { m_impl->Step(); }

void CalibSessionGraphRuntime::RequestStop() { m_impl->RequestStop(); }

void CalibSessionGraphRuntime::Stop() { m_impl->Stop(); }

bool CalibSessionGraphRuntime::Done() { return m_impl->Done(); }

bool CalibSessionGraphRuntime::Ok() const { return m_impl->Ok(); }

} // namespace smartdrone::core::application
