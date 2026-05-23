#include "core/application/session/epg/slam_session_graph_service.h"

#include <atomic>
#include <memory>
#include <utility>

#include "common/epg/epg.h"
#include "core/application/config/runtime_app_types.h"
#include "core/application/epg/epg_graph_runtime.h"
#include "core/application/runtime/epg_dfx_snapshot.h"
#include "core/application/runtime/epg_graph_lifecycle.h"
#include "core/application/runtime/application_runtime_factories.h"
#include "core/application/state/live_pose_state.h"
#include "core/application/session/epg/slam_session_task_factory.h"
#include "core/application/session/epg/slam_session_task_utils.h"
#include "core/application/session/slam/slam_session_runtime_service.h"
#include "core/ports/pose_publisher.h"
#include "core/ports/slam_session_telemetry.h"

namespace SmartDrone::Core::Application {

class SlamSessionGraphRuntime::Impl final {
  public:
    explicit Impl(SlamSessionGraphRuntimeConfig config)
        : m_cfg(config.cfg),
          m_tuning(config.tuning),
          m_telemetry(config.telemetry),
          m_posePublisher(config.posePublisher),
          m_stop(config.stop),
          m_livePose(config.livePose),
          m_runningFlag(config.runningFlag),
          m_factories(config.factories),
          m_lifecycle(EpgGraphLifecycleConfig{
              m_stop,
              [this]() { return ResourcesStopped(); },
              [this]() { StopResources(); },
              [this]() { ResetResources(); },
          })
    {
    }

    ~Impl()
    {
        Stop();
    }

    bool Start()
    {
        if (m_lifecycle.HasGraph()) {
            return true;
        }
        m_lifecycle.ResetForStart();
        m_sessionOk.store(true, std::memory_order_relaxed);
        m_runtimeService = std::make_shared<SlamSessionRuntimeService>(RuntimeConfig());
        auto graphRef = std::make_shared<EpgGraphRef>();
        auto graph = StartEpgGraph({
            EpgDomain::SlamSession,
            MakeSlamGraphTaskFactoryResolver({
                m_runtimeService,
                m_stop,
                m_runningFlag,
                m_sessionOk,
                m_tuning,
                m_cfg.app.camera.fps,
                graphRef,
            }),
            graphRef,
            [this](Epg::GraphConfig &graphConfig) {
                ApplySlamRuntimePacing(graphConfig, m_cfg);
            },
        });
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
        if (!m_runningFlag.load() || m_stop.load()) {
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
    bool Ok() const
    {
        return m_sessionOk.load(std::memory_order_relaxed);
    }

  private:
    SlamSessionRuntimeServiceConfig RuntimeConfig()
    {
        return {
            m_cfg,
            m_tuning,
            m_telemetry,
            m_posePublisher,
            m_stop,
            m_livePose,
            m_runningFlag,
            m_factories,
        };
    }

    bool ResourcesStopped() const
    {
        return m_runtimeService && m_runtimeService->Stopped();
    }

    void StopResources()
    {
        if (m_runtimeService) {
            m_runtimeService->Stop();
        }
    }

    void ResetResources()
    {
        m_runtimeService.reset();
    }

    UnifiedConfig m_cfg;
    LiveRuntimeTuning &m_tuning;
    SmartDrone::Core::Ports::ISlamSessionTelemetryPort &m_telemetry;
    SmartDrone::Core::Ports::IPosePublisher &m_posePublisher;
    std::atomic<bool> &m_stop;
    LivePoseState &m_livePose;
    std::atomic<bool> &m_runningFlag;
    const ApplicationRuntimeFactories &m_factories;
    std::shared_ptr<SlamSessionRuntimeService> m_runtimeService;
    EpgGraphLifecycle m_lifecycle;
    std::atomic<bool> m_sessionOk{true};
};

SlamSessionGraphRuntime::SlamSessionGraphRuntime(SlamSessionGraphRuntimeConfig config)
    : m_impl(new Impl(std::move(config)))
{
}

SlamSessionGraphRuntime::~SlamSessionGraphRuntime() = default;

bool SlamSessionGraphRuntime::Start()
{
    return m_impl->Start();
}

void SlamSessionGraphRuntime::Step()
{
    m_impl->Step();
}

void SlamSessionGraphRuntime::RequestStop()
{
    m_impl->RequestStop();
}

void SlamSessionGraphRuntime::Stop()
{
    m_impl->Stop();
}

bool SlamSessionGraphRuntime::Done()
{
    return m_impl->Done();
}

bool SlamSessionGraphRuntime::Ok() const
{
    return m_impl->Ok();
}

} // namespace SmartDrone::Core::Application
