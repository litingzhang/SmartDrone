#include "core/application/runtime/runtime_session_supervisor.h"

#include <csignal>
#include <functional>
#include <utility>

#include "common/thread_launch.h"

namespace smartdrone::core::application {

namespace {

class FunctionTask final : public smartdrone::runtime_graph::ITask {
  public:
    explicit FunctionTask(std::function<void()> onTick) : m_onTick(std::move(onTick)) {}

    void OnTick(smartdrone::runtime_graph::TaskContext &context) override
    {
        (void)context;
        m_onTick();
    }

  private:
    std::function<void()> m_onTick;
};

smartdrone::runtime_graph::RuntimeGraphConfig MakeRuntimeSupervisorGraphConfig()
{
    smartdrone::runtime_graph::TaskConfig supervisor;
    supervisor.name = "supervisor";
    supervisor.type = "RuntimeSupervisorTask";
    supervisor.trigger.mode = smartdrone::runtime_graph::TriggerMode::Periodic;
    supervisor.trigger.interval = std::chrono::milliseconds(100);

    smartdrone::runtime_graph::RuntimeGraphConfig config;
    config.tasks.push_back(std::move(supervisor));
    return config;
}

} // namespace

RuntimeSessionSupervisor::RuntimeSessionSupervisor(std::atomic<bool> &runningFlag, LiveRuntimeTuning &tuning,
                                                   Px4MavlinkGateway &mav, LivePoseState &livePose,
                                                   CurrentConfigFn currentConfig, SlamSessionRunner slamSessionRunner,
                                                   CalibSessionRunner calibSessionRunner)
    : m_runningFlag(runningFlag), m_tuning(tuning), m_mav(mav), m_livePose(livePose),
      m_currentConfig(std::move(currentConfig)), m_slamSessionRunner(std::move(slamSessionRunner)),
      m_calibSessionRunner(std::move(calibSessionRunner))
{
}

void RuntimeSessionSupervisor::Start()
{
    ConfigureGraph();
    m_graph->Start();
}

void RuntimeSessionSupervisor::Stop()
{
    {
        std::lock_guard<std::mutex> lock(m_mu);
        m_stopping = true;
        m_modeManager.RequestMode(ControllerMode::Idle);
        m_sessionStop.store(true);
    }
    m_cv.notify_all();
    if (m_graph) {
        m_graph->Stop();
    }
    JoinSession();
}

bool RuntimeSessionSupervisor::RequestMode(ControllerMode mode, std::string *err)
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (m_stopping) {
        if (err) {
            *err = "runtime stopping";
        }
        return false;
    }
    m_modeManager.RequestMode(mode);
    m_cv.notify_all();
    return true;
}

void RuntimeSessionSupervisor::RequestRestart()
{
    std::lock_guard<std::mutex> lock(m_mu);
    m_modeManager.RequestRestart();
    m_sessionStop.store(true);
    m_cv.notify_all();
}

RuntimeSessionSupervisor::ControllerMode RuntimeSessionSupervisor::DesiredMode() const
{
    std::lock_guard<std::mutex> lock(m_mu);
    return m_modeManager.DesiredMode();
}

RuntimeSessionSupervisor::ControllerMode RuntimeSessionSupervisor::ActiveMode() const
{
    std::lock_guard<std::mutex> lock(m_mu);
    return m_modeManager.ActiveMode();
}

bool RuntimeSessionSupervisor::WaitForIdle(std::chrono::milliseconds timeout, bool *stoppingOut)
{
    std::unique_lock<std::mutex> lock(m_mu);
    const bool idleReady = m_cv.wait_for(lock, timeout, [this]() {
        return m_stopping ||
               (m_modeManager.ActiveMode() == ControllerMode::Idle &&
                m_modeManager.DesiredMode() == ControllerMode::Idle && !m_session.joinable() && !m_sessionDone);
    });
    if (stoppingOut) {
        *stoppingOut = m_stopping;
    }
    return idleReady;
}

void RuntimeSessionSupervisor::JoinSession()
{
    if (m_session.joinable()) {
        m_session.join();
    }
}

void RuntimeSessionSupervisor::ConfigureGraph()
{
    if (m_graph) {
        return;
    }

    m_graphRegistry.RegisterTaskFactory(
        "RuntimeSupervisorTask", {}, {},
        [this]() {
            return std::unique_ptr<smartdrone::runtime_graph::ITask>(new FunctionTask([this]() { StepSupervisor(); }));
        });
    m_graph.reset(new smartdrone::runtime_graph::RuntimeGraph(m_graphRegistry));
    m_graph->Configure(MakeRuntimeSupervisorGraphConfig());
}

void RuntimeSessionSupervisor::StepSupervisor()
{
    if (!m_runningFlag.load()) {
        std::lock_guard<std::mutex> lock(m_mu);
        m_sessionStop.store(true);
        m_stopping = true;
        m_modeManager.RequestMode(ControllerMode::Idle);
    }

    ControllerMode startMode = ControllerMode::Idle;
    UnifiedConfig cfg{};
    bool startSession = false;
    bool needJoin = false;
    {
        std::lock_guard<std::mutex> lock(m_mu);
        if (m_sessionDone) {
            needJoin = true;
        }
        if (m_stopping) {
            m_sessionStop.store(true);
        }
        if (m_modeManager.ShouldStopActiveSession()) {
            m_sessionStop.store(true);
            needJoin = true;
        }
    }
    if (needJoin) {
        JoinSession();
    }
    {
        std::lock_guard<std::mutex> lock(m_mu);
        if (needJoin) {
            m_sessionDone = false;
            m_modeManager.MarkSessionJoined();
            m_cv.notify_all();
        }
        if (m_stopping) {
            return;
        }
        if (m_modeManager.ActiveMode() != ControllerMode::Idle &&
            m_modeManager.DesiredMode() == m_modeManager.ActiveMode() && !m_modeManager.RestartRequested()) {
            return;
        }
        if (m_modeManager.DesiredMode() != ControllerMode::Idle) {
            cfg = m_currentConfig();
            startMode = m_modeManager.DesiredMode();
            m_modeManager.MarkSessionLaunching(startMode);
            m_sessionStop.store(false);
            startSession = true;
        } else {
            m_modeManager.MarkSessionJoined();
        }
    }
    if (startSession) {
        m_session = smartdrone::common::StartThread(
            smartdrone::common::MakeThreadLaunchInfo(smartdrone::common::ThreadRole::RuntimeSession,
                                                     "RuntimeSessionSupervisor"),
            [this, cfg, startMode]() mutable {
                bool ok = false;
                if (startMode == ControllerMode::Slam && m_slamSessionRunner) {
                    ok = m_slamSessionRunner(cfg, m_tuning, m_mav, m_sessionStop, m_livePose);
                } else if (startMode == ControllerMode::Calib && m_calibSessionRunner) {
                    ok = m_calibSessionRunner(cfg, m_sessionStop, m_livePose);
                }
                (void)ok;
                std::lock_guard<std::mutex> lock(m_mu);
                m_sessionDone = true;
                m_cv.notify_all();
            });
    }
}

} // namespace smartdrone::core::application
