#include "core/application/runtime/runtime_session_supervisor.h"

#include <utility>

#include "core/application/session/epg/session_graph_runtime.h"

namespace SmartDrone::Core::Application {

RuntimeSessionSupervisor::RuntimeSessionSupervisor(Config config)
    : m_runningFlag(config.runningFlag), m_currentConfig(std::move(config.currentConfig)),
      m_createSession(std::move(config.createSession))
{
}

RuntimeSessionSupervisor::~RuntimeSessionSupervisor()
{
    Stop();
}

void RuntimeSessionSupervisor::Stop()
{
    {
        std::lock_guard<std::mutex> lock(m_mu);
        m_stopping = true;
        m_modeManager.RequestMode(ControllerMode::Idle);
        m_sessionStop.store(true);
    }
    std::lock_guard<std::mutex> stepLock(m_stepMu);
    StopActiveSessionSynchronously();
}

void RuntimeSessionSupervisor::OnGraphTick()
{
    StepSupervisor();
}

void RuntimeSessionSupervisor::Step()
{
    OnGraphTick();
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
    return true;
}

void RuntimeSessionSupervisor::RequestRestart()
{
    std::lock_guard<std::mutex> lock(m_mu);
    m_modeManager.RequestRestart();
    m_sessionStop.store(true);
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

RuntimeSessionSupervisor::IdleStatus RuntimeSessionSupervisor::GetIdleStatus() const
{
    std::lock_guard<std::mutex> lock(m_mu);
    return {SessionIdleUnlocked(), m_stopping};
}

void RuntimeSessionSupervisor::StepSupervisor()
{
    std::lock_guard<std::mutex> stepLock(m_stepMu);
    ApplyGlobalStop();
    StopRequestedSession();
    StepActiveSession();
    FinishCompletedSession();
    LaunchRequestedSession();
}

void RuntimeSessionSupervisor::ApplyGlobalStop()
{
    if (m_runningFlag.load()) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_mu);
    m_sessionStop.store(true);
    m_stopping = true;
    m_modeManager.RequestMode(ControllerMode::Idle);
}

void RuntimeSessionSupervisor::StopRequestedSession()
{
    std::shared_ptr<ISessionGraphRuntime> session;
    {
        std::lock_guard<std::mutex> lock(m_mu);
        if (m_stopping || m_modeManager.ShouldStopActiveSession()) {
            m_sessionStop.store(true);
            session = m_session;
        }
    }
    if (session) {
        session->RequestStop();
    }
}

void RuntimeSessionSupervisor::StepActiveSession()
{
    std::shared_ptr<ISessionGraphRuntime> session;
    {
        std::lock_guard<std::mutex> lock(m_mu);
        session = m_session;
    }
    if (session) {
        session->Step();
    }
}

void RuntimeSessionSupervisor::FinishCompletedSession()
{
    std::shared_ptr<ISessionGraphRuntime> session;
    {
        std::lock_guard<std::mutex> lock(m_mu);
        session = m_session;
    }
    if (!session || !session->Done()) {
        return;
    }
    MarkSessionJoined();
}

void RuntimeSessionSupervisor::LaunchRequestedSession()
{
    ControllerMode mode = ControllerMode::Idle;
    UnifiedConfig cfg{};
    if (!PrepareLaunch(mode, cfg)) {
        return;
    }
    auto session = MakeSessionRuntime(mode, cfg);
    if (!session || !session->Start()) {
        MarkSessionJoined();
        return;
    }
    std::lock_guard<std::mutex> lock(m_mu);
    m_session = std::move(session);
}

bool RuntimeSessionSupervisor::PrepareLaunch(ControllerMode &mode, UnifiedConfig &cfg)
{
    std::lock_guard<std::mutex> lock(m_mu);
    if (m_stopping || m_session || m_modeManager.ActiveMode() != ControllerMode::Idle) {
        return false;
    }
    if (m_modeManager.DesiredMode() == ControllerMode::Idle) {
        m_modeManager.MarkSessionJoined();
        return false;
    }
    cfg = m_currentConfig();
    mode = m_modeManager.DesiredMode();
    m_modeManager.MarkSessionLaunching(mode);
    m_sessionStop.store(false);
    return true;
}

std::shared_ptr<ISessionGraphRuntime> RuntimeSessionSupervisor::MakeSessionRuntime(
    ControllerMode mode, const UnifiedConfig &cfg)
{
    if (!m_createSession) {
        return nullptr;
    }
    auto runtime = m_createSession(SessionStartRequest{mode, cfg, m_sessionStop, m_runningFlag});
    if (!runtime) {
        return nullptr;
    }
    return std::shared_ptr<ISessionGraphRuntime>(std::move(runtime));
}

void RuntimeSessionSupervisor::StopActiveSessionSynchronously()
{
    std::shared_ptr<ISessionGraphRuntime> session;
    {
        std::lock_guard<std::mutex> lock(m_mu);
        session = std::move(m_session);
    }
    if (session) {
        session->Stop();
    }
    MarkSessionJoined();
}

void RuntimeSessionSupervisor::MarkSessionJoined()
{
    std::lock_guard<std::mutex> lock(m_mu);
    m_session.reset();
    m_modeManager.MarkSessionJoined();
}

bool RuntimeSessionSupervisor::SessionIdleUnlocked() const
{
    return m_modeManager.ActiveMode() == ControllerMode::Idle &&
           m_modeManager.DesiredMode() == ControllerMode::Idle && !m_session;
}

} // namespace SmartDrone::Core::Application
