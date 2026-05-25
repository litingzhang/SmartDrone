#include "core/application/runtime/runtime_session_supervisor.h"

#include <utility>

#include "core/application/session/epg/session_graph_runtime.h"

namespace SmartDrone::Core::Application {

namespace {

class AtomicFlagResetGuard {
  public:
    explicit AtomicFlagResetGuard(std::atomic<bool> &flag)
        : m_flag(flag)
    {
    }

    ~AtomicFlagResetGuard()
    {
        m_flag.store(false, std::memory_order_release);
    }

  private:
    std::atomic<bool> &m_flag;
};

} // namespace

RuntimeSessionSupervisor::RuntimeSessionSupervisor(Config config)
    : m_runningFlag(config.runningFlag), m_currentConfig(std::move(config.currentConfig)),
      m_createSession(std::move(config.createSession))
{
    StoreState(std::make_shared<const SupervisorState>());
}

RuntimeSessionSupervisor::~RuntimeSessionSupervisor()
{
    Stop();
}

void RuntimeSessionSupervisor::Stop()
{
    RequestSupervisorStop();
    StopActiveSessionSynchronously();
}

void RuntimeSessionSupervisor::RequestSupervisorStop()
{
    std::shared_ptr<const SupervisorState> current = LoadState();
    while (current) {
        auto next = std::make_shared<SupervisorState>(*current);
        next->stopping = true;
        next->modeManager.RequestMode(ControllerMode::Idle);
        if (ReplaceState(current, std::move(next))) {
            m_sessionStop.store(true);
            break;
        }
    }
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
    std::shared_ptr<const SupervisorState> current = LoadState();
    while (current) {
        if (current->stopping) {
            if (err) {
                *err = "runtime stopping";
            }
            return false;
        }
        auto next = std::make_shared<SupervisorState>(*current);
        next->modeManager.RequestMode(mode);
        if (ReplaceState(current, std::move(next))) {
            return true;
        }
    }
    return false;
}

void RuntimeSessionSupervisor::RequestRestart()
{
    std::shared_ptr<const SupervisorState> current = LoadState();
    while (current) {
        auto next = std::make_shared<SupervisorState>(*current);
        next->modeManager.RequestRestart();
        if (ReplaceState(current, std::move(next))) {
            m_sessionStop.store(true);
            return;
        }
    }
}

RuntimeSessionSupervisor::ControllerMode RuntimeSessionSupervisor::DesiredMode() const
{
    return LoadState()->modeManager.DesiredMode();
}

RuntimeSessionSupervisor::ControllerMode RuntimeSessionSupervisor::ActiveMode() const
{
    return LoadState()->modeManager.ActiveMode();
}

RuntimeSessionSupervisor::IdleStatus RuntimeSessionSupervisor::GetIdleStatus() const
{
    std::shared_ptr<const SupervisorState> state = LoadState();
    return {SessionIdle(*state), state->stopping};
}

void RuntimeSessionSupervisor::StepSupervisor()
{
    bool expected = false;
    if (!m_stepRunning.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return;
    }
    AtomicFlagResetGuard stepGuard(m_stepRunning);
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
    RequestSupervisorStop();
}

void RuntimeSessionSupervisor::StopRequestedSession()
{
    std::shared_ptr<const SupervisorState> state = LoadState();
    if (!state->stopping && !state->modeManager.ShouldStopActiveSession()) {
        return;
    }
    m_sessionStop.store(true);
    std::shared_ptr<ISessionGraphRuntime> session = state->session;
    if (session) {
        session->RequestStop();
    }
}

void RuntimeSessionSupervisor::StepActiveSession()
{
    std::shared_ptr<ISessionGraphRuntime> session = LoadState()->session;
    if (session) {
        session->Step();
    }
}

void RuntimeSessionSupervisor::FinishCompletedSession()
{
    std::shared_ptr<ISessionGraphRuntime> session = LoadState()->session;
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
    std::shared_ptr<const SupervisorState> current = LoadState();
    while (current) {
        auto next = std::make_shared<SupervisorState>(*current);
        next->session = session;
        if (ReplaceState(current, std::move(next))) {
            return;
        }
    }
}

bool RuntimeSessionSupervisor::PrepareLaunch(ControllerMode &mode, UnifiedConfig &cfg)
{
    std::shared_ptr<const SupervisorState> current = LoadState();
    while (current) {
        if (current->stopping || current->session ||
            current->modeManager.ActiveMode() != ControllerMode::Idle) {
            return false;
        }
        auto next = std::make_shared<SupervisorState>(*current);
        if (current->modeManager.DesiredMode() == ControllerMode::Idle) {
            next->modeManager.MarkSessionJoined();
            if (ReplaceState(current, std::move(next))) {
                return false;
            }
            continue;
        }
        mode = current->modeManager.DesiredMode();
        next->modeManager.MarkSessionLaunching(mode);
        std::shared_ptr<const SupervisorState> nextState = next;
        if (ReplaceState(current, nextState)) {
            m_sessionStop.store(false);
            std::shared_ptr<const SupervisorState> latest = LoadState();
            if (latest != nextState &&
                (latest->stopping || latest->modeManager.ShouldStopActiveSession())) {
                m_sessionStop.store(true);
            }
            cfg = m_currentConfig();
            return true;
        }
    }
    return false;
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
    std::shared_ptr<ISessionGraphRuntime> session = LoadState()->session;
    if (session) {
        session->Stop();
    }
    MarkSessionJoined();
}

void RuntimeSessionSupervisor::MarkSessionJoined()
{
    std::shared_ptr<const SupervisorState> current = LoadState();
    while (current) {
        auto next = std::make_shared<SupervisorState>(*current);
        next->session.reset();
        next->modeManager.MarkSessionJoined();
        if (ReplaceState(current, std::move(next))) {
            return;
        }
    }
}

bool RuntimeSessionSupervisor::SessionIdle(const SupervisorState &state) const
{
    return state.modeManager.ActiveMode() == ControllerMode::Idle &&
           state.modeManager.DesiredMode() == ControllerMode::Idle && !state.session;
}

std::shared_ptr<const RuntimeSessionSupervisor::SupervisorState>
RuntimeSessionSupervisor::LoadState() const
{
    return std::atomic_load_explicit(&m_state, std::memory_order_acquire);
}

void RuntimeSessionSupervisor::StoreState(std::shared_ptr<const SupervisorState> state)
{
    std::atomic_store_explicit(&m_state, std::move(state), std::memory_order_release);
}

bool RuntimeSessionSupervisor::ReplaceState(
    std::shared_ptr<const SupervisorState> &expected,
    std::shared_ptr<const SupervisorState> next)
{
    return std::atomic_compare_exchange_weak_explicit(&m_state, &expected, std::move(next),
                                                      std::memory_order_acq_rel,
                                                      std::memory_order_acquire);
}

} // namespace SmartDrone::Core::Application
