#include "core/application/runtime/epg_graph_lifecycle.h"

#include <chrono>
#include <utility>

#include "common/epg/epg.h"

namespace SmartDrone::Core::Application {
namespace {

constexpr auto SYNCHRONOUS_STOP_TIMEOUT = std::chrono::seconds(5);

} // namespace

EpgGraphLifecycle::EpgGraphLifecycle(EpgGraphLifecycleConfig config)
    : m_stop(config.stop),
      m_resourcesStopped(std::move(config.resourcesStopped)),
      m_stopResources(std::move(config.stopResources)),
      m_resetResources(std::move(config.resetResources))
{
}

EpgGraphLifecycle::~EpgGraphLifecycle() = default;

bool EpgGraphLifecycle::HasGraph() const
{
    return static_cast<bool>(m_graph);
}

bool EpgGraphLifecycle::StopRequested() const
{
    return m_stopRequested;
}

bool EpgGraphLifecycle::Done() const
{
    return m_done;
}

void EpgGraphLifecycle::ResetForStart()
{
    m_stop.store(false, std::memory_order_relaxed);
    m_done = false;
    m_stopRequested = false;
    m_graphStopRequested = false;
}

void EpgGraphLifecycle::AttachGraph(std::unique_ptr<Epg::EventPipelineGraph> graph)
{
    m_graph = std::move(graph);
}

void EpgGraphLifecycle::RequestStop()
{
    if (m_stopRequested || !m_graph) {
        return;
    }
    m_stopRequested = true;
    m_stop.store(true, std::memory_order_relaxed);
}

void EpgGraphLifecycle::StepStop()
{
    if (!m_stopRequested) {
        return;
    }
    RequestGraphStopIfReady();
    FinishStopIfReady();
}

void EpgGraphLifecycle::StopSynchronously()
{
    if (!m_graph) {
        return;
    }
    RequestStop();
    if (StepSynchronousStopUntilReady()) {
        return;
    }
    if (m_stopResources) {
        m_stopResources();
    }
    m_graph->Stop();
    ResetGraphAndResources();
}

bool EpgGraphLifecycle::StepSynchronousStopUntilReady()
{
    const auto deadline =
        std::chrono::steady_clock::now() + SYNCHRONOUS_STOP_TIMEOUT;
    while (m_graph && std::chrono::steady_clock::now() < deadline) {
        StepStop();
        if (m_done) {
            return true;
        }
    }
    return false;
}

void EpgGraphLifecycle::RequestGraphStopIfReady()
{
    if (!m_graph || m_graphStopRequested || !m_resourcesStopped()) {
        return;
    }
    m_graphStopRequested = true;
    m_graph->RequestStop();
}

void EpgGraphLifecycle::FinishStopIfReady()
{
    if (!m_graph || !m_graphStopRequested || !m_graph->JoinStopped()) {
        return;
    }
    ResetGraphAndResources();
}

void EpgGraphLifecycle::ResetGraphAndResources()
{
    m_graph.reset();
    m_resetResources();
    m_done = true;
}

} // namespace SmartDrone::Core::Application
