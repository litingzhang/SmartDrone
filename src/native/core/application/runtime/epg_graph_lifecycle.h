#pragma once

#include <atomic>
#include <functional>
#include <memory>

namespace Epg {
class EventPipelineGraph;
}

namespace SmartDrone::Core::Application {

struct EpgGraphLifecycleConfig {
    std::atomic<bool> &stop;
    std::function<bool()> resourcesStopped;
    std::function<void()> stopResources;
    std::function<void()> resetResources;
};

class EpgGraphLifecycle final {
  public:
    explicit EpgGraphLifecycle(EpgGraphLifecycleConfig config);
    ~EpgGraphLifecycle();

    bool HasGraph() const;
    bool StopRequested() const;
    bool Done() const;
    void ResetForStart();
    void AttachGraph(std::unique_ptr<Epg::EventPipelineGraph> graph);
    void RequestStop();
    void StepStop();
    void StopSynchronously();

  private:
    bool StepSynchronousStopUntilReady();
    void RequestGraphStopIfReady();
    void FinishStopIfReady();
    void ResetGraphAndResources();

    std::atomic<bool> &m_stop;
    std::function<bool()> m_resourcesStopped;
    std::function<void()> m_stopResources;
    std::function<void()> m_resetResources;
    std::unique_ptr<Epg::EventPipelineGraph> m_graph;
    bool m_done{false};
    bool m_stopRequested{false};
    bool m_graphStopRequested{false};
};

} // namespace SmartDrone::Core::Application
