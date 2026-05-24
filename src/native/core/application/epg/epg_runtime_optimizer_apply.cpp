#include "core/application/epg/epg_runtime_optimizer_apply.h"

#include <utility>

namespace SmartDrone::Core::Application::EpgOptimizerPlanInternal {

void ApplyQueueSolution(Epg::QueueConfig &queue,
                        const QueueSolverNode &node,
                        const QueueCandidate &candidate,
                        std::vector<SolverDecision> &decisions)
{
    queue.depth = static_cast<std::size_t>(candidate.depth);
    SolverDecision decision;
    decision.kind = "queue";
    decision.name = queue.name;
    decision.depthBefore = node.depthBefore;
    decision.depthAfter = candidate.depth;
    decision.pressureBefore = node.pressureBefore;
    decision.pressureAfter = candidate.pressureAfter;
    decision.maxDepthObserved = node.stats.maxDepthObserved;
    decision.droppedNewest = node.stats.droppedNewest;
    decision.overwrittenOldest = node.stats.overwrittenOldest;
    decision.pushedPerSecond = node.stats.pushedPerSecond;
    decision.poppedPerSecond = node.stats.poppedPerSecond;
    decision.droppedPerSecond = node.stats.droppedPerSecond;
    decision.reason =
        candidate.depth != node.depthBefore ? "global_optimum_depth" : "keep";
    decisions.push_back(std::move(decision));
}

bool HasExecutionPlanChange(const std::vector<SolverDecision> &decisions)
{
    for (const auto &decision : decisions) {
        const bool queueChanged = decision.kind == "queue" &&
                                  decision.depthBefore != decision.depthAfter;
        const bool taskChanged =
            decision.kind == "task" &&
            (decision.intervalBeforeMs != decision.intervalAfterMs ||
             decision.resourceBefore != decision.resourceAfter ||
             decision.cpuAffinityBefore != decision.cpuAffinityAfter ||
             decision.backpressureBefore != decision.backpressureAfter);
        if (queueChanged || taskChanged) {
            return true;
        }
    }
    return false;
}

} // namespace SmartDrone::Core::Application::EpgOptimizerPlanInternal
