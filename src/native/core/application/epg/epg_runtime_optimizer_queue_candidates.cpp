#include "core/application/epg/epg_runtime_optimizer_candidates.h"

#include "core/application/epg/epg_solver_primitives.h"

#include <algorithm>

namespace SmartDrone::Core::Application::EpgOptimizerPlanInternal {
namespace {

namespace Solver = EpgSolverPrimitives;

QueueCandidate MakeQueueCandidate(std::uint64_t depth,
                                  const Epg::QueueProfileMetrics &stats)
{
    return {
        depth,
        Solver::QueuePressureAtDepth(depth, stats),
        Solver::QueueCandidatePenalty(depth, stats),
    };
}

std::vector<QueueCandidate> BuildQueueCandidates(
    const Epg::QueueConfig &queue,
    const Epg::QueueProfileMetrics &stats)
{
    const auto minDepth = std::max<std::uint64_t>(1, queue.depth);
    return {MakeQueueCandidate(minDepth, stats)};
}

QueueSolverNode BuildQueueSolverNode(
    std::size_t index,
    const Epg::QueueConfig &queue,
    const Epg::QueueProfileMetrics &stats)
{
    QueueSolverNode node;
    node.index = index;
    node.depthBefore = queue.depth;
    node.pressureBefore = Solver::QueuePressure(queue, stats);
    node.stats = stats;
    node.candidates = BuildQueueCandidates(queue, stats);
    return node;
}

} // namespace

std::vector<QueueSolverNode> BuildQueueSolverNodes(
    const Epg::GraphConfig &config,
    const Epg::GraphProfileDiagnostics &diagnostics)
{
    std::vector<QueueSolverNode> nodes;
    nodes.reserve(config.queues.size());
    for (std::size_t index = 0; index < config.queues.size(); ++index) {
        const auto &queue = config.queues[index];
        nodes.push_back(
            BuildQueueSolverNode(index, queue,
                                 diagnostics.queues.at(queue.name)));
    }
    return nodes;
}

} // namespace SmartDrone::Core::Application::EpgOptimizerPlanInternal
