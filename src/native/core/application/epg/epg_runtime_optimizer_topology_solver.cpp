#include "core/application/epg/epg_runtime_optimizer_candidates.h"

#include "core/application/epg/epg_solver_primitives.h"

#include <algorithm>
#include <utility>

namespace SmartDrone::Core::Application::EpgOptimizerPlanInternal {
namespace {

namespace Solver = EpgSolverPrimitives;

const auto SOLVER_LIMITS = Solver::DefaultSolverLimits();

std::size_t BestQueueCandidateIndex(const QueueSolverNode &node)
{
    std::size_t bestIndex = 0;
    for (std::size_t index = 1; index < node.candidates.size(); ++index) {
        if (node.candidates[index].penalty <
            node.candidates[bestIndex].penalty) {
            bestIndex = index;
        }
    }
    return bestIndex;
}

bool TaskCandidateIsBetter(const TaskCandidate &candidate,
                           const TaskCandidate &best)
{
    if (candidate.penalty != best.penalty) {
        return candidate.penalty < best.penalty;
    }
    if (candidate.predictedResourceWaitUs != best.predictedResourceWaitUs) {
        return candidate.predictedResourceWaitUs < best.predictedResourceWaitUs;
    }
    if (candidate.topologyPenalty != best.topologyPenalty) {
        return candidate.topologyPenalty < best.topologyPenalty;
    }
    return candidate.backpressureOutputs.size() >
           best.backpressureOutputs.size();
}

std::size_t BestTaskCandidateIndex(const TaskSolverNode &node)
{
    std::size_t bestIndex = 0;
    for (std::size_t index = 1; index < node.candidates.size(); ++index) {
        if (TaskCandidateIsBetter(node.candidates[index],
                                  node.candidates[bestIndex])) {
            bestIndex = index;
        }
    }
    return bestIndex;
}

TopologySolution SolveIndependentTopology(
    const std::vector<QueueSolverNode> &queues,
    const std::vector<TaskSolverNode> &tasks)
{
    TopologySolution solution;
    solution.queueCandidateIndexes.reserve(queues.size());
    solution.taskCandidateIndexes.reserve(tasks.size());
    for (const auto &queue : queues) {
        solution.queueCandidateIndexes.push_back(BestQueueCandidateIndex(queue));
    }
    for (const auto &task : tasks) {
        solution.taskCandidateIndexes.push_back(BestTaskCandidateIndex(task));
    }
    return solution;
}

Solver::DiscreteCandidateSet BuildQueueCandidateSet(
    const QueueSolverNode &queue)
{
    Solver::DiscreteCandidateSet set;
    for (const auto &candidate : queue.candidates) {
        set.penalties.push_back(candidate.penalty);
        set.tieWeights.push_back(0);
    }
    return set;
}

std::uint64_t TaskCandidateTieWeight(const TaskCandidate &candidate)
{
    std::uint64_t tieWeight = candidate.backpressureOutputs.size();
    if (candidate.predictedResourceWaitUs == 0) {
        ++tieWeight;
    }
    return tieWeight +
           1000 - std::min<std::uint64_t>(1000, candidate.topologyPenalty);
}

Solver::DiscreteCandidateSet BuildTaskCandidateSet(
    const TaskSolverNode &task)
{
    Solver::DiscreteCandidateSet set;
    for (const auto &candidate : task.candidates) {
        set.penalties.push_back(candidate.penalty);
        set.tieWeights.push_back(TaskCandidateTieWeight(candidate));
    }
    return set;
}

std::vector<Solver::DiscreteCandidateSet> BuildGlobalCandidateSets(
    const std::vector<QueueSolverNode> &queues,
    const std::vector<TaskSolverNode> &tasks)
{
    std::vector<Solver::DiscreteCandidateSet> sets;
    sets.reserve(queues.size() + tasks.size());
    for (const auto &queue : queues) {
        sets.push_back(BuildQueueCandidateSet(queue));
    }
    for (const auto &task : tasks) {
        sets.push_back(BuildTaskCandidateSet(task));
    }
    return sets;
}

TopologySolution BuildTopologySolutionFromGlobalIndexes(
    const std::vector<std::size_t> &candidateIndexes,
    std::size_t queueCount,
    std::size_t taskCount)
{
    TopologySolution solution;
    solution.queueCandidateIndexes.assign(queueCount, 0);
    solution.taskCandidateIndexes.assign(taskCount, 0);
    for (std::size_t index = 0; index < candidateIndexes.size(); ++index) {
        if (index < queueCount) {
            solution.queueCandidateIndexes[index] = candidateIndexes[index];
            continue;
        }
        solution.taskCandidateIndexes[index - queueCount] =
            candidateIndexes[index];
    }
    return solution;
}

} // namespace

TopologySolution SolveGlobalTopology(
    const std::vector<QueueSolverNode> &queues,
    const std::vector<TaskSolverNode> &tasks)
{
    const auto candidateSets = BuildGlobalCandidateSets(queues, tasks);
    const auto globalSolution = Solver::SolveDiscreteGlobalTopology(
        candidateSets, SOLVER_LIMITS.maxExactTopologyCandidates);
    if (!globalSolution.exact) {
        return SolveIndependentTopology(queues, tasks);
    }
    return BuildTopologySolutionFromGlobalIndexes(
        globalSolution.candidateIndexes, queues.size(), tasks.size());
}

} // namespace SmartDrone::Core::Application::EpgOptimizerPlanInternal
