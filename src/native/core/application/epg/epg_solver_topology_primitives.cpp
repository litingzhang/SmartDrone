#include "core/application/epg/epg_solver_primitives.h"
#include "core/application/epg/epg_solver_topology_primitives.h"

#include <algorithm>
#include <utility>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {
namespace {

const Epg::QueueProfileMetrics *FindQueueStats(
    const Epg::GraphProfileDiagnostics *diagnostics,
    const std::string &name)
{
    if (!diagnostics) {
        return nullptr;
    }
    const auto it = diagnostics->queues.find(name);
    if (it == diagnostics->queues.end()) {
        return nullptr;
    }
    return &it->second;
}

const Epg::QueueProfileMetrics *FindQueueStats(
    const Epg::GraphProfileDiagnostics *diagnostics,
    const Epg::QueueConfig &queue)
{
    return FindQueueStats(diagnostics, queue.name);
}

bool QueueHasPressure(const Epg::QueueConfig &queue,
                      const Epg::GraphProfileDiagnostics *diagnostics)
{
    const auto *stats = FindQueueStats(diagnostics, queue);
    return stats && QueuePressure(queue, *stats) > 0;
}

std::uint64_t BackpressureChangeCost(
    const std::vector<Epg::PortId> &before,
    const std::vector<Epg::PortId> &after,
    std::uint64_t totalResourceWaitUs)
{
    std::uint64_t penalty = after.size();
    for (const auto port : after) {
        if (!ContainsPort(before, port)) {
            penalty += 10;
        }
    }
    if (totalResourceWaitUs == 0 && after.size() > before.size()) {
        penalty += 100;
    }
    return penalty;
}

} // namespace

std::vector<Epg::PortId> SortedUniquePorts(std::vector<Epg::PortId> ports)
{
    std::sort(ports.begin(), ports.end());
    ports.erase(std::unique(ports.begin(), ports.end()), ports.end());
    return ports;
}

bool ContainsPort(const std::vector<Epg::PortId> &ports, Epg::PortId port)
{
    return std::find(ports.begin(), ports.end(), port) != ports.end();
}

const Epg::QueueConfig *FindQueueConfig(const Epg::GraphConfig &graphConfig,
                                        const std::string &name)
{
    for (const auto &queue : graphConfig.queues) {
        if (queue.name == name) {
            return &queue;
        }
    }
    return nullptr;
}

const Epg::TaskConfig *FindTaskConfig(const Epg::GraphConfig &graphConfig,
                                      const std::string &name)
{
    for (const auto &task : graphConfig.tasks) {
        if (task.name == name) {
            return &task;
        }
    }
    return nullptr;
}

std::vector<Epg::PortId> CandidateBackpressurePorts(
    const Epg::GraphConfig &config,
    const Epg::GraphProfileDiagnostics *diagnostics,
    const Epg::TaskConfig &task,
    const std::vector<Epg::PortId> &before,
    bool replaceable)
{
    if (!replaceable) {
        return SortedUniquePorts(before);
    }
    auto ports = before;
    for (const auto &output : task.outputs) {
        const auto *queue = FindQueueConfig(config, output.second);
        if (!queue || !QueueHasPressure(*queue, diagnostics)) {
            continue;
        }
        ports.push_back(output.first);
    }
    return SortedUniquePorts(std::move(ports));
}

std::uint64_t BackpressureTopologyPenalty(
    const BackpressureTopologyPenaltyInput &input)
{
    const auto &before = *input.before;
    const auto &after = *input.after;
    std::uint64_t penalty = BackpressureChangeCost(
        before, after, input.totalResourceWaitUs);
    if (!input.replaceable || !input.sourceGraphConfig) {
        return penalty;
    }
    for (const auto &output : input.task->outputs) {
        if (ContainsPort(after, output.first)) {
            continue;
        }
        const auto *queue = FindQueueConfig(*input.sourceGraphConfig,
                                            output.second);
        if (!queue) {
            continue;
        }
        const auto *queueStats = FindQueueStats(input.diagnostics, *queue);
        if (!queueStats) {
            continue;
        }
        penalty += QueuePressure(*queue, *queueStats) * 1000;
    }
    return penalty;
}

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
