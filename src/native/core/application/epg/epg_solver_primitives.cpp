#include "core/application/epg/epg_solver_primitives.h"

#include <algorithm>
#include <sstream>
#include <string>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {

std::uint64_t CeilDiv(std::uint64_t numerator, std::uint64_t denominator)
{
    if (denominator == 0) {
        return numerator;
    }
    return (numerator + denominator - 1) / denominator;
}

std::string JsonEscape(const std::string &value)
{
    std::ostringstream out;
    for (const char ch : value) {
        switch (ch) {
        case '\\':
            out << "\\\\";
            break;
        case '"':
            out << "\\\"";
            break;
        case '\n':
            out << "\\n";
            break;
        case '\r':
            out << "\\r";
            break;
        case '\t':
            out << "\\t";
            break;
        default:
            out << ch;
            break;
        }
    }
    return out.str();
}

std::string JoinReasonTags(const std::vector<std::string> &reasons)
{
    if (reasons.empty()) {
        return {};
    }
    std::string reason = reasons.front();
    for (std::size_t index = 1; index < reasons.size(); ++index) {
        reason += "+" + reasons[index];
    }
    return reason;
}

SolverLimits DefaultSolverLimits()
{
    return {};
}

Epg::SolverReportDecision *FindSolverReportTaskDecision(
    std::vector<Epg::SolverReportDecision> &decisions,
    const std::string &taskName)
{
    for (auto &decision : decisions) {
        if (decision.kind == "task" && decision.name == taskName) {
            return &decision;
        }
    }
    return nullptr;
}

std::uint64_t EffectiveLoopUs(const Epg::TaskProfileMetrics &stats)
{
    return std::max({stats.p99LoopUs, stats.p90LoopUs, stats.maxLoopUs,
                     stats.averageLoopUs});
}

bool HasResourceWaitPressure(const Epg::TaskProfileMetrics &stats,
                             std::uint64_t pressureThresholdUs)
{
    return stats.maxResourceWaitUs > pressureThresholdUs ||
           stats.averageResourceWaitUs > pressureThresholdUs ||
           stats.totalResourceWaitUs > pressureThresholdUs;
}

bool HasResourceSplitPressure(const Epg::TaskProfileMetrics &stats,
                              std::uint64_t resourceWaitThresholdUs,
                              std::uint64_t targetUtilizationPpm)
{
    return HasResourceWaitPressure(stats, resourceWaitThresholdUs) ||
           stats.deadlineMissCount > 0 || stats.budgetOverrunCount > 0 ||
           stats.schedulingErrorCount > 0 ||
           stats.utilizationPpm > targetUtilizationPpm;
}

std::uint64_t QueuePressureAtDepth(
    std::uint64_t depth,
    const Epg::QueueProfileMetrics &stats)
{
    const auto requiredDepth =
        stats.maxDepthObserved + stats.droppedNewest +
        stats.overwrittenOldest;
    return requiredDepth > depth ? requiredDepth - depth : 0;
}

std::uint64_t QueuePressure(const Epg::QueueConfig &queue,
                            const Epg::QueueProfileMetrics &stats)
{
    return QueuePressureAtDepth(static_cast<std::uint64_t>(queue.depth),
                                stats);
}

std::uint64_t QueueCandidatePenalty(
    std::uint64_t depth,
    const Epg::QueueProfileMetrics &stats)
{
    return QueuePressureAtDepth(depth, stats) * 1000 + depth;
}

std::uint64_t TaskPeriodicOverloadUs(std::uint64_t intervalAfterMs,
                                     std::uint64_t effectiveLoopUs)
{
    const std::uint64_t intervalUs = intervalAfterMs * 1000;
    if (intervalUs == 0 || effectiveLoopUs <= intervalUs) {
        return 0;
    }
    return effectiveLoopUs - intervalUs;
}

std::uint64_t TaskUtilizationOverPpm(std::uint64_t intervalBeforeMs,
                                     std::uint64_t intervalAfterMs,
                                     std::uint64_t utilizationPpm,
                                     std::uint64_t targetUtilizationPpm)
{
    if (intervalAfterMs == 0 || utilizationPpm <= targetUtilizationPpm) {
        return 0;
    }
    const auto scaledUtilization =
        CeilDiv(utilizationPpm * intervalBeforeMs, intervalAfterMs);
    if (scaledUtilization <= targetUtilizationPpm) {
        return 0;
    }
    return scaledUtilization - targetUtilizationPpm;
}

std::uint64_t TaskFeasibleIntervalLimit(
    std::uint64_t intervalBeforeMs,
    const Epg::TaskProfileMetrics &stats,
    std::uint64_t effectiveLoopUs,
    std::uint64_t targetUtilizationPpm,
    std::uint64_t maxPeriodicIntervalMs)
{
    std::uint64_t limit = intervalBeforeMs;
    if (intervalBeforeMs > 0 && effectiveLoopUs > intervalBeforeMs * 1000) {
        limit = std::max(limit, CeilDiv(effectiveLoopUs, 1000));
    }
    if (intervalBeforeMs > 0 && stats.utilizationPpm > targetUtilizationPpm) {
        limit = std::max(
            limit,
            CeilDiv(intervalBeforeMs * stats.utilizationPpm,
                    targetUtilizationPpm));
    }
    return std::min(std::max(limit, intervalBeforeMs),
                    maxPeriodicIntervalMs);
}

std::uint64_t PredictedResourceWaitUs(
    std::uint64_t totalResourceWaitUs,
    const std::string &resourceBefore,
    const std::string &resourceAfter)
{
    return resourceAfter == resourceBefore ? totalResourceWaitUs : 0;
}

std::uint64_t ResourceTopologyPenalty(
    const std::string &resourceBefore,
    const std::string &resourceAfter)
{
    return resourceAfter == resourceBefore ? 0 : 20;
}

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
