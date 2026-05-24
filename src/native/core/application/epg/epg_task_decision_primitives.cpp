#include "core/application/epg/epg_solver_primitives.h"
#include "core/application/epg/epg_solver_task_evaluation_primitives.h"

#include <string>
#include <vector>

namespace SmartDrone::Core::Application::EpgSolverPrimitives {
namespace {

std::vector<std::string> TaskChangeReasons(
    const TaskDecisionReasonInput &input)
{
    std::vector<std::string> reasons;
    if (input.intervalAfterMs != input.intervalBeforeMs) {
        reasons.push_back("global_optimum_interval");
    }
    if (*input.backpressureAfter != *input.backpressureBefore) {
        reasons.push_back("global_optimum_backpressure");
    }
    if (*input.resourceAfter != *input.resourceBefore) {
        reasons.push_back("global_optimum_resource");
    }
    const auto cpuAffinity =
        input.cpuBindingAffinity >= 0 ? input.cpuBindingAffinity
                                      : input.cpuAffinityAfter;
    if (cpuAffinity != input.cpuAffinityBefore) {
        reasons.push_back("global_optimum_cpu_binding");
    }
    return reasons;
}

std::vector<std::string> TaskPressureReasons(
    const TaskDecisionReasonInput &input)
{
    const auto &stats = *input.stats;
    std::vector<std::string> reasons;
    if (stats.utilizationPpm > input.targetUtilizationPpm) {
        reasons.push_back("utilization_over_target");
    }
    if (stats.budgetOverrunCount > 0 ||
        (input.budgetUs > 0 && input.effectiveLoopUs > input.budgetUs)) {
        reasons.push_back("budget_overrun");
    }
    if (stats.deadlineMissCount > 0 ||
        (input.deadlineUs > 0 && input.effectiveLoopUs > input.deadlineUs)) {
        reasons.push_back("deadline_miss");
    }
    if (stats.schedulingErrorCount > 0) {
        reasons.push_back("scheduling_error");
    }
    if (HasResourceWaitPressure(stats, input.resourceWaitPressureUs)) {
        reasons.push_back("resource_wait");
    }
    return reasons;
}

std::string JoinTaskDecisionReasons(
    const std::vector<std::string> &reasons)
{
    if (reasons.empty()) {
        return "keep";
    }
    return JoinReasonTags(reasons);
}

} // namespace

Epg::TaskProfileMetrics TaskMetricsFromDecision(
    const Epg::SolverReportDecision &decision)
{
    Epg::TaskProfileMetrics stats;
    stats.maxLoopUs = decision.maxLoopUs;
    stats.averageLoopUs = decision.averageLoopUs;
    stats.p90LoopUs = decision.p90LoopUs;
    stats.p99LoopUs = decision.p99LoopUs;
    stats.resourceWaitCount = decision.resourceWaitCount;
    stats.maxResourceWaitUs = decision.maxResourceWaitUs;
    stats.averageResourceWaitUs = decision.averageResourceWaitUs;
    stats.totalResourceWaitUs = decision.totalResourceWaitUs;
    stats.utilizationPpm = decision.utilizationPpm;
    stats.budgetOverrunCount = decision.budgetOverrunCount;
    stats.deadlineMissCount = decision.deadlineMissCount;
    stats.schedulingErrorCount = decision.schedulingErrorCount;
    return stats;
}

std::string BuildTaskDecisionReason(
    const TaskDecisionReasonInput &input)
{
    const auto changeReason = JoinTaskDecisionReasons(TaskChangeReasons(input));
    if (changeReason != "keep") {
        return changeReason;
    }
    const auto pressureReason =
        JoinTaskDecisionReasons(TaskPressureReasons(input));
    if (input.replaceable) {
        return pressureReason;
    }
    if (pressureReason == "keep") {
        return "not_replaceable";
    }
    return "not_replaceable+" + pressureReason;
}

} // namespace SmartDrone::Core::Application::EpgSolverPrimitives
