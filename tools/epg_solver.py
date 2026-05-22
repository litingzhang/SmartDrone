#!/usr/bin/env python3
"""Generate an optimized EPG GraphConfig from an EPG profile JSON."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Set, Tuple


PROFILE_SCHEMA = "smartdrone.epg.profile.v1"
OPTIMIZED_SCHEMA = "smartdrone.epg.optimized_config.v1"
SOLVER_REPORT_SCHEMA = "smartdrone.epg.solver_report.v1"
SOLVER_VERSION = "python-exact-v1"
EXACT_SOLVER_OBJECTIVE = "global_minimize_predicted_epg_penalty_discrete_topology"
RESOURCE_WAIT_PRESSURE_US = 1000
RESOURCE_ISOLATION_CPU_AFFINITY = 2
RESOURCE_ISOLATION_PRIORITY = 20


@dataclass(frozen=True)
class SolverLimits:
    max_queue_depth: int
    max_periodic_interval_ms: int
    target_utilization_ppm: int


def integer(value: Any, fallback: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return fallback


def ceil_div(numerator: int, denominator: int) -> int:
    if denominator <= 0:
        return numerator
    return (numerator + denominator - 1) // denominator


def load_profile(path: Path) -> Dict[str, Any]:
    profile = json.loads(path.read_text(encoding="utf-8"))
    if profile.get("schema") != PROFILE_SCHEMA:
        raise ValueError(f"unsupported EPG profile schema: {profile.get('schema')}")
    topology = profile.get("topology")
    if not isinstance(topology, dict):
        raise ValueError("EPG profile missing topology object")
    return profile


def queue_diagnostics(profile: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    diagnostics = profile.get("diagnostics", {})
    queues = diagnostics.get("queues", {})
    if not isinstance(queues, dict):
        return {}
    return {name: value for name, value in queues.items() if isinstance(value, dict)}


def task_diagnostics(profile: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    diagnostics = profile.get("diagnostics", {})
    tasks = diagnostics.get("tasks", {})
    if not isinstance(tasks, dict):
        return {}
    return {name: value for name, value in tasks.items() if isinstance(value, dict)}


def task_catalog(profile: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    catalog = profile.get("taskCatalog", [])
    if not isinstance(catalog, list):
        raise ValueError("EPG profile taskCatalog must be an array")
    result = {}
    for item in catalog:
        task_type = validate_task_catalog_entry(item)
        if task_type in result:
            raise ValueError(
                f"EPG profile taskCatalog duplicates taskType: {task_type}")
        result[task_type] = item
    if not result:
        raise ValueError("EPG profile taskCatalog must not be empty")
    return result


def required_catalog_text(item: Dict[str, Any], field: str) -> str:
    value = str(item.get(field, ""))
    if not value:
        raise ValueError(f"EPG profile taskCatalog entry missing {field}")
    return value


def required_catalog_positive_int(item: Dict[str, Any], field: str) -> int:
    value = integer(item.get(field))
    if value <= 0:
        raise ValueError(f"EPG profile taskCatalog entry invalid {field}")
    return value


def validate_task_catalog_entry(item: Any) -> str:
    if not isinstance(item, dict):
        raise ValueError("EPG profile taskCatalog entries must be objects")
    task_type = required_catalog_text(item, "taskType")
    required_catalog_text(item, "role")
    required_catalog_text(item, "resource")
    budget_us = required_catalog_positive_int(item, "budgetUs")
    deadline_us = required_catalog_positive_int(item, "deadlineUs")
    if deadline_us < budget_us:
        raise ValueError(
            f"EPG profile taskCatalog timing invalid: {task_type}")
    if "replaceable" in item and not isinstance(item["replaceable"], bool):
        raise ValueError(
            f"EPG profile taskCatalog replaceable invalid: {task_type}")
    return task_type


def validate_task_catalog_coverage(tasks: List[Dict[str, Any]],
                                   catalog: Dict[str, Dict[str, Any]]) -> None:
    for task in tasks:
        if not isinstance(task, dict):
            continue
        task_type = str(task.get("type", ""))
        if task_type not in catalog:
            raise ValueError(
                f"EPG profile task type missing catalog metadata: {task_type}")


def require_text(item: Dict[str, Any], field: str, kind: str) -> str:
    value = str(item.get(field, ""))
    if not value:
        raise ValueError(f"EPG profile {kind} missing {field}")
    return value


QUEUE_DIAGNOSTIC_FIELDS = [
    "maxDepthObserved",
    "droppedNewest",
    "overwrittenOldest",
    "pushedPerSecond",
    "poppedPerSecond",
    "droppedPerSecond",
]

TASK_DIAGNOSTIC_FIELDS = [
    "maxLoopUs",
    "averageLoopUs",
    "p90LoopUs",
    "p99LoopUs",
    "utilizationPpm",
    "budgetOverrunCount",
    "deadlineMissCount",
    "schedulingErrorCount",
]

SOLVER_SCORE_FIELDS = [
    "queuePressure",
    "periodicOverloadUs",
    "resourceWaitUs",
    "schedulingErrors",
    "budgetOverruns",
    "deadlineMisses",
    "utilizationOverPpm",
    "totalPenalty",
]

SOLVER_CONSTRAINT_FIELDS = [
    "maxQueueDepth",
    "maxPeriodicIntervalMs",
    "targetUtilizationPpm",
]

QUEUE_DECISION_FIELDS = [
    "depthBefore",
    "depthAfter",
    "pressureBefore",
    "pressureAfter",
    "maxDepthObserved",
    "droppedNewest",
    "overwrittenOldest",
    "pushedPerSecond",
    "poppedPerSecond",
    "droppedPerSecond",
]

TASK_DECISION_FIELDS = [
    "intervalBeforeMs",
    "intervalAfterMs",
    "maxLoopUs",
    "averageLoopUs",
    "p90LoopUs",
    "p99LoopUs",
    "effectiveLoopUs",
    "resourceWaitCount",
    "maxResourceWaitUs",
    "averageResourceWaitUs",
    "totalResourceWaitUs",
    "utilizationPpm",
    "targetUtilizationPpm",
    "budgetUs",
    "deadlineUs",
    "budgetOverrunCount",
    "deadlineMissCount",
    "schedulingErrorCount",
]

TASK_REASON_ORDER = [
    "utilization_over_target",
    "budget_overrun",
    "deadline_miss",
    "scheduling_error",
    "resource_wait",
]


def validate_diagnostic_fields(diag: Dict[str, Any],
                               fields: List[str],
                               name: str,
                               kind: str) -> None:
    for field in fields:
        if field not in diag:
            raise ValueError(
                f"EPG profile diagnostics {kind} missing {field}: {name}")
        if integer(diag.get(field), -1) < 0:
            raise ValueError(
                f"EPG profile diagnostics {kind} invalid {field}: {name}")


def validate_diagnostics_coverage(queues: List[Dict[str, Any]],
                                  tasks: List[Dict[str, Any]],
                                  queue_diag: Dict[str, Dict[str, Any]],
                                  task_diag: Dict[str, Dict[str, Any]]) -> None:
    for queue in queues:
        if not isinstance(queue, dict):
            continue
        name = require_text(queue, "name", "queue")
        if name not in queue_diag:
            raise ValueError(f"EPG profile diagnostics missing queue: {name}")
        validate_diagnostic_fields(
            queue_diag[name], QUEUE_DIAGNOSTIC_FIELDS, name, "queue")
    for task in tasks:
        if not isinstance(task, dict):
            continue
        name = require_text(task, "name", "task")
        if name not in task_diag:
            raise ValueError(f"EPG profile diagnostics missing task: {name}")
        validate_diagnostic_fields(
            task_diag[name], TASK_DIAGNOSTIC_FIELDS, name, "task")


def config_items_by_name(config: Dict[str, Any],
                         field: str,
                         kind: str) -> Dict[str, Dict[str, Any]]:
    result = {}
    for item in config.get(field, []):
        if not isinstance(item, dict):
            continue
        name = require_text(item, "name", kind)
        result[name] = item
    return result


def require_non_negative_fields(item: Dict[str, Any],
                                fields: List[str],
                                kind: str,
                                name: str) -> None:
    for field in fields:
        if integer(item.get(field), -1) < 0:
            raise ValueError(
                f"EPG solver report {kind} decision invalid {field}: {name}")


def validate_queue_decision(item: Dict[str, Any],
                            source_queue: Dict[str, Any],
                            queue: Dict[str, Any],
                            constraints: Dict[str, Any]) -> None:
    name = require_text(item, "name", "solver report decision")
    require_non_negative_fields(item, QUEUE_DECISION_FIELDS, "queue", name)
    depth_after = integer(item.get("depthAfter"))
    expected_reason = (
        "global_optimum_depth"
        if depth_after != integer(item.get("depthBefore"))
        else "keep"
    )
    if depth_after != integer(queue.get("depth")):
        raise ValueError(f"EPG solver report queue depth mismatch: {name}")
    if integer(item.get("depthBefore")) != integer(source_queue.get("depth")):
        raise ValueError(f"EPG solver report queue source mismatch: {name}")
    if depth_after <= 0 or depth_after > integer(constraints.get("maxQueueDepth")):
        raise ValueError(
            f"EPG solver report queue constraint mismatch: {name}")
    if item.get("reason") != expected_reason:
        raise ValueError(f"EPG solver report queue reason mismatch: {name}")


def decision_reason_set(item: Dict[str, Any]) -> Set[str]:
    reasons: Set[str] = set()
    if integer(item.get("utilizationPpm")) > integer(
            item.get("targetUtilizationPpm")):
        reasons.add("utilization_over_target")
    if integer(item.get("budgetOverrunCount")) > 0:
        reasons.add("budget_overrun")
    if integer(item.get("budgetUs")) > 0:
        if integer(item.get("effectiveLoopUs")) > integer(item.get("budgetUs")):
            reasons.add("budget_overrun")
    if integer(item.get("deadlineMissCount")) > 0:
        reasons.add("deadline_miss")
    if integer(item.get("deadlineUs")) > 0:
        if integer(item.get("effectiveLoopUs")) > integer(item.get("deadlineUs")):
            reasons.add("deadline_miss")
    if integer(item.get("schedulingErrorCount")) > 0:
        reasons.add("scheduling_error")
    if (integer(item.get("maxResourceWaitUs")) > RESOURCE_WAIT_PRESSURE_US or
            integer(item.get("averageResourceWaitUs")) >
            RESOURCE_WAIT_PRESSURE_US or
            integer(item.get("totalResourceWaitUs")) >
            RESOURCE_WAIT_PRESSURE_US):
        reasons.add("resource_wait")
    return reasons


def ordered_reason_text(reasons: Set[str]) -> str:
    ordered = [reason for reason in TASK_REASON_ORDER if reason in reasons]
    return "+".join(ordered) if ordered else "keep"


def expected_task_decision_reason(item: Dict[str, Any]) -> str:
    if integer(item.get("intervalAfterMs")) != integer(item.get("intervalBeforeMs")):
        return "global_optimum_interval"
    reason = ordered_reason_text(decision_reason_set(item))
    if item.get("replaceable"):
        return reason
    return "not_replaceable" if reason == "keep" else f"not_replaceable+{reason}"


def validate_task_decision(item: Dict[str, Any],
                           source_task: Dict[str, Any],
                           task: Dict[str, Any],
                           constraints: Dict[str, Any],
                           catalog: Dict[str, Dict[str, Any]]) -> None:
    name = require_text(item, "name", "solver report decision")
    require_non_negative_fields(item, TASK_DECISION_FIELDS, "task", name)
    trigger = task.get("trigger", {})
    scheduling = task.get("scheduling", {})
    if not isinstance(trigger, dict) or not isinstance(scheduling, dict):
        raise ValueError(f"EPG optimized task config invalid: {name}")
    catalog_item = catalog.get(str(task.get("type", "")))
    if not isinstance(catalog_item, dict):
        raise ValueError(f"EPG solver report task catalog missing: {name}")
    if not isinstance(item.get("replaceable"), bool):
        raise ValueError(f"EPG solver report task replaceable invalid: {name}")
    if require_text(item, "catalogRole", "solver report decision") != str(
            catalog_item.get("role", "")):
        raise ValueError(f"EPG solver report task role mismatch: {name}")
    if item.get("replaceable") != bool(catalog_item.get("replaceable", False)):
        raise ValueError(f"EPG solver report task replaceable mismatch: {name}")
    if integer(item.get("intervalAfterMs")) != integer(trigger.get("interval_ms")):
        raise ValueError(f"EPG solver report task interval mismatch: {name}")
    source_trigger = source_task.get("trigger", {})
    if not isinstance(source_trigger, dict):
        raise ValueError(f"EPG profile task trigger invalid: {name}")
    if integer(item.get("intervalBeforeMs")) != integer(
            source_trigger.get("interval_ms")):
        raise ValueError(f"EPG solver report task source mismatch: {name}")
    if integer(item.get("budgetUs")) != integer(scheduling.get("budget_us")):
        raise ValueError(f"EPG solver report task budget mismatch: {name}")
    if integer(item.get("deadlineUs")) != integer(scheduling.get("deadline_us")):
        raise ValueError(f"EPG solver report task deadline mismatch: {name}")
    if integer(item.get("targetUtilizationPpm")) != integer(
            constraints.get("targetUtilizationPpm")):
        raise ValueError(f"EPG solver report task target mismatch: {name}")
    if integer(item.get("intervalAfterMs")) > integer(
            constraints.get("maxPeriodicIntervalMs")):
        raise ValueError(
            f"EPG solver report task interval constraint mismatch: {name}")
    if item.get("reason") != expected_task_decision_reason(item):
        raise ValueError(f"EPG solver report task reason mismatch: {name}")


def queue_pressure(depth: int, diag: Dict[str, Any]) -> int:
    max_depth = integer(diag.get("maxDepthObserved"))
    drops = integer(diag.get("droppedNewest"))
    overwrites = integer(diag.get("overwrittenOldest"))
    return max(max_depth - depth, 0) + drops + overwrites


def queue_candidate_penalty(depth: int, diag: Dict[str, Any]) -> int:
    return queue_pressure(depth, diag) * 1000 + depth


def queue_candidates(queue: Dict[str, Any],
                     limits: SolverLimits,
                     diag: Dict[str, Any]) -> List[Dict[str, int]]:
    depth = max(1, integer(queue.get("depth"), 1))
    max_depth = max(depth, limits.max_queue_depth)
    return [
        {
            "depth": candidate,
            "pressureAfter": queue_pressure(candidate, diag),
            "penalty": queue_candidate_penalty(candidate, diag),
        }
        for candidate in range(depth, max_depth + 1)
    ]


def best_candidate_index(candidates: List[Dict[str, int]]) -> int:
    return min(range(len(candidates)), key=lambda index: candidates[index]["penalty"])


def queue_decision(queue: Dict[str, Any],
                   diag: Dict[str, Any],
                   candidate: Dict[str, int]) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    result = dict(queue)
    name = str(result.get("name", ""))
    depth = integer(result.get("depth"), 1)
    pressure_before = queue_pressure(depth, diag)
    result["depth"] = candidate["depth"]
    decision = {
        "kind": "queue",
        "name": name,
        "depthBefore": depth,
        "depthAfter": result["depth"],
        "pressureBefore": pressure_before,
        "pressureAfter": candidate["pressureAfter"],
        "maxDepthObserved": integer(diag.get("maxDepthObserved")),
        "droppedNewest": integer(diag.get("droppedNewest")),
        "overwrittenOldest": integer(diag.get("overwrittenOldest")),
        "pushedPerSecond": integer(diag.get("pushedPerSecond")),
        "poppedPerSecond": integer(diag.get("poppedPerSecond")),
        "droppedPerSecond": integer(diag.get("droppedPerSecond")),
        "reason": (
            "global_optimum_depth"
            if result["depth"] != depth
            else "keep"
        ),
    }
    return result, decision


def effective_loop_us(diag: Dict[str, Any]) -> int:
    return max(
        integer(diag.get("p99LoopUs")),
        integer(diag.get("p90LoopUs")),
        integer(diag.get("maxLoopUs")),
        integer(diag.get("averageLoopUs")),
    )


def has_resource_wait_pressure(diag: Dict[str, Any]) -> bool:
    return (
        integer(diag.get("maxResourceWaitUs")) > RESOURCE_WAIT_PRESSURE_US or
        integer(diag.get("averageResourceWaitUs")) >
        RESOURCE_WAIT_PRESSURE_US or
        integer(diag.get("totalResourceWaitUs")) >
        RESOURCE_WAIT_PRESSURE_US
    )


def reason_set(diag: Dict[str, Any],
               loop_us: int,
               utilization_ppm: int,
               scheduling: Dict[str, Any],
               limits: SolverLimits) -> Set[str]:
    reasons: Set[str] = set()
    if utilization_ppm > limits.target_utilization_ppm:
        reasons.add("utilization_over_target")
    if integer(diag.get("budgetOverrunCount")) > 0:
        reasons.add("budget_overrun")
    if integer(diag.get("deadlineMissCount")) > 0:
        reasons.add("deadline_miss")
    if integer(diag.get("schedulingErrorCount")) > 0:
        reasons.add("scheduling_error")
    if integer(scheduling.get("budget_us")) > 0:
        if loop_us > integer(scheduling.get("budget_us")):
            reasons.add("budget_overrun")
    if integer(scheduling.get("deadline_us")) > 0:
        if loop_us > integer(scheduling.get("deadline_us")):
            reasons.add("deadline_miss")
    if has_resource_wait_pressure(diag):
        reasons.add("resource_wait")
    return reasons


def task_periodic_overload_us(interval_ms: int, loop_us: int) -> int:
    interval_us = interval_ms * 1000
    if interval_us <= 0 or loop_us <= interval_us:
        return 0
    return loop_us - interval_us


def task_utilization_over_ppm(interval_before_ms: int,
                              interval_after_ms: int,
                              utilization_ppm: int,
                              target_utilization_ppm: int) -> int:
    if interval_after_ms <= 0 or utilization_ppm <= target_utilization_ppm:
        return 0
    scaled_utilization = ceil_div(
        utilization_ppm * interval_before_ms, interval_after_ms)
    return max(scaled_utilization - target_utilization_ppm, 0)


def task_feasible_interval_limit(interval_ms: int,
                                 diag: Dict[str, Any],
                                 loop_us: int,
                                 limits: SolverLimits) -> int:
    limit = interval_ms
    if interval_ms > 0 and loop_us > interval_ms * 1000:
        limit = max(limit, ceil_div(loop_us, 1000))
    utilization_ppm = integer(diag.get("utilizationPpm"))
    if interval_ms > 0 and utilization_ppm > limits.target_utilization_ppm:
        limit = max(
            limit,
            ceil_div(interval_ms * utilization_ppm,
                     limits.target_utilization_ppm),
        )
    return min(max(limit, interval_ms), limits.max_periodic_interval_ms)


def task_candidate_penalty(interval_before_ms: int,
                           interval_after_ms: int,
                           diag: Dict[str, Any],
                           loop_us: int,
                           limits: SolverLimits) -> int:
    return (
        task_periodic_overload_us(interval_after_ms, loop_us) +
        integer(diag.get("totalResourceWaitUs")) +
        integer(diag.get("schedulingErrorCount")) * 10000 +
        integer(diag.get("budgetOverrunCount")) * 2000 +
        integer(diag.get("deadlineMissCount")) * 5000 +
        task_utilization_over_ppm(
            interval_before_ms,
            interval_after_ms,
            integer(diag.get("utilizationPpm")),
            limits.target_utilization_ppm,
        ) +
        interval_after_ms
    )


def task_candidates(interval_ms: int,
                    diag: Dict[str, Any],
                    loop_us: int,
                    replaceable: bool,
                    limits: SolverLimits) -> List[Dict[str, int]]:
    if not replaceable or interval_ms <= 0:
        return [{
            "intervalMs": interval_ms,
            "penalty": task_candidate_penalty(
                interval_ms, interval_ms, diag, loop_us, limits),
        }]
    max_interval = task_feasible_interval_limit(interval_ms, diag, loop_us, limits)
    return [
        {
            "intervalMs": candidate,
            "penalty": task_candidate_penalty(
                interval_ms, candidate, diag, loop_us, limits),
        }
        for candidate in range(interval_ms, max_interval + 1)
    ]


def report_reason(reasons: Set[str],
                  interval_changed: bool,
                  replaceable: bool) -> str:
    if interval_changed:
        return "global_optimum_interval"
    reason = ordered_reason_text(reasons)
    if replaceable:
        return reason
    return "not_replaceable" if reason == "keep" else f"not_replaceable+{reason}"


def apply_resource_isolation(scheduling: Dict[str, Any],
                             trigger: Dict[str, Any],
                             diag: Dict[str, Any],
                             replaceable: bool) -> None:
    if not replaceable or not has_resource_wait_pressure(diag):
        return
    if integer(scheduling.get("cpu_affinity"), -1) < 0:
        scheduling["cpu_affinity"] = RESOURCE_ISOLATION_CPU_AFFINITY
    if (not bool(scheduling.get("realtime", False)) and
            integer(trigger.get("interval_ms")) > 0):
        scheduling["realtime"] = True
        scheduling["priority"] = RESOURCE_ISOLATION_PRIORITY


def task_decision(task: Dict[str, Any],
                  limits: SolverLimits,
                  diag: Dict[str, Any],
                  catalog_item: Dict[str, Any],
                  candidate: Dict[str, int]
                  ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    result = dict(task)
    trigger = dict(result.get("trigger", {}))
    scheduling = dict(result.get("scheduling", {}))
    name = str(result.get("name", ""))
    scheduling["budget_us"] = integer(catalog_item.get("budgetUs"))
    scheduling["deadline_us"] = integer(catalog_item.get("deadlineUs"))
    scheduling["resource"] = catalog_item.get("resource")
    loop_us = effective_loop_us(diag)
    average_loop_us = integer(diag.get("averageLoopUs"), loop_us)
    utilization_ppm = integer(diag.get("utilizationPpm"))
    interval_ms = integer(trigger.get("interval_ms"))
    reasons = reason_set(diag, loop_us, utilization_ppm, scheduling, limits)
    replaceable = bool(catalog_item.get("replaceable", False))
    target_interval = candidate["intervalMs"]
    if target_interval != interval_ms:
        trigger["interval_ms"] = target_interval
    apply_resource_isolation(scheduling, trigger, diag, replaceable)
    result["trigger"] = trigger
    result["scheduling"] = scheduling
    decision = {
        "kind": "task",
        "name": name,
        "intervalBeforeMs": interval_ms,
        "intervalAfterMs": target_interval,
        "maxLoopUs": integer(diag.get("maxLoopUs")),
        "averageLoopUs": average_loop_us,
        "p90LoopUs": integer(diag.get("p90LoopUs")),
        "p99LoopUs": integer(diag.get("p99LoopUs")),
        "effectiveLoopUs": loop_us,
        "resourceWaitCount": integer(diag.get("resourceWaitCount")),
        "maxResourceWaitUs": integer(diag.get("maxResourceWaitUs")),
        "averageResourceWaitUs": integer(
            diag.get("averageResourceWaitUs")),
        "totalResourceWaitUs": integer(diag.get("totalResourceWaitUs")),
        "utilizationPpm": utilization_ppm,
        "targetUtilizationPpm": limits.target_utilization_ppm,
        "budgetUs": integer(scheduling.get("budget_us")),
        "deadlineUs": integer(scheduling.get("deadline_us")),
        "catalogRole": catalog_item.get("role", ""),
        "replaceable": replaceable,
        "budgetOverrunCount": integer(diag.get("budgetOverrunCount")),
        "deadlineMissCount": integer(diag.get("deadlineMissCount")),
        "schedulingErrorCount": integer(diag.get("schedulingErrorCount")),
        "reason": report_reason(
            reasons, target_interval != interval_ms, replaceable),
    }
    return result, decision


def score_decisions(decisions: List[Dict[str, Any]]) -> Dict[str, int]:
    queue_pressure_sum = sum(
        integer(item.get("pressureAfter"))
        for item in decisions
        if item.get("kind") == "queue"
    )
    overload_sum = sum(
        task_periodic_overload_us(
            integer(item.get("intervalAfterMs")),
            integer(item.get("effectiveLoopUs")),
        )
        for item in decisions
        if item.get("kind") == "task"
    )
    scheduling_errors = sum(
        integer(item.get("schedulingErrorCount"))
        for item in decisions
        if item.get("kind") == "task"
    )
    resource_wait_us = sum(
        integer(item.get("totalResourceWaitUs"))
        for item in decisions
        if item.get("kind") == "task"
    )
    budget_overruns = sum(
        integer(item.get("budgetOverrunCount"))
        for item in decisions
        if item.get("kind") == "task"
    )
    deadline_misses = sum(
        integer(item.get("deadlineMissCount"))
        for item in decisions
        if item.get("kind") == "task"
    )
    utilization_over = sum(
        task_utilization_over_ppm(
            integer(item.get("intervalBeforeMs")),
            integer(item.get("intervalAfterMs")),
            integer(item.get("utilizationPpm")),
            integer(item.get("targetUtilizationPpm")),
        )
        for item in decisions
        if item.get("kind") == "task"
    )
    return {
        "queuePressure": queue_pressure_sum,
        "periodicOverloadUs": overload_sum,
        "resourceWaitUs": resource_wait_us,
        "schedulingErrors": scheduling_errors,
        "budgetOverruns": budget_overruns,
        "deadlineMisses": deadline_misses,
        "utilizationOverPpm": utilization_over,
        "totalPenalty": queue_pressure_sum * 1000 + overload_sum +
        resource_wait_us + scheduling_errors * 10000 + budget_overruns * 2000 +
        deadline_misses * 5000 + utilization_over,
    }


def build_queue_nodes(queues: List[Dict[str, Any]],
                      limits: SolverLimits,
                      diagnostics: Dict[str, Dict[str, Any]]
                      ) -> List[Dict[str, Any]]:
    nodes = []
    for index, queue in enumerate(queues):
        if not isinstance(queue, dict):
            continue
        name = str(queue.get("name", ""))
        diag = diagnostics.get(name, {})
        nodes.append({
            "index": index,
            "queue": queue,
            "diag": diag,
            "candidates": queue_candidates(queue, limits, diag),
        })
    return nodes


def build_task_nodes(tasks: List[Dict[str, Any]],
                     limits: SolverLimits,
                     diagnostics: Dict[str, Dict[str, Any]],
                     catalog: Dict[str, Dict[str, Any]]
                     ) -> List[Dict[str, Any]]:
    nodes = []
    for index, task in enumerate(tasks):
        if not isinstance(task, dict):
            continue
        name = str(task.get("name", ""))
        diag = diagnostics.get(name, {})
        catalog_item = catalog.get(str(task.get("type", "")), {})
        trigger = task.get("trigger", {})
        interval_ms = integer(trigger.get("interval_ms")) if isinstance(
            trigger, dict) else 0
        replaceable = bool(catalog_item.get("replaceable", False))
        loop_us = effective_loop_us(diag)
        nodes.append({
            "index": index,
            "task": task,
            "diag": diag,
            "catalog": catalog_item,
            "candidates": task_candidates(
                interval_ms, diag, loop_us, replaceable, limits),
        })
    return nodes


def solve_global_topology(queue_nodes: List[Dict[str, Any]],
                          task_nodes: List[Dict[str, Any]]
                          ) -> Dict[str, List[int]]:
    return {
        "queues": [
            best_candidate_index(node["candidates"])
            for node in queue_nodes
        ],
        "tasks": [
            best_candidate_index(node["candidates"])
            for node in task_nodes
        ],
    }


def validate_generated_pair(config: Dict[str, Any],
                            profile: Dict[str, Any],
                            catalog: Dict[str, Dict[str, Any]],
                            report: Dict[str, Any]) -> None:
    fields = [
        "targetGraph",
        "topologyVersion",
        "sourceProfile",
        "sourceTimestampMs",
        "generatedAtMs",
        "solverVersion",
    ]
    if config.get("schema") != OPTIMIZED_SCHEMA:
        raise ValueError("EPG optimized config schema mismatch")
    if report.get("schema") != SOLVER_REPORT_SCHEMA:
        raise ValueError("EPG solver report schema mismatch")
    if profile.get("schema") != PROFILE_SCHEMA:
        raise ValueError("EPG solver report profile schema mismatch")
    for field in fields:
        if config.get(field) != report.get(field):
            raise ValueError(f"EPG solver report provenance mismatch: {field}")
    if config.get("sourceProfile") != profile.get("graph"):
        raise ValueError("EPG solver report profile graph mismatch")
    if config.get("topologyVersion") != profile.get("topologyVersion"):
        raise ValueError("EPG solver report profile topology mismatch")
    if integer(config.get("sourceTimestampMs")) != integer(
            profile.get("timestampMs")):
        raise ValueError("EPG solver report profile timestamp mismatch")
    objective = report.get("objective")
    if not isinstance(objective, dict) or not objective.get("name"):
        raise ValueError("EPG solver report objective missing")
    if objective.get("name") != EXACT_SOLVER_OBJECTIVE:
        raise ValueError("EPG solver report objective mismatch")
    score = objective.get("score")
    if not isinstance(score, dict):
        raise ValueError("EPG solver report score missing")
    constraints = report.get("constraints")
    if not isinstance(constraints, dict):
        raise ValueError("EPG solver report constraints missing")
    for field in SOLVER_SCORE_FIELDS:
        if integer(score.get(field), -1) < 0:
            raise ValueError(f"EPG solver report score invalid: {field}")
    for field in SOLVER_CONSTRAINT_FIELDS:
        if integer(constraints.get(field), -1) <= 0:
            raise ValueError(f"EPG solver report constraint invalid: {field}")
    decisions = report.get("decisions")
    if not isinstance(decisions, list):
        raise ValueError("EPG solver report decisions missing")
    source_topology = profile.get("topology", {})
    if not isinstance(source_topology, dict):
        raise ValueError("EPG solver report profile topology missing")
    queue_by_name = config_items_by_name(config, "queues", "queue")
    task_by_name = config_items_by_name(config, "tasks", "task")
    source_queue_by_name = config_items_by_name(
        source_topology, "queues", "queue")
    source_task_by_name = config_items_by_name(source_topology, "tasks", "task")
    expected = {
        f"queue:{item.get('name', '')}"
        for item in queue_by_name.values()
    }
    expected.update({
        f"task:{item.get('name', '')}"
        for item in task_by_name.values()
    })
    actual = set()
    for item in decisions:
        if not isinstance(item, dict):
            raise ValueError("EPG solver report decision invalid")
        kind = require_text(item, "kind", "solver report decision")
        name = require_text(item, "name", "solver report decision")
        require_text(item, "reason", "solver report decision")
        key = f"{kind}:{name}"
        if key in actual:
            raise ValueError(f"EPG solver report duplicates decision: {key}")
        actual.add(key)
        if kind == "queue":
            if name not in queue_by_name:
                raise ValueError(
                    f"EPG solver report queue decision target missing: {name}")
            if name not in source_queue_by_name:
                raise ValueError(
                    f"EPG solver report queue source missing: {name}")
            validate_queue_decision(
                item, source_queue_by_name[name], queue_by_name[name],
                constraints)
            continue
        if kind == "task":
            if name not in task_by_name:
                raise ValueError(
                    f"EPG solver report task decision target missing: {name}")
            if name not in source_task_by_name:
                raise ValueError(
                    f"EPG solver report task source missing: {name}")
            validate_task_decision(
                item, source_task_by_name[name], task_by_name[name],
                constraints, catalog)
            continue
        raise ValueError(f"EPG solver report decision kind unsupported: {kind}")
    if actual != expected:
        raise ValueError("EPG solver report decision coverage mismatch")
    expected_score = score_decisions(decisions)
    for field in SOLVER_SCORE_FIELDS:
        if integer(score.get(field)) != expected_score[field]:
            raise ValueError(f"EPG solver report score mismatch: {field}")
    validate_global_optimum(profile, report)


def decision_by_key(report: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    return {
        f"{item.get('kind')}:{item.get('name')}": item
        for item in report.get("decisions", [])
        if isinstance(item, dict)
    }


def validate_global_optimum(profile: Dict[str, Any],
                            report: Dict[str, Any]) -> None:
    topology = profile.get("topology", {})
    constraints = report.get("constraints", {})
    decisions = decision_by_key(report)
    queue_diag = queue_diagnostics(profile)
    task_diag = task_diagnostics(profile)
    for queue in topology.get("queues", []):
        if not isinstance(queue, dict):
            continue
        name = str(queue.get("name", ""))
        decision = decisions[f"queue:{name}"]
        diag = queue_diag[name]
        if (integer(decision.get("pressureBefore")) !=
                queue_pressure(integer(decision.get("depthBefore")), diag) or
                integer(decision.get("pressureAfter")) !=
                queue_pressure(integer(decision.get("depthAfter")), diag) or
                integer(decision.get("maxDepthObserved")) !=
                integer(diag.get("maxDepthObserved")) or
                integer(decision.get("droppedNewest")) !=
                integer(diag.get("droppedNewest")) or
                integer(decision.get("overwrittenOldest")) !=
                integer(diag.get("overwrittenOldest")) or
                integer(decision.get("pushedPerSecond")) !=
                integer(diag.get("pushedPerSecond")) or
                integer(decision.get("poppedPerSecond")) !=
                integer(diag.get("poppedPerSecond")) or
                integer(decision.get("droppedPerSecond")) !=
                integer(diag.get("droppedPerSecond"))):
            raise ValueError(f"EPG solver report queue metrics mismatch: {name}")
        actual = queue_candidate_penalty(integer(decision.get("depthAfter")), diag)
        best = min(
            queue_candidate_penalty(candidate, diag)
            for candidate in range(
                max(1, integer(decision.get("depthBefore"))),
                max(max(1, integer(decision.get("depthBefore"))),
                    integer(constraints.get("maxQueueDepth"))) + 1)
        )
        if actual != best:
            raise ValueError(f"EPG solver report queue is not optimal: {name}")
    for task in topology.get("tasks", []):
        if not isinstance(task, dict):
            continue
        name = str(task.get("name", ""))
        decision = decisions[f"task:{name}"]
        diag = task_diag[name]
        interval = integer(decision.get("intervalBeforeMs"))
        loop_us = integer(decision.get("effectiveLoopUs"))
        expected_loop_us = effective_loop_us(diag)
        for field in [
                "maxLoopUs",
                "averageLoopUs",
                "p90LoopUs",
                "p99LoopUs",
                "resourceWaitCount",
                "maxResourceWaitUs",
                "averageResourceWaitUs",
                "totalResourceWaitUs",
                "utilizationPpm",
                "budgetOverrunCount",
                "deadlineMissCount",
                "schedulingErrorCount",
        ]:
            if integer(decision.get(field)) != integer(diag.get(field)):
                raise ValueError(
                    f"EPG solver report task metrics mismatch: {name}")
        if loop_us != expected_loop_us:
            raise ValueError(f"EPG solver report task metrics mismatch: {name}")
        replaceable = bool(decision.get("replaceable", False))
        limits = SolverLimits(
            max_queue_depth=integer(constraints.get("maxQueueDepth")),
            max_periodic_interval_ms=integer(
                constraints.get("maxPeriodicIntervalMs")),
            target_utilization_ppm=integer(
                constraints.get("targetUtilizationPpm")),
        )
        actual = task_candidate_penalty(
            interval, integer(decision.get("intervalAfterMs")),
            diag, loop_us, limits)
        best = min(
            item["penalty"]
            for item in task_candidates(interval, diag, loop_us,
                                        replaceable, limits)
        )
        if actual != best:
            raise ValueError(f"EPG solver report task is not optimal: {name}")


def optimize_profile(profile: Dict[str, Any],
                     limits: SolverLimits,
                     generated_at_ms: int) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    topology = profile["topology"]
    queues = topology.get("queues", [])
    tasks = topology.get("tasks", [])
    if not isinstance(queues, list) or not isinstance(tasks, list):
        raise ValueError("EPG profile topology queues/tasks must be arrays")

    queue_diag = queue_diagnostics(profile)
    task_diag = task_diagnostics(profile)
    catalog = task_catalog(profile)
    validate_task_catalog_coverage(tasks, catalog)
    validate_diagnostics_coverage(queues, tasks, queue_diag, task_diag)
    queue_nodes = build_queue_nodes(queues, limits, queue_diag)
    task_nodes = build_task_nodes(tasks, limits, task_diag, catalog)
    solution = solve_global_topology(queue_nodes, task_nodes)
    optimized_queues = [
        dict(queue)
        for queue in queues
        if isinstance(queue, dict)
    ]
    optimized_tasks = [
        dict(task)
        for task in tasks
        if isinstance(task, dict)
    ]
    decisions = []
    for node_index, node in enumerate(queue_nodes):
        candidate = node["candidates"][solution["queues"][node_index]]
        optimized, decision = queue_decision(
            node["queue"], node["diag"], candidate)
        optimized_queues[node_index] = optimized
        decisions.append(decision)
    for node_index, node in enumerate(task_nodes):
        candidate = node["candidates"][solution["tasks"][node_index]]
        optimized, decision = task_decision(
            node["task"], limits, node["diag"], node["catalog"], candidate)
        optimized_tasks[node_index] = optimized
        decisions.append(decision)

    config = {
        "schema": OPTIMIZED_SCHEMA,
        "targetGraph": profile.get("graph", ""),
        "topologyVersion": profile.get("topologyVersion", ""),
        "solverVersion": SOLVER_VERSION,
        "sourceProfile": profile.get("graph", ""),
        "sourceTimestampMs": profile.get("timestampMs", 0),
        "generatedAtMs": generated_at_ms,
        "queues": optimized_queues,
        "tasks": optimized_tasks,
    }
    report = {
        "schema": SOLVER_REPORT_SCHEMA,
        "targetGraph": profile.get("graph", ""),
        "topologyVersion": profile.get("topologyVersion", ""),
        "sourceProfile": profile.get("graph", ""),
        "sourceTimestampMs": profile.get("timestampMs", 0),
        "generatedAtMs": generated_at_ms,
        "solverVersion": SOLVER_VERSION,
        "objective": {
            "name": EXACT_SOLVER_OBJECTIVE,
            "score": score_decisions(decisions),
        },
        "constraints": {
            "maxQueueDepth": limits.max_queue_depth,
            "maxPeriodicIntervalMs": limits.max_periodic_interval_ms,
            "targetUtilizationPpm": limits.target_utilization_ppm,
        },
        "decisions": decisions,
    }
    validate_generated_pair(config, profile, catalog, report)
    return config, report


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n",
                    encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Solve an EPG profile into a runtime GraphConfig JSON")
    parser.add_argument("--profile", required=True, type=Path,
                        help="Input smartdrone.epg.profile.v1 JSON")
    parser.add_argument("--output", required=True, type=Path,
                        help="Output runtime GraphConfig JSON")
    parser.add_argument("--report", type=Path,
                        help="Optional solver report JSON")
    parser.add_argument("--validate-report", type=Path,
                        help="Validate an existing solver report JSON")
    parser.add_argument("--max-queue-depth", type=int, default=16)
    parser.add_argument("--max-periodic-interval-ms", type=int, default=1000)
    parser.add_argument("--target-utilization-ppm", type=int, default=800000)
    parser.add_argument("--generated-at-ms", type=int,
                        help="Override output generation timestamp")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    limits = SolverLimits(
        max_queue_depth=max(1, args.max_queue_depth),
        max_periodic_interval_ms=max(1, args.max_periodic_interval_ms),
        target_utilization_ppm=max(1, args.target_utilization_ppm),
    )
    generated_at_ms = args.generated_at_ms
    if generated_at_ms is None:
        generated_at_ms = int(time.time() * 1000)
    profile = load_profile(args.profile)
    if args.validate_report:
        optimized = json.loads(args.output.read_text(encoding="utf-8"))
        report = json.loads(args.validate_report.read_text(encoding="utf-8"))
        validate_generated_pair(optimized, profile, task_catalog(profile),
                                report)
        return 0
    optimized, report = optimize_profile(profile, limits, generated_at_ms)
    write_json(args.output, optimized)
    if args.report:
        write_json(args.report, report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
