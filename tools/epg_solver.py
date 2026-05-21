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
SOLVER_VERSION = "python-heuristic-v2"


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
    for task in tasks:
        if not isinstance(task, dict):
            continue
        name = require_text(task, "name", "task")
        if name not in task_diag:
            raise ValueError(f"EPG profile diagnostics missing task: {name}")


def queue_pressure(depth: int, diag: Dict[str, Any]) -> int:
    max_depth = integer(diag.get("maxDepthObserved"))
    drops = integer(diag.get("droppedNewest"))
    overwrites = integer(diag.get("overwrittenOldest"))
    return max(max_depth - depth, 0) + drops + overwrites


def normalized_queue(queue: Dict[str, Any], limits: SolverLimits,
                     diagnostics: Dict[str, Dict[str, Any]]
                     ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    result = dict(queue)
    name = str(result.get("name", ""))
    depth = integer(result.get("depth"), 1)
    diag = diagnostics.get(name, {})
    max_depth = integer(diag.get("maxDepthObserved"))
    pressure = queue_pressure(depth, diag)

    target_depth = depth
    if pressure > 0:
        target_depth = max(depth + 1, max_depth * 2, 2)
    result["depth"] = min(max(target_depth, 1), limits.max_queue_depth)
    decision = {
        "kind": "queue",
        "name": name,
        "depthBefore": depth,
        "depthAfter": result["depth"],
        "pressureBefore": pressure,
        "pushedPerSecond": integer(diag.get("pushedPerSecond")),
        "poppedPerSecond": integer(diag.get("poppedPerSecond")),
        "droppedPerSecond": integer(diag.get("droppedPerSecond")),
        "reason": "increase_depth" if result["depth"] != depth else "keep",
    }
    return result, decision


def effective_loop_us(diag: Dict[str, Any]) -> int:
    return max(
        integer(diag.get("p99LoopUs")),
        integer(diag.get("p90LoopUs")),
        integer(diag.get("maxLoopUs")),
        integer(diag.get("averageLoopUs")),
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
    return reasons


def target_interval_ms(interval_ms: int,
                       loop_us: int,
                       utilization_ppm: int,
                       limits: SolverLimits) -> int:
    target = interval_ms
    if interval_ms > 0 and loop_us > interval_ms * 1000:
        target = max(target, ceil_div(loop_us, 1000))
    if interval_ms > 0 and utilization_ppm > limits.target_utilization_ppm:
        target = max(
            target,
            ceil_div(interval_ms * utilization_ppm, limits.target_utilization_ppm),
        )
    return min(max(target, interval_ms), limits.max_periodic_interval_ms)


def report_reason(reasons: Set[str],
                  interval_changed: bool,
                  replaceable: bool) -> str:
    if interval_changed:
        return "increase_interval"
    reason = "+".join(sorted(reasons)) if reasons else "keep"
    if replaceable:
        return reason
    return "not_replaceable" if reason == "keep" else f"not_replaceable+{reason}"


def normalized_task(task: Dict[str, Any], limits: SolverLimits,
                    diagnostics: Dict[str, Dict[str, Any]],
                    catalog: Dict[str, Dict[str, Any]]
                    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    result = dict(task)
    trigger = dict(result.get("trigger", {}))
    scheduling = dict(result.get("scheduling", {}))
    name = str(result.get("name", ""))
    diag = diagnostics.get(name, {})
    catalog_item = catalog.get(str(result.get("type", "")), {})
    scheduling["budget_us"] = integer(catalog_item.get("budgetUs"))
    scheduling["deadline_us"] = integer(catalog_item.get("deadlineUs"))
    scheduling["resource"] = catalog_item.get("resource")
    loop_us = effective_loop_us(diag)
    average_loop_us = integer(diag.get("averageLoopUs"), loop_us)
    utilization_ppm = integer(diag.get("utilizationPpm"))
    interval_ms = integer(trigger.get("interval_ms"))
    reasons = reason_set(diag, loop_us, utilization_ppm, scheduling, limits)
    replaceable = bool(catalog_item.get("replaceable", False))
    target_interval = interval_ms
    if replaceable:
        target_interval = target_interval_ms(
            interval_ms, loop_us, utilization_ppm, limits)
    if target_interval != interval_ms:
        trigger["interval_ms"] = target_interval
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
        integer(item.get("pressureBefore"))
        for item in decisions
        if item.get("kind") == "queue"
    )
    overload_sum = sum(
        max(integer(item.get("effectiveLoopUs")) -
            integer(item.get("intervalBeforeMs")) * 1000, 0)
        for item in decisions
        if item.get("kind") == "task"
    )
    scheduling_errors = sum(
        integer(item.get("schedulingErrorCount"))
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
        max(integer(item.get("utilizationPpm")) -
            integer(item.get("targetUtilizationPpm")), 0)
        for item in decisions
        if item.get("kind") == "task"
    )
    return {
        "queuePressure": queue_pressure_sum,
        "periodicOverloadUs": overload_sum,
        "schedulingErrors": scheduling_errors,
        "budgetOverruns": budget_overruns,
        "deadlineMisses": deadline_misses,
        "utilizationOverPpm": utilization_over,
        "totalPenalty": queue_pressure_sum * 1000 + overload_sum +
        scheduling_errors * 10000 + budget_overruns * 2000 +
        deadline_misses * 5000 + utilization_over,
    }


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
    optimized_queues = []
    optimized_tasks = []
    decisions = []
    for queue in queues:
        if isinstance(queue, dict):
            optimized, decision = normalized_queue(queue, limits, queue_diag)
            optimized_queues.append(optimized)
            decisions.append(decision)
    for task in tasks:
        if isinstance(task, dict):
            optimized, decision = normalized_task(task, limits, task_diag, catalog)
            optimized_tasks.append(optimized)
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
        "schema": "smartdrone.epg.solver_report.v1",
        "targetGraph": profile.get("graph", ""),
        "topologyVersion": profile.get("topologyVersion", ""),
        "sourceProfile": profile.get("graph", ""),
        "sourceTimestampMs": profile.get("timestampMs", 0),
        "generatedAtMs": generated_at_ms,
        "solverVersion": SOLVER_VERSION,
        "objective": {
            "name": "minimize_epg_pressure_overload_deadline_and_scheduling_penalty",
            "score": score_decisions(decisions),
        },
        "constraints": {
            "maxQueueDepth": limits.max_queue_depth,
            "maxPeriodicIntervalMs": limits.max_periodic_interval_ms,
            "targetUtilizationPpm": limits.target_utilization_ppm,
        },
        "decisions": decisions,
    }
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
    optimized, report = optimize_profile(
        load_profile(args.profile), limits, generated_at_ms)
    write_json(args.output, optimized)
    if args.report:
        write_json(args.report, report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
