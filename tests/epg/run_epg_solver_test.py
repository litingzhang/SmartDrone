#!/usr/bin/env python3
"""Smoke test for tools/epg_solver.py."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


def minimal_queue_diagnostics() -> dict:
    return {
        "maxDepthObserved": 0,
        "droppedNewest": 0,
        "overwrittenOldest": 0,
        "pushedPerSecond": 0,
        "poppedPerSecond": 0,
        "droppedPerSecond": 0,
    }


def write_profile(path: Path) -> None:
    payload = {
        "schema": "smartdrone.epg.profile.v1",
        "graph": "test_graph",
        "topologyVersion": "test-topology-v1",
        "timestampMs": 123,
        "taskCatalog": [
            {
                "taskType": "TestSourceTask",
                "role": "source",
                "resource": "cpu",
                "resourceAlternates": ["cpu_isolated"],
                "budgetUs": 1500,
                "deadlineUs": 2200,
                "replaceable": True,
            }
        ],
        "topology": {
            "queues": [
                {
                    "name": "packets",
                    "type": "TestPacket",
                    "depth": 1,
                    "overflow": "drop_newest",
                }
            ],
            "tasks": [
                {
                    "name": "source",
                    "type": "TestSourceTask",
                    "trigger": {"mode": "periodic", "interval_ms": 1},
                    "scheduling": {
                        "resource": "cpu",
                        "cpu_affinity": -1,
                        "budget_us": 1500,
                        "deadline_us": 2200,
                        "realtime": False,
                        "priority": 0,
                    },
                    "inputs": {},
                    "outputs": {"0": "packets"},
                }
            ],
        },
        "diagnostics": {
            "queues": {
                "packets": {
                    "maxDepthObserved": 1,
                    "droppedNewest": 2,
                    "overwrittenOldest": 0,
                    "pushedPerSecond": 900,
                    "poppedPerSecond": 450,
                    "droppedPerSecond": 100,
                }
            },
            "tasks": {
                "source": {
                    "maxLoopUs": 2500,
                    "p90LoopUs": 2100,
                    "p99LoopUs": 2500,
                    "averageLoopUs": 1800,
                    "resourceWaitCount": 2,
                    "maxResourceWaitUs": 1500,
                    "averageResourceWaitUs": 1250,
                    "totalResourceWaitUs": 2500,
                    "utilizationPpm": 900000,
                    "loopCount": 10,
                    "errorCount": 0,
                    "budgetOverrunCount": 2,
                    "deadlineMissCount": 1,
                    "schedulingErrorCount": 1,
                }
            },
        },
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def write_non_replaceable_profile(path: Path) -> None:
    write_profile(path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["taskCatalog"][0]["replaceable"] = False
    path.write_text(json.dumps(payload), encoding="utf-8")


def write_accuracy_preserving_profile(path: Path) -> None:
    write_profile(path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["taskCatalog"][0]["preserveAccuracy"] = True
    payload["diagnostics"]["queues"]["packets"]["maxDepthObserved"] = 5
    payload["diagnostics"]["queues"]["packets"]["droppedNewest"] = 0
    payload["diagnostics"]["queues"]["packets"]["droppedPerSecond"] = 0
    path.write_text(json.dumps(payload), encoding="utf-8")


def write_depth_optimization_profile(path: Path) -> None:
    write_profile(path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["topology"]["queues"][0]["depth"] = 1
    payload["diagnostics"]["queues"]["packets"]["maxDepthObserved"] = 5
    payload["diagnostics"]["queues"]["packets"]["droppedNewest"] = 0
    payload["diagnostics"]["queues"]["packets"]["overwrittenOldest"] = 0
    payload["diagnostics"]["queues"]["packets"]["droppedPerSecond"] = 0
    path.write_text(json.dumps(payload), encoding="utf-8")


def task_catalog_entry(task_type: str) -> dict:
    budget_us = 5000 if task_type == "TestSinkTask" else 1000
    return {
        "taskType": task_type,
        "role": task_type,
        "resource": "cpu",
        "budgetUs": budget_us,
        "deadlineUs": 5000,
        "replaceable": False,
    }


def task_diagnostics(loop_us: int) -> dict:
    return {
        "maxLoopUs": loop_us,
        "p90LoopUs": loop_us,
        "p99LoopUs": loop_us,
        "averageLoopUs": loop_us,
        "resourceWaitCount": 0,
        "maxResourceWaitUs": 0,
        "averageResourceWaitUs": 0,
        "totalResourceWaitUs": 0,
        "utilizationPpm": 0,
        "loopCount": 1,
        "errorCount": 0,
        "budgetOverrunCount": 0,
        "deadlineMissCount": 0,
        "schedulingErrorCount": 0,
    }


def write_topology_schedule_profile(path: Path) -> None:
    payload = {
        "schema": "smartdrone.epg.profile.v1",
        "graph": "test_graph",
        "topologyVersion": "test-topology-v1",
        "timestampMs": 123,
        "taskCatalog": [
            task_catalog_entry("TestSourceTask"),
            task_catalog_entry("TestForwardTask"),
            task_catalog_entry("TestSinkTask"),
        ],
        "topology": {
            "queues": [
                {"name": "source_to_left", "type": "TestPacket",
                 "depth": 1, "overflow": "drop_newest"},
                {"name": "source_to_right", "type": "TestPacket",
                 "depth": 1, "overflow": "drop_newest"},
                {"name": "left_to_sink", "type": "TestPacket",
                 "depth": 1, "overflow": "drop_newest"},
                {"name": "right_to_sink", "type": "TestPacket",
                 "depth": 1, "overflow": "drop_newest"},
            ],
            "tasks": [
                {
                    "name": "sink",
                    "type": "TestSinkTask",
                    "trigger": {
                        "mode": "any_queue_ready",
                        "queues": ["left_to_sink", "right_to_sink"],
                    },
                    "inputs": {"0": "left_to_sink", "1": "right_to_sink"},
                    "outputs": {},
                },
                {
                    "name": "left",
                    "type": "TestForwardTask",
                    "trigger": {
                        "mode": "any_queue_ready",
                        "queues": ["source_to_left"],
                    },
                    "inputs": {"0": "source_to_left"},
                    "outputs": {"0": "left_to_sink"},
                },
                {
                    "name": "right",
                    "type": "TestForwardTask",
                    "trigger": {
                        "mode": "any_queue_ready",
                        "queues": ["source_to_right"],
                    },
                    "inputs": {"0": "source_to_right"},
                    "outputs": {"0": "right_to_sink"},
                },
                {
                    "name": "source",
                    "type": "TestSourceTask",
                    "trigger": {"mode": "periodic", "interval_ms": 1},
                    "inputs": {},
                    "outputs": {"0": "source_to_left",
                                "1": "source_to_right"},
                },
            ],
        },
        "diagnostics": {
            "queues": {
                "source_to_left": minimal_queue_diagnostics(),
                "source_to_right": minimal_queue_diagnostics(),
                "left_to_sink": minimal_queue_diagnostics(),
                "right_to_sink": minimal_queue_diagnostics(),
            },
            "tasks": {
                "source": task_diagnostics(1200),
                "left": task_diagnostics(2000),
                "right": task_diagnostics(3000),
                "sink": task_diagnostics(400),
            },
        },
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def expect_report_failure(repo_root: Path,
                          profile_path: Path,
                          output_path: Path,
                          report_path: Path,
                          validate_report_path: Path,
                          expected: str) -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(repo_root / "tools" / "epg_solver.py"),
            "--profile",
            str(profile_path),
            "--output",
            str(output_path),
            "--validate-report",
            str(validate_report_path),
            "--generated-at-ms",
            "456",
        ],
        check=False,
        stderr=subprocess.PIPE,
        text=True,
    )
    assert result.returncode != 0
    assert expected in result.stderr


def expect_solver_failure(repo_root: Path,
                          profile_path: Path,
                          output_path: Path,
                          expected: str) -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(repo_root / "tools" / "epg_solver.py"),
            "--profile",
            str(profile_path),
            "--output",
            str(output_path),
        ],
        check=False,
        stderr=subprocess.PIPE,
        text=True,
    )
    assert result.returncode != 0
    assert expected in result.stderr


def write_bad_catalog_profile(path: Path,
                              catalog: object,
                              task_type: str = "TestSourceTask") -> None:
    payload = {
        "schema": "smartdrone.epg.profile.v1",
        "graph": "test_graph",
        "topologyVersion": "test-topology-v1",
        "timestampMs": 123,
        "taskCatalog": catalog,
        "topology": {
            "queues": [],
            "tasks": [
                {
                    "name": "source",
                    "type": task_type,
                    "trigger": {"mode": "periodic", "interval_ms": 1},
                    "inputs": {},
                    "outputs": {},
                }
            ],
        },
        "diagnostics": {"queues": {}, "tasks": {}},
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def write_missing_diagnostics_profile(path: Path) -> None:
    payload = {
        "schema": "smartdrone.epg.profile.v1",
        "graph": "test_graph",
        "topologyVersion": "test-topology-v1",
        "timestampMs": 123,
        "taskCatalog": [
            {
                "taskType": "TestSourceTask",
                "role": "source",
                "resource": "cpu",
                "budgetUs": 1500,
                "deadlineUs": 2200,
            }
        ],
        "topology": {
            "queues": [
                {
                    "name": "packets",
                    "type": "TestPacket",
                    "depth": 1,
                    "overflow": "drop_newest",
                }
            ],
            "tasks": [
                {
                    "name": "source",
                    "type": "TestSourceTask",
                    "trigger": {"mode": "periodic", "interval_ms": 1},
                    "inputs": {},
                    "outputs": {"0": "packets"},
                }
            ],
        },
        "diagnostics": {
            "queues": {"packets": minimal_queue_diagnostics()},
            "tasks": {},
        },
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def write_missing_diagnostic_field_profile(path: Path) -> None:
    write_profile(path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    del payload["diagnostics"]["tasks"]["source"]["p99LoopUs"]
    path.write_text(json.dumps(payload), encoding="utf-8")


def main() -> int:
    repo_root = Path(sys.argv[1])
    work_dir = Path(sys.argv[2])
    work_dir.mkdir(parents=True, exist_ok=True)
    profile_path = work_dir / "profile.json"
    output_path = work_dir / "optimized.json"
    report_path = work_dir / "report.json"
    write_profile(profile_path)

    subprocess.run(
        [
            sys.executable,
            str(repo_root / "tools" / "epg_solver.py"),
            "--profile",
            str(profile_path),
            "--output",
            str(output_path),
            "--report",
            str(report_path),
            "--max-queue-depth",
            "8",
            "--generated-at-ms",
            "456",
        ],
        check=True,
    )
    optimized = json.loads(output_path.read_text(encoding="utf-8"))
    assert optimized["schema"] == "smartdrone.epg.optimized_config.v1"
    assert optimized["targetGraph"] == "test_graph"
    assert optimized["topologyVersion"] == "test-topology-v1"
    assert optimized["solverVersion"] == "python-exact-v1"
    assert optimized["sourceProfile"] == "test_graph"
    assert optimized["sourceTimestampMs"] == 123
    assert optimized["generatedAtMs"] == 456
    assert optimized["queues"][0]["depth"] == 1
    assert optimized["tasks"][0]["trigger"]["interval_ms"] == 3
    assert optimized["tasks"][0]["scheduling"]["resource"] == "cpu_isolated"
    assert optimized["tasks"][0]["scheduling"]["backpressure_outputs"] == [0]
    assert optimized["tasks"][0]["scheduling"]["cpu_affinity"] == 2
    assert optimized["tasks"][0]["scheduling"]["realtime"] is True
    assert optimized["tasks"][0]["scheduling"]["priority"] == 20
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["schema"] == "smartdrone.epg.solver_report.v1"
    assert report["targetGraph"] == "test_graph"
    assert report["topologyVersion"] == "test-topology-v1"
    assert report["sourceProfile"] == "test_graph"
    assert report["sourceTimestampMs"] == 123
    assert report["generatedAtMs"] == 456
    assert report["solverVersion"] == "python-exact-v1"
    for field in [
            "targetGraph",
            "topologyVersion",
            "sourceProfile",
            "sourceTimestampMs",
            "generatedAtMs",
            "solverVersion",
    ]:
        assert report[field] == optimized[field]
    assert report["objective"]["score"]["totalPenalty"] > 0
    assert report["objective"]["score"]["budgetOverruns"] == 2
    assert report["objective"]["score"]["deadlineMisses"] == 1
    assert report["objective"]["score"]["resourceWaitUs"] == 0
    assert report["objective"]["score"]["topologyPenalty"] == 31
    assert report["constraints"]["maxQueueDepth"] == 8
    assert report["decisions"][0]["depthAfter"] == optimized["queues"][0]["depth"]
    assert report["decisions"][0]["pressureAfter"] == 2
    assert report["decisions"][1]["intervalAfterMs"] == (
        optimized["tasks"][0]["trigger"]["interval_ms"])
    assert report["decisions"][0]["droppedPerSecond"] == 100
    assert report["decisions"][1]["budgetUs"] == 1500
    assert report["decisions"][1]["totalResourceWaitUs"] == 2500
    assert report["decisions"][1]["predictedResourceWaitUs"] == 0
    assert report["decisions"][1]["resourceBefore"] == "cpu"
    assert report["decisions"][1]["resourceAfter"] == "cpu_isolated"
    assert report["decisions"][1]["cpuAffinityBefore"] == -1
    assert report["decisions"][1]["cpuAffinityAfter"] == 2
    assert report["decisions"][1]["catalogRole"] == "source"
    assert report["decisions"][1]["replaceable"] is True
    assert report["decisions"][1]["backpressureBefore"] == []
    assert report["decisions"][1]["backpressureAfter"] == [0]
    assert report["decisions"][1]["topologyPenalty"] == 31
    assert report["decisions"][0]["reason"] == "keep"
    assert report["decisions"][1]["reason"] == (
        "global_optimum_interval+global_optimum_backpressure+global_optimum_resource+global_optimum_cpu_binding")
    fixed_profile = work_dir / "non_replaceable_profile.json"
    fixed_output = work_dir / "non_replaceable_optimized.json"
    fixed_report_path = work_dir / "non_replaceable_report.json"
    write_non_replaceable_profile(fixed_profile)
    subprocess.run(
        [
            sys.executable,
            str(repo_root / "tools" / "epg_solver.py"),
            "--profile",
            str(fixed_profile),
            "--output",
            str(fixed_output),
            "--report",
            str(fixed_report_path),
            "--max-queue-depth",
            "8",
            "--generated-at-ms",
            "456",
        ],
        check=True,
    )
    fixed_optimized = json.loads(fixed_output.read_text(encoding="utf-8"))
    fixed_report = json.loads(fixed_report_path.read_text(encoding="utf-8"))
    assert fixed_optimized["tasks"][0]["trigger"]["interval_ms"] == 1
    assert fixed_report["decisions"][1]["reason"].startswith("not_replaceable+")
    accuracy_profile = work_dir / "accuracy_profile.json"
    accuracy_output = work_dir / "accuracy_optimized.json"
    accuracy_report_path = work_dir / "accuracy_report.json"
    write_accuracy_preserving_profile(accuracy_profile)
    subprocess.run(
        [
            sys.executable,
            str(repo_root / "tools" / "epg_solver.py"),
            "--profile",
            str(accuracy_profile),
            "--output",
            str(accuracy_output),
            "--report",
            str(accuracy_report_path),
            "--max-queue-depth",
            "8",
            "--generated-at-ms",
            "456",
        ],
        check=True,
    )
    accuracy_optimized = json.loads(
        accuracy_output.read_text(encoding="utf-8"))
    accuracy_report = json.loads(
        accuracy_report_path.read_text(encoding="utf-8"))
    accuracy_task = accuracy_optimized["tasks"][0]
    assert accuracy_optimized["queues"][0]["depth"] == 1
    assert accuracy_report["decisions"][0]["reason"] == "keep"
    assert accuracy_report["decisions"][0]["pressureAfter"] == 4
    accuracy_decision = accuracy_report["decisions"][1]
    assert accuracy_task["trigger"]["interval_ms"] == 1
    assert accuracy_task["scheduling"]["resource"] == "cpu"
    assert accuracy_task["scheduling"]["backpressure_outputs"] == []
    assert accuracy_task["scheduling"]["cpu_affinity"] == -1
    assert accuracy_decision["cpuAffinityBefore"] == -1
    assert accuracy_decision["cpuAffinityAfter"] == -1
    assert accuracy_task["scheduling"]["realtime"] is False
    assert accuracy_task["scheduling"]["priority"] == 0
    assert accuracy_decision["reason"] == (
        "utilization_over_target+budget_overrun+deadline_miss+scheduling_error+resource_wait")
    assert accuracy_report["objective"]["score"]["resourceWaitUs"] == 2500
    depth_profile = work_dir / "depth_profile.json"
    depth_output = work_dir / "depth_optimized.json"
    depth_report_path = work_dir / "depth_report.json"
    write_depth_optimization_profile(depth_profile)
    subprocess.run(
        [
            sys.executable,
            str(repo_root / "tools" / "epg_solver.py"),
            "--profile",
            str(depth_profile),
            "--output",
            str(depth_output),
            "--report",
            str(depth_report_path),
            "--max-queue-depth",
            "8",
            "--generated-at-ms",
            "456",
        ],
        check=True,
    )
    depth_optimized = json.loads(depth_output.read_text(encoding="utf-8"))
    depth_report = json.loads(depth_report_path.read_text(encoding="utf-8"))
    assert depth_optimized["queues"][0]["depth"] == 1
    assert depth_report["decisions"][0]["pressureAfter"] == 4
    assert depth_report["decisions"][0]["reason"] == "keep"
    topology_profile = work_dir / "topology_profile.json"
    topology_output = work_dir / "topology_optimized.json"
    topology_report_path = work_dir / "topology_report.json"
    write_topology_schedule_profile(topology_profile)
    subprocess.run(
        [
            sys.executable,
            str(repo_root / "tools" / "epg_solver.py"),
            "--profile",
            str(topology_profile),
            "--output",
            str(topology_output),
            "--report",
            str(topology_report_path),
            "--max-queue-depth",
            "8",
            "--generated-at-ms",
            "456",
        ],
        check=True,
    )
    topology_optimized = json.loads(
        topology_output.read_text(encoding="utf-8"))
    topology_report = json.loads(
        topology_report_path.read_text(encoding="utf-8"))
    assert [task["name"] for task in topology_optimized["tasks"]] == [
        "source", "left", "right", "sink"]
    topology_schedule = {
        task["name"]: task["scheduling"]
        for task in topology_optimized["tasks"]
    }
    assert topology_schedule["source"]["topology_level"] == 0
    assert topology_schedule["left"]["topology_level"] == 1
    assert topology_schedule["right"]["topology_level"] == 1
    assert topology_schedule["sink"]["topology_level"] == 2
    assert topology_schedule["source"]["phase_offset_ms"] == 0
    assert topology_schedule["left"]["phase_offset_ms"] == 2
    assert topology_schedule["right"]["phase_offset_ms"] == 2
    assert topology_schedule["sink"]["phase_offset_ms"] == 5
    task_decisions = {
        item["name"]: item
        for item in topology_report["decisions"]
        if item["kind"] == "task"
    }
    assert task_decisions["left"]["phaseOffsetMs"] == 2
    assert task_decisions["right"]["phaseOffsetMs"] == 2
    assert task_decisions["sink"]["durationMs"] == 1
    stale_report_profile = work_dir / "stale_report_profile.json"
    stale_output = work_dir / "stale_report_optimized.json"
    stale_report_path = work_dir / "stale_report_report.json"
    write_depth_optimization_profile(stale_report_profile)
    stale_output.write_text(depth_output.read_text(encoding="utf-8"),
                            encoding="utf-8")
    stale_report_path.write_text(json.dumps(depth_report), encoding="utf-8")
    stale_payload = json.loads(stale_report_profile.read_text(encoding="utf-8"))
    stale_payload["diagnostics"]["queues"]["packets"]["maxDepthObserved"] = 6
    stale_report_profile.write_text(json.dumps(stale_payload), encoding="utf-8")
    expect_report_failure(
        repo_root,
        stale_report_profile,
        stale_output,
        stale_report_path,
        stale_report_path,
        "queue metrics mismatch: packets")
    profile_root = work_dir / "profiles"
    output_root = work_dir / "batch"
    profile_root.mkdir(parents=True, exist_ok=True)
    write_profile(profile_root / "smartdrone_epg_slam_profile.json")
    subprocess.run(
        [
            sys.executable,
            str(repo_root / "tools" / "epg_optimize_all.py"),
            "--profile-root",
            str(profile_root),
            "--output-root",
            str(output_root),
            "--generated-at-ms",
            "789",
        ],
        check=True,
    )
    assert (output_root / "optimized_slam_session_graph.json").exists()
    assert (output_root / "optimized_slam_session_graph_report.json").exists()
    batch_optimized = json.loads(
        (output_root / "optimized_slam_session_graph.json").read_text(
            encoding="utf-8"))
    batch_report = json.loads(
        (output_root / "optimized_slam_session_graph_report.json").read_text(
            encoding="utf-8"))
    assert batch_optimized["generatedAtMs"] == 789
    assert batch_report["generatedAtMs"] == 789

    bad_profile = work_dir / "bad_catalog.json"
    bad_output = work_dir / "bad_optimized.json"
    write_bad_catalog_profile(
        bad_profile,
        [
            {
                "taskType": "TestSourceTask",
                "role": "source",
                "resource": "cpu",
                "budgetUs": 2200,
                "deadlineUs": 1500,
            }
        ],
    )
    expect_solver_failure(repo_root, bad_profile, bad_output, "timing invalid")
    write_bad_catalog_profile(
        bad_profile,
        [
            {
                "taskType": "TestSourceTask",
                "role": "source",
                "resource": "cpu",
                "budgetUs": 1500,
                "deadlineUs": 2200,
            },
            {
                "taskType": "TestSourceTask",
                "role": "source",
                "resource": "cpu",
                "budgetUs": 1500,
                "deadlineUs": 2200,
            },
        ],
    )
    expect_solver_failure(repo_root, bad_profile, bad_output, "duplicates")
    write_bad_catalog_profile(bad_profile, [], "MissingTask")
    expect_solver_failure(repo_root, bad_profile, bad_output, "must not be empty")
    write_bad_catalog_profile(
        bad_profile,
        [
            {
                "taskType": "OtherTask",
                "role": "source",
                "resource": "cpu",
                "budgetUs": 1500,
                "deadlineUs": 2200,
            }
        ],
        "MissingTask",
    )
    expect_solver_failure(repo_root, bad_profile, bad_output, "missing catalog")
    missing_diag_profile = work_dir / "missing_diagnostics.json"
    write_missing_diagnostics_profile(missing_diag_profile)
    expect_solver_failure(
        repo_root,
        missing_diag_profile,
        bad_output,
        "diagnostics missing task: source")
    missing_diag_field_profile = work_dir / "missing_diagnostic_field.json"
    write_missing_diagnostic_field_profile(missing_diag_field_profile)
    expect_solver_failure(
        repo_root,
        missing_diag_field_profile,
        bad_output,
        "diagnostics task missing p99LoopUs: source")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
