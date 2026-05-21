#!/usr/bin/env python3
"""Smoke test for tools/epg_solver.py."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


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
            "queues": {"packets": {}},
            "tasks": {},
        },
    }
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
    assert optimized["solverVersion"] == "python-heuristic-v2"
    assert optimized["sourceProfile"] == "test_graph"
    assert optimized["sourceTimestampMs"] == 123
    assert optimized["generatedAtMs"] == 456
    assert optimized["queues"][0]["depth"] > 1
    assert optimized["tasks"][0]["trigger"]["interval_ms"] == 3
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["schema"] == "smartdrone.epg.solver_report.v1"
    assert report["generatedAtMs"] == 456
    assert report["objective"]["score"]["totalPenalty"] > 0
    assert report["objective"]["score"]["budgetOverruns"] == 2
    assert report["objective"]["score"]["deadlineMisses"] == 1
    assert report["constraints"]["maxQueueDepth"] == 8
    assert report["decisions"][0]["droppedPerSecond"] == 100
    assert report["decisions"][1]["budgetUs"] == 1500
    assert report["decisions"][1]["catalogRole"] == "source"
    assert report["decisions"][1]["replaceable"] is True
    assert report["decisions"][0]["reason"] == "increase_depth"
    assert report["decisions"][1]["reason"] == "increase_interval"
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
