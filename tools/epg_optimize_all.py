#!/usr/bin/env python3
"""Solve all known SmartDrone EPG profiles into deployable configs."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List

from epg_solver import SolverLimits, load_profile, optimize_profile, write_json


@dataclass(frozen=True)
class EpgProfileTarget:
    domain: str
    profile_name: str
    config_name: str


TARGETS = (
    EpgProfileTarget(
        "system",
        "smartdrone_epg_system_profile.json",
        "optimized_system_runtime_graph.json",
    ),
    EpgProfileTarget(
        "slam",
        "smartdrone_epg_slam_profile.json",
        "optimized_slam_session_graph.json",
    ),
    EpgProfileTarget(
        "calib",
        "smartdrone_epg_calib_profile.json",
        "optimized_calib_session_graph.json",
    ),
)


def report_name_for_config(config_name: str) -> str:
    config_path = Path(config_name)
    if config_path.suffix == ".json":
        return f"{config_path.stem}_report.json"
    return f"{config_name}_report.json"


def solve_target(target: EpgProfileTarget,
                 profile_root: Path,
                 output_root: Path,
                 limits: SolverLimits,
                 generated_at_ms: int) -> Dict[str, Any]:
    profile_path = profile_root / target.profile_name
    output_path = output_root / target.config_name
    report_path = output_root / report_name_for_config(target.config_name)
    if not profile_path.exists():
        return {
            "domain": target.domain,
            "status": "missing_profile",
            "profile": str(profile_path),
        }
    optimized, report = optimize_profile(
        load_profile(profile_path), limits, generated_at_ms)
    write_json(output_path, optimized)
    write_json(report_path, report)
    return {
        "domain": target.domain,
        "status": "optimized",
        "profile": str(profile_path),
        "output": str(output_path),
        "report": str(report_path),
        "score": report["objective"]["score"],
    }


def build_limits(args: argparse.Namespace) -> SolverLimits:
    return SolverLimits(
        max_queue_depth=max(1, args.max_queue_depth),
        max_periodic_interval_ms=max(1, args.max_periodic_interval_ms),
        target_utilization_ppm=max(1, args.target_utilization_ppm),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Solve system/slam/calib EPG profiles into optimized configs")
    parser.add_argument("--profile-root", type=Path, default=Path("/tmp"))
    parser.add_argument("--output-root", type=Path, default=Path("output/epg"))
    parser.add_argument("--require-all", action="store_true")
    parser.add_argument("--max-queue-depth", type=int, default=16)
    parser.add_argument("--max-periodic-interval-ms", type=int, default=1000)
    parser.add_argument("--target-utilization-ppm", type=int, default=800000)
    parser.add_argument("--generated-at-ms", type=int,
                        help="Override output generation timestamp")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    limits = build_limits(args)
    generated_at_ms = args.generated_at_ms
    if generated_at_ms is None:
        generated_at_ms = int(time.time() * 1000)
    results: List[Dict[str, Any]] = [
        solve_target(target, args.profile_root, args.output_root, limits,
                     generated_at_ms)
        for target in TARGETS
    ]
    print(json.dumps({"results": results}, indent=2, sort_keys=False))
    if args.require_all:
        return 1 if any(item["status"] != "optimized" for item in results) else 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
