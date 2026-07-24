#!/usr/bin/env python3
"""Analyze PX4 ULog hover truth, estimator, and visual odometry data."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, fields
from pathlib import Path
from typing import Any, Sequence

from sitl_hover.metrics import AnalysisOptions, analyze_ulog
from sitl_hover.provenance import (
    ANALYSIS_OPTION_FIELDS,
    ProvenanceError,
    RunProvenance,
    apply_provenance_gate,
    unverified_provenance,
    verify_run_manifest,
)


class AnalysisUsageError(ValueError):
    """Raised when diagnostic analysis lacks an explicit interpretation."""


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate trajectory.csv, metrics.json, and an optional PNG from a PX4 ULog.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--run-manifest", type=Path, help="managed run manifest required for formal acceptance")
    parser.add_argument("--profile", choices=("control", "truth", "vision"))
    parser.add_argument("--scenario", choices=("nominal", "impulse", "quality", "loss"))
    parser.add_argument("--target-x", type=float)
    parser.add_argument("--target-y", type=float)
    parser.add_argument("--target-z", type=float)
    parser.add_argument("--hover-start-s", type=float, help="PX4 boot time; both start and end must be supplied")
    parser.add_argument("--hover-end-s", type=float, help="PX4 boot time; both start and end must be supplied")
    parser.add_argument(
        "--disturbance-time-s", type=float,
        help="PX4 boot time; defaults to hover start plus 15 seconds",
    )
    parser.add_argument("--horizontal-rmse-limit-m", type=float)
    parser.add_argument("--horizontal-max-limit-m", type=float)
    parser.add_argument("--vertical-rmse-limit-m", type=float)
    parser.add_argument("--vertical-max-limit-m", type=float)
    parser.add_argument("--odometry-rate-limit-hz", type=float)
    parser.add_argument("--latency-p95-limit-ms", type=float)
    parser.add_argument("--visual-availability-limit", type=float)
    parser.add_argument("--recovery-limit-s", type=float)
    parser.add_argument("--minimum-hover-duration-s", type=float)
    return parser.parse_args(argv)


def _options(args: argparse.Namespace, provenance: RunProvenance) -> AnalysisOptions:
    if provenance.verified:
        options = _formal_options(args, provenance)
    else:
        options = _diagnostic_options(args, provenance)
    _validate_options(options)
    return options


def _validate_options(options: AnalysisOptions) -> None:
    if (options.hover_start_s is None) != (options.hover_end_s is None):
        raise ValueError("--hover-start-s and --hover-end-s must be supplied together")
    limits = (
        options.horizontal_rmse_limit_m, options.horizontal_max_limit_m,
        options.vertical_rmse_limit_m, options.vertical_max_limit_m,
        options.odometry_rate_limit_hz, options.latency_p95_limit_ms,
        options.recovery_limit_s, options.minimum_hover_duration_s,
    )
    if any(value < 0.0 for value in limits):
        raise ValueError("acceptance limits must be non-negative")
    if not 0.0 <= options.visual_availability_limit <= 1.0:
        raise ValueError("--visual-availability-limit must be in [0, 1]")


def _diagnostic_options(
    args: argparse.Namespace, provenance: RunProvenance,
) -> AnalysisOptions:
    values = asdict(AnalysisOptions(
        profile=provenance.profile, scenario=provenance.scenario,
    ))
    for field in fields(AnalysisOptions):
        supplied = getattr(args, field.name)
        if supplied is not None:
            values[field.name] = supplied
    return AnalysisOptions(**values)


def _formal_options(
    args: argparse.Namespace, provenance: RunProvenance,
) -> AnalysisOptions:
    if not isinstance(provenance.analysis_options, dict):
        raise ProvenanceError("verified manifest does not contain analysis_options")
    values = dict(provenance.analysis_options)
    for name in ANALYSIS_OPTION_FIELDS:
        supplied = getattr(args, name)
        if supplied is None or _equal_option(supplied, values[name]):
            continue
        raise ProvenanceError(
            f"--{name.replace('_', '-')} does not match run manifest analysis_options.{name}",
        )
    return AnalysisOptions(**values)


def _equal_option(actual: Any, expected: Any) -> bool:
    if isinstance(actual, bool) or isinstance(expected, bool):
        return actual is expected
    if isinstance(actual, (int, float)) and isinstance(expected, (int, float)):
        return math.isclose(float(actual), float(expected), rel_tol=0.0, abs_tol=1e-12)
    return actual == expected


def _resolve_provenance(args: argparse.Namespace) -> RunProvenance:
    if args.run_manifest is None:
        if args.profile is None or args.scenario is None:
            raise AnalysisUsageError(
                "--profile and --scenario are required without --run-manifest",
            )
        return unverified_provenance(args.profile, args.scenario)
    return verify_run_manifest(
        args.run_manifest,
        args.ulog,
        expected_profile=args.profile,
        expected_scenario=args.scenario,
    )


def _write_metrics(path: Path, metrics: dict[str, Any]) -> None:
    path.mkdir(parents=True, exist_ok=True)
    (path / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8",
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        provenance = _resolve_provenance(args)
        output_dir = args.output_dir.resolve()
        metrics = analyze_ulog(
            args.ulog.resolve(), output_dir, _options(args, provenance),
        )
        apply_provenance_gate(metrics, provenance)
        _write_metrics(output_dir, metrics)
    except AnalysisUsageError as error:
        print(f"analysis usage error: {error}", file=sys.stderr)
        return 2
    except Exception as error:
        print(f"analysis failed: {error}", file=sys.stderr)
        return 1
    print(json.dumps(metrics["acceptance"], indent=2, sort_keys=True))
    return 0 if metrics["acceptance"]["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
