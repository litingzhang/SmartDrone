#!/usr/bin/env python3
"""Run the repeatable SmartDrone visual-hover quality matrix."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

from sitl_hover.provenance import (
    ProvenanceError,
    RunProvenance,
    apply_provenance_gate,
    verify_run_manifest,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = REPO_ROOT / "scripts" / "run_hover_sitl.py"
DEFAULT_SEEDS = (1, 2, 3, 4, 5)


@dataclass(frozen=True)
class QualityCase:
    name: str
    scenario: str
    blur_sigma: float = 0.0
    brightness: float = 1.0
    noise_stddev: float = 0.0
    drop_rate: float = 0.0
    delay_ms: int = 0


QUALITY_CASES = (
    QualityCase("nominal", "nominal"),
    QualityCase("blur", "quality", blur_sigma=1.5),
    QualityCase("dark", "quality", brightness=0.65),
    QualityCase("noise", "quality", noise_stddev=3.0),
    QualityCase("drop5", "quality", drop_rate=0.05),
    QualityCase("drop10", "quality", drop_rate=0.10),
    QualityCase("delay40", "quality", delay_ms=40),
    QualityCase("delay80", "quality", delay_ms=80),
)
CASE_BY_NAME = {quality_case.name: quality_case for quality_case in QUALITY_CASES}
CSV_FIELDS = (
    "case", "seed", "scenario", "status", "return_code", "duration_s",
    "acceptance_passed", "acceptance_complete", "outcome", "error",
    "run_id", "provenance_status",
    "blur_sigma", "brightness", "noise_stddev", "drop_rate", "delay_ms",
    "horizontal_rmse_m", "horizontal_max_m", "vertical_rmse_m",
    "vertical_max_m", "odometry_frequency_hz", "latency_p95_ms",
    "visual_availability", "run_dir", "metrics_path", "runner_log",
)


def _default_output() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return REPO_ROOT / "output" / "sitl_matrix" / stamp


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run visual hover across image-quality cases and fixed seeds.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--seeds", nargs="+", type=int, default=list(DEFAULT_SEEDS))
    parser.add_argument(
        "--cases", nargs="+", choices=tuple(CASE_BY_NAME),
        default=[quality_case.name for quality_case in QUALITY_CASES],
    )
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--px4-command", default="")
    parser.add_argument("--smart-drone-command", default="")
    parser.add_argument("--sim-config", type=Path)
    parser.add_argument(
        "--gz-partition",
        default=os.environ.get("SMART_DRONE_GZ_PARTITION", os.environ.get("GZ_PARTITION", "smartdrone_sitl")),
    )
    parser.add_argument("--command-host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=14550)
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--bind-port", type=int, default=0)
    parser.add_argument(
        "--sim-watchdog-scale", type=float,
        default=os.environ.get("SMART_DRONE_SIM_WATCHDOG_SCALE", "1.0"),
    )
    return parser.parse_args(argv)


def _validate_args(args: argparse.Namespace) -> None:
    if any(seed <= 0 for seed in args.seeds):
        raise ValueError("--seeds values must be positive integers")
    if len(set(args.seeds)) != len(args.seeds):
        raise ValueError("--seeds contains duplicates")
    if len(set(args.cases)) != len(args.cases):
        raise ValueError("--cases contains duplicates")
    if not 1 <= args.command_port <= 65535 or not 0 <= args.bind_port <= 65535:
        raise ValueError("UDP ports are outside the valid range")
    if not math.isfinite(args.sim_watchdog_scale) or not 1.0 <= args.sim_watchdog_scale <= 20.0:
        raise ValueError("simulation watchdog scale must be in [1, 20]")
    if not args.px4_command or not args.smart_drone_command:
        raise ValueError("the quality matrix requires fresh --px4-command and --smart-drone-command launchers")


def _prepare_output(output_dir: Path) -> None:
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError(f"output directory is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)


def _runner_command(
    args: argparse.Namespace,
    quality_case: QualityCase,
    seed: int,
    run_dir: Path,
) -> list[str]:
    command = [
        sys.executable, str(RUNNER_PATH),
        "--profile", "vision", "--scenario", quality_case.scenario,
        "--seed", str(seed), "--output-dir", str(run_dir),
        "--gz-partition", args.gz_partition,
        "--command-host", args.command_host, "--command-port", str(args.command_port),
        "--bind-host", args.bind_host, "--bind-port", str(args.bind_port),
        "--sim-watchdog-scale", str(args.sim_watchdog_scale),
        "--blur-sigma", str(quality_case.blur_sigma),
        "--brightness", str(quality_case.brightness),
        "--noise-stddev", str(quality_case.noise_stddev),
        "--drop-rate", str(quality_case.drop_rate),
        "--delay-ms", str(quality_case.delay_ms),
    ]
    if args.px4_command:
        command.extend(("--px4-command", args.px4_command))
    if args.smart_drone_command:
        command.extend(("--smart-drone-command", args.smart_drone_command))
    if args.sim_config is not None:
        command.extend(("--sim-config", str(args.sim_config)))
    return command


def _write_invocation(
    run_dir: Path,
    quality_case: QualityCase,
    seed: int,
    command: Sequence[str],
) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema": "smartdrone.sitl.quality_matrix.run.v1",
        "case": asdict(quality_case),
        "seed": seed,
        "command": list(command),
    }
    path = run_dir / "matrix_invocation.json"
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _execute_run(command: Sequence[str], run_dir: Path) -> tuple[int, float, str | None]:
    started_s = time.monotonic()
    log_path = run_dir / "matrix_runner.log"
    try:
        with log_path.open("w", encoding="utf-8") as log:
            result = subprocess.run(
                command, cwd=REPO_ROOT, stdout=log, stderr=subprocess.STDOUT,
                text=True, check=False,
            )
        return result.returncode, time.monotonic() - started_s, None
    except OSError as error:
        message = f"{type(error).__name__}: {error}"
        log_path.write_text(message + "\n", encoding="utf-8")
        return 127, time.monotonic() - started_s, message


def _load_metrics(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    if not path.is_file():
        return None, "metrics.json is missing"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return None, f"invalid metrics.json: {error}"
    if not isinstance(payload, dict):
        return None, "metrics.json root is not an object"
    return payload, None


def _nested(payload: dict[str, Any], *keys: str) -> Any:
    value: Any = payload
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _metric(payload: dict[str, Any], *keys: str) -> float | None:
    value = _nested(payload, *keys)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _run_outcome(metrics: dict[str, Any]) -> tuple[Any, Any]:
    runner = metrics.get("runner")
    runner_metrics = runner if isinstance(runner, dict) else metrics
    return runner_metrics.get("outcome"), runner_metrics.get("error")


def _quality_contract(quality_case: QualityCase) -> dict[str, float | int]:
    return {
        "blur_sigma": quality_case.blur_sigma,
        "brightness": quality_case.brightness,
        "noise_stddev": quality_case.noise_stddev,
        "drop_rate": quality_case.drop_rate,
        "delay_ms": quality_case.delay_ms,
    }


def _verify_metrics_provenance(
    metrics: dict[str, Any], provenance: RunProvenance,
) -> None:
    if metrics.get("profile") != provenance.profile:
        raise ProvenanceError("metrics profile does not match run manifest")
    if metrics.get("scenario") != provenance.scenario:
        raise ProvenanceError("metrics scenario does not match run manifest")
    recorded = metrics.get("provenance")
    if not isinstance(recorded, dict):
        raise ProvenanceError("metrics.json does not contain provenance")
    expected = provenance.to_metrics()
    for name in ("status", "mode", "run_id", "profile", "scenario", "seed"):
        if recorded.get(name) != expected[name]:
            raise ProvenanceError(f"metrics provenance {name} does not match run manifest")
    recorded_ulog = recorded.get("ulog")
    if not isinstance(recorded_ulog, dict):
        raise ProvenanceError("metrics provenance ULog descriptor is missing")
    if recorded_ulog.get("sha256") != provenance.ulog_sha256:
        raise ProvenanceError("metrics provenance ULog SHA-256 does not match run manifest")
    _verify_provenance_checks(metrics, provenance)


def _verify_provenance_checks(
    metrics: dict[str, Any], provenance: RunProvenance,
) -> None:
    acceptance = metrics.get("acceptance")
    checks = acceptance.get("checks") if isinstance(acceptance, dict) else None
    if not isinstance(checks, list):
        raise ProvenanceError("metrics acceptance checks are missing")
    by_name = {
        check.get("name"): check
        for check in checks
        if isinstance(check, dict) and isinstance(check.get("name"), str)
    }
    expected_metrics = {
        "acceptance": {"passed": True, "complete": True, "checks": []},
    }
    apply_provenance_gate(expected_metrics, provenance)
    expected = {
        check["name"]: check
        for check in expected_metrics["acceptance"]["checks"]
    }
    for name in (
        "run_provenance_verified", "runner_outcome_completed",
        "scenario_execution_confirmed",
    ):
        check = by_name.get(name)
        if not isinstance(check, dict):
            raise ProvenanceError(f"metrics formal gate {name} is missing")
        if check.get("actual") != expected[name]["actual"]:
            raise ProvenanceError(f"metrics formal gate {name} actual value is inconsistent")
        if check.get("passed") != expected[name]["passed"]:
            raise ProvenanceError(f"metrics formal gate {name} pass value is inconsistent")


def _record_provenance(
    quality_case: QualityCase,
    seed: int,
    run_dir: Path,
    metrics: dict[str, Any],
) -> tuple[RunProvenance | None, str | None]:
    try:
        provenance = verify_run_manifest(
            run_dir / "run_manifest.json",
            expected_profile="vision",
            expected_scenario=quality_case.scenario,
            expected_seed=seed,
            expected_quality=_quality_contract(quality_case),
            require_managed=True,
        )
        _verify_metrics_provenance(metrics, provenance)
        return provenance, None
    except (OSError, ProvenanceError) as error:
        return None, str(error)


def _record_run(
    quality_case: QualityCase,
    seed: int,
    run_dir: Path,
    return_code: int,
    duration_s: float,
    execution_error: str | None,
) -> dict[str, Any]:
    metrics_path = run_dir / "metrics.json"
    metrics, metrics_error = _load_metrics(metrics_path)
    payload = metrics or {}
    acceptance = payload.get("acceptance") if isinstance(payload.get("acceptance"), dict) else {}
    outcome, runner_error = _run_outcome(payload)
    provenance, provenance_error = (None, None)
    if metrics_error is None:
        provenance, provenance_error = _record_provenance(
            quality_case, seed, run_dir, payload,
        )
    status = "completed"
    if execution_error is not None or (return_code != 0 and metrics_error is not None):
        status = "failed"
    elif metrics_error is not None:
        status = "metrics_missing"
    elif provenance_error is not None:
        status = "provenance_invalid"
    elif outcome != "completed":
        status = "failed"
    elif acceptance.get("complete") is not True:
        status = "acceptance_incomplete"
    elif acceptance.get("passed") is not True:
        status = "acceptance_failed"
    elif return_code != 0:
        status = "failed"
    error = execution_error or metrics_error or provenance_error or runner_error
    if error is None and status == "acceptance_incomplete":
        error = "formal acceptance is incomplete"
    elif error is None and status == "acceptance_failed":
        error = "one or more formal acceptance checks failed"
    elif error is None and status == "failed":
        error = f"runner exited with status {return_code}"
    return {
        "case": quality_case.name, "seed": seed, "scenario": quality_case.scenario,
        "status": status, "return_code": return_code, "duration_s": round(duration_s, 3),
        "acceptance_passed": acceptance.get("passed"),
        "acceptance_complete": acceptance.get("complete"),
        "outcome": outcome, "error": error,
        "run_id": provenance.run_id if provenance is not None else None,
        "provenance_status": provenance.status if provenance is not None else "invalid",
        "blur_sigma": quality_case.blur_sigma, "brightness": quality_case.brightness,
        "noise_stddev": quality_case.noise_stddev, "drop_rate": quality_case.drop_rate,
        "delay_ms": quality_case.delay_ms,
        "horizontal_rmse_m": _metric(payload, "hover_accuracy", "horizontal_m", "rmse"),
        "horizontal_max_m": _metric(payload, "hover_accuracy", "horizontal_m", "max"),
        "vertical_rmse_m": _metric(payload, "hover_accuracy", "vertical_m", "rmse"),
        "vertical_max_m": _metric(payload, "hover_accuracy", "vertical_m", "max"),
        "odometry_frequency_hz": _metric(payload, "visual_odometry", "frequency_hz"),
        "latency_p95_ms": _metric(payload, "visual_odometry", "latency_p95_ms"),
        "visual_availability": _metric(payload, "visual_odometry", "availability"),
        "run_dir": str(run_dir), "metrics_path": str(metrics_path),
        "runner_log": str(run_dir / "matrix_runner.log"),
    }


def _write_csv(path: Path, records: Sequence[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(records)


def _plot_axis(axis: Any, records: Sequence[dict[str, Any]], cases: Sequence[str], field: str, label: str) -> None:
    means: list[float] = []
    mean_positions: list[int] = []
    for position, case_name in enumerate(cases):
        values = [record[field] for record in records if record["case"] == case_name and record[field] is not None]
        axis.scatter([position] * len(values), values, color="#167d8d", alpha=0.75)
        if values:
            mean_positions.append(position)
            means.append(sum(values) / len(values))
    axis.plot(mean_positions, means, color="#cf4d32", marker="o", label="seed mean")
    axis.set_xticks(range(len(cases)), cases, rotation=30, ha="right")
    axis.set_ylabel(label)
    axis.grid(axis="y", alpha=0.25)
    if means:
        axis.legend()


def _try_plot(
    output_dir: Path,
    records: Sequence[dict[str, Any]],
    cases: Sequence[str],
) -> dict[str, Any]:
    try:
        import matplotlib  # type: ignore

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # type: ignore
    except ImportError:
        return {"generated": False, "reason": "matplotlib is not installed"}
    try:
        figure, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        _plot_axis(axes[0], records, cases, "horizontal_rmse_m", "horizontal RMSE (m)")
        _plot_axis(axes[1], records, cases, "vertical_rmse_m", "vertical RMSE (m)")
        figure.suptitle("SmartDrone visual-hover quality matrix")
        figure.tight_layout()
        plot_path = output_dir / "quality_matrix.png"
        figure.savefig(plot_path, dpi=140)
        plt.close(figure)
        return {"generated": True, "path": plot_path.name}
    except Exception as error:
        return {"generated": False, "reason": f"plot failed: {error}"}


def _summary(records: Sequence[dict[str, Any]]) -> dict[str, Any]:
    failed = sum(record["status"] != "completed" for record in records)
    accepted = sum(record["acceptance_passed"] is True for record in records)
    complete = sum(record["acceptance_complete"] is True for record in records)
    return {
        "planned_runs": len(records), "completed_runs": len(records) - failed,
        "failed_runs": failed, "acceptance_complete_runs": complete,
        "acceptance_passed_runs": accepted,
        "overall_passed": bool(records) and failed == 0 and complete == len(records) and accepted == len(records),
    }


def run_matrix(args: argparse.Namespace) -> int:
    _validate_args(args)
    output_dir = (args.output_dir or _default_output()).resolve()
    _prepare_output(output_dir)
    selected_cases = [CASE_BY_NAME[name] for name in args.cases]
    records: list[dict[str, Any]] = []
    total = len(selected_cases) * len(args.seeds)
    for quality_case in selected_cases:
        for seed in args.seeds:
            run_dir = output_dir / quality_case.name / f"seed_{seed:03d}"
            command = _runner_command(args, quality_case, seed, run_dir)
            _write_invocation(run_dir, quality_case, seed, command)
            print(f"[{len(records) + 1}/{total}] case={quality_case.name} seed={seed}", flush=True)
            return_code, duration_s, execution_error = _execute_run(command, run_dir)
            records.append(_record_run(
                quality_case, seed, run_dir, return_code, duration_s, execution_error,
            ))
    _write_csv(output_dir / "quality_matrix.csv", records)
    case_names = [quality_case.name for quality_case in selected_cases]
    plot = _try_plot(output_dir, records, case_names)
    summary = _summary(records)
    report = {
        "schema": "smartdrone.sitl.quality_matrix.v2",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "profile": "vision", "seeds": list(args.seeds),
        "cases": [asdict(quality_case) for quality_case in selected_cases],
        "summary": summary, "plot": plot, "runs": records,
    }
    (output_dir / "quality_matrix.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8",
    )
    print(f"matrix artifacts: {output_dir}")
    return 0 if summary["overall_passed"] else 1


def main(argv: Sequence[str] | None = None) -> int:
    try:
        return run_matrix(_parse_args(argv))
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
