#!/usr/bin/env python3
"""Tests for the visual-hover quality matrix orchestrator."""

from __future__ import annotations

import csv
import json
import sys
import tempfile
import unittest
import uuid
from dataclasses import asdict
from pathlib import Path
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import run_hover_matrix as matrix  # noqa: E402
from sitl_hover.metrics import AnalysisOptions  # noqa: E402
from sitl_hover.provenance import (  # noqa: E402
    apply_provenance_gate,
    create_run_manifest,
    verify_run_manifest,
    write_launch_attestation,
)


FAKE_RUNNER = """\
#!/usr/bin/env python3
import argparse
import json
import sys
import uuid
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path.cwd() / "scripts"))
from sitl_hover.metrics import AnalysisOptions
from sitl_hover.provenance import (
    apply_provenance_gate, create_run_manifest, verify_run_manifest,
    write_launch_attestation,
)

parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, required=True)
parser.add_argument("--output-dir", type=Path, required=True)
parser.add_argument("--drop-rate", type=float, required=True)
parser.add_argument("--profile", required=True)
parser.add_argument("--scenario", required=True)
parser.add_argument("--blur-sigma", type=float, required=True)
parser.add_argument("--brightness", type=float, required=True)
parser.add_argument("--noise-stddev", type=float, required=True)
parser.add_argument("--delay-ms", type=int, required=True)
args, _ = parser.parse_known_args()
if args.seed == 1 and args.drop_rate == 0.05:
    raise SystemExit(4)
args.output_dir.mkdir(parents=True, exist_ok=True)
events = [{"event": "hover_started"}, {"event": "hover_finished"}]
if args.scenario == "quality":
    events.insert(1, {"event": "quality_apply", "applied": True})
    events.insert(2, {"event": "quality_clear", "applied": True})
runner = {
    "schema": "smartdrone.sitl.hover.runner_metrics.v1",
    "outcome": "completed", "error": None, "events": events,
    "visual_loss_safety": {},
}
(args.output_dir / "runner_metrics.json").write_text(json.dumps(runner), encoding="utf-8")
(args.output_dir / "px4.ulg").write_bytes(b"fixture ulog")
run_id = str(uuid.uuid4())
write_launch_attestation(
    args.output_dir / "px4_attestation.json", role="px4", run_id=run_id,
    profile=args.profile, seed=args.seed,
)
write_launch_attestation(
    args.output_dir / "smart_drone_attestation.json", role="smart_drone",
    run_id=run_id, profile=args.profile, seed=args.seed,
)
quality = {
    "blur_sigma": args.blur_sigma, "brightness": args.brightness,
    "noise_stddev": args.noise_stddev, "drop_rate": args.drop_rate,
    "delay_ms": args.delay_ms,
}
manifest = args.output_dir / "run_manifest.json"
create_run_manifest(
    manifest, run_id=run_id, mode="managed", profile=args.profile,
    scenario=args.scenario, seed=args.seed, ulog_path=args.output_dir / "px4.ulg",
    artifacts={
        "runner_metrics": args.output_dir / "runner_metrics.json",
        "px4_attestation": args.output_dir / "px4_attestation.json",
        "smart_drone_attestation": args.output_dir / "smart_drone_attestation.json",
    },
    quality=quality,
    analysis_options=asdict(AnalysisOptions(profile=args.profile, scenario=args.scenario)),
)
payload = {
    "profile": args.profile,
    "scenario": args.scenario,
    "hover_accuracy": {
        "horizontal_m": {"rmse": 0.1 + args.drop_rate, "max": 0.2},
        "vertical_m": {"rmse": 0.05, "max": 0.1},
    },
    "visual_odometry": {
        "frequency_hz": 30.0,
        "latency_p95_ms": 20.0,
        "availability": 1.0 - args.drop_rate,
    },
    "acceptance": {"passed": True, "complete": True},
}
apply_provenance_gate(payload, verify_run_manifest(manifest))
(args.output_dir / "metrics.json").write_text(json.dumps(payload), encoding="utf-8")
"""


def _events_for_case(quality_case: matrix.QualityCase) -> list[dict[str, object]]:
    events: list[dict[str, object]] = [
        {"event": "hover_started"}, {"event": "hover_finished"},
    ]
    if quality_case.scenario == "quality":
        events.insert(1, {"event": "quality_apply", "applied": True})
        events.insert(2, {"event": "quality_clear", "applied": True})
    return events


def _write_valid_record(
    run_dir: Path,
    quality_case: matrix.QualityCase,
    seed: int,
    acceptance: dict[str, object],
) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    run_id = str(uuid.uuid4())
    paths = {
        "ulog": run_dir / "px4.ulg",
        "runner": run_dir / "runner_metrics.json",
        "px4": run_dir / "px4_attestation.json",
        "smart_drone": run_dir / "smart_drone_attestation.json",
    }
    paths["ulog"].write_bytes(b"fixture")
    paths["runner"].write_text(json.dumps({
        "schema": "smartdrone.sitl.hover.runner_metrics.v1",
        "outcome": "completed", "error": None,
        "events": _events_for_case(quality_case), "visual_loss_safety": {},
    }), encoding="utf-8")
    for role, key in (("px4", "px4"), ("smart_drone", "smart_drone")):
        write_launch_attestation(
            paths[key], role=role, run_id=run_id, profile="vision", seed=seed,
        )
    manifest = run_dir / "run_manifest.json"
    create_run_manifest(
        manifest, run_id=run_id, mode="managed", profile="vision",
        scenario=quality_case.scenario, seed=seed, ulog_path=paths["ulog"],
        artifacts={
            "runner_metrics": paths["runner"], "px4_attestation": paths["px4"],
            "smart_drone_attestation": paths["smart_drone"],
        },
        quality=matrix._quality_contract(quality_case),
        analysis_options=asdict(AnalysisOptions(
            profile="vision", scenario=quality_case.scenario,
        )),
    )
    payload = {
        "profile": "vision",
        "scenario": quality_case.scenario,
        "hover_accuracy": {
            "horizontal_m": {"rmse": 0.1, "max": 0.2},
            "vertical_m": {"rmse": 0.05, "max": 0.1},
        },
        "visual_odometry": {
            "frequency_hz": 30.0, "latency_p95_ms": 20.0,
            "availability": 1.0,
        },
        "acceptance": acceptance,
    }
    apply_provenance_gate(payload, verify_run_manifest(manifest))
    (run_dir / "metrics.json").write_text(json.dumps(payload), encoding="utf-8")


class MatrixConfigurationTest(unittest.TestCase):
    def test_defaults_cover_required_cases_and_fixed_seeds(self) -> None:
        args = matrix._parse_args([])
        self.assertEqual(args.seeds, [1, 2, 3, 4, 5])
        self.assertEqual(
            args.cases,
            ["nominal", "blur", "dark", "noise", "drop5", "drop10", "delay40", "delay80"],
        )

    def test_runner_command_forwards_launchers_and_single_fault(self) -> None:
        args = matrix._parse_args([
            "--px4-command", "px4 launcher --headless",
            "--smart-drone-command", "smart drone launcher",
            "--sim-config", "sim.yaml", "--gz-partition", "matrix_partition",
        ])
        quality_case = matrix.CASE_BY_NAME["delay80"]
        command = matrix._runner_command(args, quality_case, 3, Path("run"))
        self.assertEqual(command[command.index("--seed") + 1], "3")
        self.assertEqual(command[command.index("--scenario") + 1], "quality")
        self.assertEqual(command[command.index("--delay-ms") + 1], "80")
        self.assertEqual(command[command.index("--drop-rate") + 1], "0.0")
        self.assertEqual(command[command.index("--px4-command") + 1], "px4 launcher --headless")
        self.assertEqual(command[command.index("--smart-drone-command") + 1], "smart drone launcher")
        self.assertEqual(command[command.index("--gz-partition") + 1], "matrix_partition")

    def test_duplicate_seed_is_rejected(self) -> None:
        args = matrix._parse_args(["--seeds", "2", "2"])
        with self.assertRaisesRegex(ValueError, "duplicates"):
            matrix._validate_args(args)

    def test_matrix_requires_fresh_launchers(self) -> None:
        with self.assertRaisesRegex(ValueError, "fresh --px4-command"):
            matrix._validate_args(matrix._parse_args([]))


class MatrixExecutionTest(unittest.TestCase):
    def test_incomplete_acceptance_cannot_make_matrix_pass(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory)
            _write_valid_record(
                run_dir, matrix.CASE_BY_NAME["nominal"], 1,
                {"passed": False, "complete": False, "checks": []},
            )
            record = matrix._record_run(
                matrix.CASE_BY_NAME["nominal"], 1, run_dir, 0, 1.0, None,
            )
        self.assertEqual(record["status"], "acceptance_incomplete")
        self.assertFalse(matrix._summary([record])["overall_passed"])

    def test_failed_acceptance_is_distinct_from_runner_failure(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory)
            _write_valid_record(
                run_dir, matrix.CASE_BY_NAME["nominal"], 1,
                {"passed": False, "complete": True, "checks": [{
                    "name": "fixture_failure", "actual": 1,
                    "limit": 0, "relation": "max", "passed": False,
                }]},
            )
            record = matrix._record_run(
                matrix.CASE_BY_NAME["nominal"], 1, run_dir, 2, 1.0, None,
            )
        self.assertEqual(record["status"], "acceptance_failed")
        self.assertIn("acceptance checks failed", record["error"])

    def test_passing_metrics_without_manifest_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory)
            (run_dir / "metrics.json").write_text(json.dumps({
                "outcome": "completed",
                "acceptance": {"passed": True, "complete": True},
                "provenance": {"status": "verified"},
            }), encoding="utf-8")
            record = matrix._record_run(
                matrix.CASE_BY_NAME["nominal"], 1, run_dir, 0, 1.0, None,
            )
        self.assertEqual(record["status"], "provenance_invalid")
        self.assertIn("run manifest is missing", record["error"])

    def test_quality_case_cannot_reuse_other_case_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory)
            _write_valid_record(
                run_dir, matrix.CASE_BY_NAME["drop5"], 1,
                {"passed": True, "complete": True, "checks": []},
            )
            record = matrix._record_run(
                matrix.CASE_BY_NAME["drop10"], 1, run_dir, 0, 1.0, None,
            )
        self.assertEqual(record["status"], "provenance_invalid")
        self.assertIn("quality.drop_rate mismatch", record["error"])

    def test_metrics_cannot_forge_formal_gate_result(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory)
            _write_valid_record(
                run_dir, matrix.CASE_BY_NAME["nominal"], 1,
                {"passed": True, "complete": True, "checks": []},
            )
            payload = json.loads((run_dir / "metrics.json").read_text())
            gate = next(
                check for check in payload["acceptance"]["checks"]
                if check["name"] == "scenario_execution_confirmed"
            )
            gate["actual"] = False
            (run_dir / "metrics.json").write_text(json.dumps(payload), encoding="utf-8")
            record = matrix._record_run(
                matrix.CASE_BY_NAME["nominal"], 1, run_dir, 0, 1.0, None,
            )
        self.assertEqual(record["status"], "provenance_invalid")
        self.assertIn("actual value is inconsistent", record["error"])

    def test_failed_run_is_aggregated_and_later_runs_continue(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake_runner = root / "fake_runner.py"
            fake_runner.write_text(FAKE_RUNNER, encoding="utf-8")
            output = root / "matrix"
            args = matrix._parse_args([
                "--cases", "drop5", "nominal", "--seeds", "1", "2",
                "--output-dir", str(output),
                "--px4-command", "true", "--smart-drone-command", "true",
            ])
            disabled_plot = {"generated": False, "reason": "test"}
            with mock.patch.object(matrix, "RUNNER_PATH", fake_runner), mock.patch.object(
                matrix, "_try_plot", return_value=disabled_plot,
            ):
                result = matrix.run_matrix(args)

            self.assertEqual(result, 1)
            report = json.loads((output / "quality_matrix.json").read_text())
            self.assertEqual(report["summary"]["planned_runs"], 4)
            self.assertEqual(report["summary"]["failed_runs"], 1)
            self.assertFalse(report["summary"]["overall_passed"])
            self.assertEqual(len(report["runs"]), 4)
            failed = next(
                record for record in report["runs"]
                if record["case"] == "drop5" and record["seed"] == 1
            )
            self.assertEqual(failed["status"], "failed")
            self.assertEqual(failed["return_code"], 4)
            self.assertIn("metrics.json is missing", failed["error"])
            final_run = next(
                record for record in report["runs"]
                if record["case"] == "nominal" and record["seed"] == 2
            )
            self.assertEqual(final_run["status"], "completed")
            self.assertAlmostEqual(final_run["horizontal_rmse_m"], 0.1)
            with (output / "quality_matrix.csv").open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 4)
            self.assertTrue((output / "nominal" / "seed_002" / "matrix_invocation.json").is_file())


if __name__ == "__main__":
    unittest.main()
