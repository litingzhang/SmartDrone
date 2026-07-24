#!/usr/bin/env python3
"""Tests for SITL artifact provenance and the standalone analyzer gate."""

from __future__ import annotations

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

import analyze_px4_ulog as analyzer  # noqa: E402
from sitl_hover.metrics import AnalysisOptions  # noqa: E402
from sitl_hover.provenance import (  # noqa: E402
    ProvenanceError,
    apply_provenance_gate,
    create_run_manifest,
    main as provenance_main,
    unverified_provenance,
    verify_run_manifest,
    write_launch_attestation,
)


def _events(scenario: str) -> list[dict[str, object]]:
    events: list[dict[str, object]] = [
        {"event": "hover_started"},
        {"event": "hover_finished"},
    ]
    if scenario == "impulse":
        events.insert(1, {"event": "impulse", "applied": True})
    elif scenario == "quality":
        events.insert(1, {"event": "quality_apply", "applied": True})
        events.insert(2, {"event": "quality_clear", "applied": True})
    elif scenario == "loss":
        events.insert(1, {"event": "blackout_short", "applied": True})
        events.insert(2, {"event": "blackout_long", "applied": True})
    return events


def _loss_safety() -> dict[str, bool]:
    return {
        "short_blackout_applied": True,
        "short_blackout_recovered": True,
        "long_blackout_applied": True,
        "tracking_loss_threshold_exceeded": True,
        "autonomous_land_observed": True,
        "autonomous_land_after_long_blackout": True,
        "disarmed_after_autonomous_land": True,
        "premature_autonomous_land_observed": False,
        "unexpected_disarm_observed": False,
        "flight_state_telemetry_lost": False,
        "cleanup_land_attempt_count": 0,
        "cleanup_land_sent": False,
    }


def _write_managed_run(
    root: Path,
    *,
    profile: str = "vision",
    scenario: str = "nominal",
    seed: int = 1,
    outcome: str = "completed",
    events: list[dict[str, object]] | None = None,
    quality: dict[str, float | int] | None = None,
) -> tuple[Path, Path, AnalysisOptions]:
    root.mkdir(parents=True, exist_ok=True)
    run_id = str(uuid.uuid4())
    ulog_path = root / "px4.ulg"
    ulog_path.write_bytes(b"fixture ulog")
    runner_metrics = root / "runner_metrics.json"
    runner_payload = {
        "schema": "smartdrone.sitl.hover.runner_metrics.v1",
        "outcome": outcome,
        "error": None if outcome == "completed" else "fixture failure",
        "events": _events(scenario) if events is None else events,
        "visual_loss_safety": _loss_safety() if scenario == "loss" else {},
    }
    runner_metrics.write_text(json.dumps(runner_payload), encoding="utf-8")
    px4_attestation = root / "px4_attestation.json"
    smart_drone_attestation = root / "smart_drone_attestation.json"
    write_launch_attestation(
        px4_attestation, role="px4", run_id=run_id,
        profile=profile, seed=seed, details={"launcher": "fixture"},
    )
    write_launch_attestation(
        smart_drone_attestation, role="smart_drone", run_id=run_id,
        profile=profile, seed=seed, details={"launcher": "fixture"},
    )
    options = AnalysisOptions(profile=profile, scenario=scenario)
    manifest_path = root / "run_manifest.json"
    create_run_manifest(
        manifest_path,
        run_id=run_id,
        mode="managed",
        profile=profile,
        scenario=scenario,
        seed=seed,
        ulog_path=ulog_path,
        artifacts={
            "runner_metrics": runner_metrics,
            "px4_attestation": px4_attestation,
            "smart_drone_attestation": smart_drone_attestation,
        },
        quality=quality or {},
        analysis_options=asdict(options),
    )
    return manifest_path, ulog_path, options


def _passing_metrics() -> dict[str, object]:
    return {
        "schema": "smartdrone.sitl.hover.metrics.v1",
        "acceptance": {"passed": True, "complete": True, "checks": []},
    }


class ProvenanceContractTest(unittest.TestCase):
    def test_managed_manifest_verifies_and_adds_formal_checks(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            manifest, ulog, _ = _write_managed_run(Path(directory) / "run")
            provenance = verify_run_manifest(manifest, ulog, require_managed=True)
            metrics = apply_provenance_gate(_passing_metrics(), provenance)
        checks = {check["name"]: check for check in metrics["acceptance"]["checks"]}
        self.assertTrue(metrics["acceptance"]["passed"])
        self.assertTrue(checks["run_provenance_verified"]["passed"])
        self.assertTrue(checks["runner_outcome_completed"]["passed"])
        self.assertTrue(checks["scenario_execution_confirmed"]["passed"])

    def test_ulog_change_invalidates_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            manifest, ulog, _ = _write_managed_run(Path(directory) / "run")
            ulog.write_bytes(b"different ulog")
            with self.assertRaisesRegex(ProvenanceError, "px4_ulog.*does not match"):
                verify_run_manifest(manifest, ulog)

    def test_parent_component_is_rejected_even_when_it_resolves_inside_run(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            run_dir = Path(directory) / "run"
            manifest, ulog, _ = _write_managed_run(run_dir)
            payload = json.loads(manifest.read_text())
            payload["artifacts"]["px4_ulog"]["path"] = "../run/px4.ulg"
            manifest.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ProvenanceError, "path must be relative"):
                verify_run_manifest(manifest, ulog)

    def test_attestation_identity_mismatch_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run_id = str(uuid.uuid4())
            ulog = root / "px4.ulg"
            runner = root / "runner_metrics.json"
            px4 = root / "px4.json"
            smart_drone = root / "smart_drone.json"
            ulog.write_bytes(b"ulog")
            runner.write_text(json.dumps({
                "schema": "smartdrone.sitl.hover.runner_metrics.v1",
                "outcome": "completed", "events": _events("nominal"),
            }), encoding="utf-8")
            write_launch_attestation(
                px4, role="px4", run_id=run_id, profile="truth", seed=1,
            )
            write_launch_attestation(
                smart_drone, role="smart_drone", run_id=run_id,
                profile="vision", seed=1,
            )
            with self.assertRaisesRegex(ProvenanceError, "attestation profile mismatch"):
                create_run_manifest(
                    root / "run_manifest.json", run_id=run_id, mode="managed",
                    profile="vision", scenario="nominal", seed=1, ulog_path=ulog,
                    artifacts={
                        "runner_metrics": runner, "px4_attestation": px4,
                        "smart_drone_attestation": smart_drone,
                    },
                    analysis_options=asdict(AnalysisOptions()),
                )

    def test_runner_failure_and_missing_scenario_event_fail_formal_gate(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            manifest, ulog, _ = _write_managed_run(
                Path(directory) / "run", outcome="failed",
                events=[{"event": "hover_started"}, {"event": "hover_finished"},
                        {"event": "impulse", "applied": True}],
            )
            provenance = verify_run_manifest(manifest, ulog)
            metrics = apply_provenance_gate(_passing_metrics(), provenance)
        checks = {check["name"]: check for check in metrics["acceptance"]["checks"]}
        self.assertFalse(metrics["acceptance"]["passed"])
        self.assertFalse(checks["runner_outcome_completed"]["actual"])
        self.assertFalse(checks["scenario_execution_confirmed"]["actual"])

    def test_all_scenario_evidence_contracts_can_pass(self) -> None:
        for scenario in ("nominal", "impulse", "quality", "loss"):
            with self.subTest(scenario=scenario), tempfile.TemporaryDirectory() as directory:
                manifest, ulog, _ = _write_managed_run(
                    Path(directory) / "run", scenario=scenario,
                )
                provenance = verify_run_manifest(manifest, ulog)
                metrics = apply_provenance_gate(_passing_metrics(), provenance)
                scenario_check = next(
                    check for check in metrics["acceptance"]["checks"]
                    if check["name"] == "scenario_execution_confirmed"
                )
                self.assertTrue(scenario_check["passed"])

    def test_loss_requires_complete_safety_outcome(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "run"
            manifest, ulog, _ = _write_managed_run(root, scenario="loss")
            manifest_payload = json.loads(manifest.read_text())
            runner_path = root / "runner_metrics.json"
            runner = json.loads(runner_path.read_text())
            runner["visual_loss_safety"]["disarmed_after_autonomous_land"] = False
            runner_path.write_text(json.dumps(runner), encoding="utf-8")
            create_run_manifest(
                manifest,
                run_id=manifest_payload["run_id"], mode="managed",
                profile="vision", scenario="loss", seed=1, ulog_path=ulog,
                artifacts={
                    "runner_metrics": runner_path,
                    "px4_attestation": root / "px4_attestation.json",
                    "smart_drone_attestation": root / "smart_drone_attestation.json",
                },
                analysis_options=manifest_payload["analysis_options"],
            )
            provenance = verify_run_manifest(manifest, ulog)
            metrics = apply_provenance_gate(_passing_metrics(), provenance)
            scenario_check = next(
                check for check in metrics["acceptance"]["checks"]
                if check["name"] == "scenario_execution_confirmed"
            )
        self.assertFalse(scenario_check["passed"])

    def test_unverified_analysis_is_incomplete(self) -> None:
        metrics = apply_provenance_gate(
            _passing_metrics(), unverified_provenance("vision", "nominal"),
        )
        self.assertFalse(metrics["acceptance"]["complete"])
        self.assertFalse(metrics["acceptance"]["passed"])
        self.assertEqual(metrics["provenance"]["status"], "unverified")

    def test_attached_manifest_remains_unverified(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ulog = root / "px4.ulg"
            ulog.write_bytes(b"fixture")
            manifest = root / "run_manifest.json"
            create_run_manifest(
                manifest, run_id=str(uuid.uuid4()), mode="attached",
                profile="vision", scenario="nominal", seed=1, ulog_path=ulog,
            )
            provenance = verify_run_manifest(manifest, ulog)
            metrics = apply_provenance_gate(_passing_metrics(), provenance)
        self.assertEqual(provenance.status, "unverified")
        self.assertFalse(metrics["acceptance"]["complete"])

    def test_attest_cli_writes_atomic_declaration(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "px4.json"
            run_id = str(uuid.uuid4())
            status = provenance_main([
                "attest", "--output", str(output), "--role", "px4",
                "--run-id", run_id, "--profile", "vision", "--seed", "4",
                "--detail", "world=smartdrone_hover",
            ])
            payload = json.loads(output.read_text())
        self.assertEqual(status, 0)
        self.assertEqual(payload["run_id"], run_id)
        self.assertEqual(payload["details"]["world"], "smartdrone_hover")


class AnalyzerProvenanceTest(unittest.TestCase):
    def test_standalone_requires_explicit_profile_and_scenario(self) -> None:
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            analyzer, "analyze_ulog",
        ) as analyze:
            root = Path(directory)
            ulog = root / "px4.ulg"
            ulog.write_bytes(b"fixture")
            status = analyzer.main([str(ulog), "--output-dir", str(root / "out")])
        self.assertEqual(status, 2)
        analyze.assert_not_called()

    def test_standalone_analysis_is_diagnostic_only(self) -> None:
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            analyzer, "analyze_ulog", return_value=_passing_metrics(),
        ):
            root = Path(directory)
            ulog = root / "px4.ulg"
            output = root / "out"
            ulog.write_bytes(b"fixture")
            status = analyzer.main([
                str(ulog), "--output-dir", str(output),
                "--profile", "vision", "--scenario", "nominal",
            ])
            metrics = json.loads((output / "metrics.json").read_text())
        self.assertEqual(status, 2)
        self.assertFalse(metrics["acceptance"]["complete"])
        self.assertEqual(metrics["provenance"]["status"], "unverified")

    def test_formal_analysis_uses_manifest_options(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "run"
            manifest, ulog, expected_options = _write_managed_run(root)
            with mock.patch.object(
                analyzer, "analyze_ulog", return_value=_passing_metrics(),
            ) as analyze:
                status = analyzer.main([
                    str(ulog), "--output-dir", str(root / "analysis"),
                    "--run-manifest", str(manifest),
                ])
            actual_options = analyze.call_args.args[2]
        self.assertEqual(status, 0)
        self.assertEqual(actual_options, expected_options)

    def test_formal_override_mismatch_is_an_error(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "run"
            manifest, ulog, _ = _write_managed_run(root)
            with mock.patch.object(analyzer, "analyze_ulog") as analyze:
                status = analyzer.main([
                    str(ulog), "--output-dir", str(root / "analysis"),
                    "--run-manifest", str(manifest),
                    "--horizontal-rmse-limit-m", "999",
                ])
        self.assertEqual(status, 1)
        analyze.assert_not_called()

    def test_formal_hash_mismatch_exits_one_before_analysis(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "run"
            manifest, ulog, _ = _write_managed_run(root)
            ulog.write_bytes(b"replaced")
            with mock.patch.object(analyzer, "analyze_ulog") as analyze:
                status = analyzer.main([
                    str(ulog), "--output-dir", str(root / "analysis"),
                    "--run-manifest", str(manifest),
                ])
        self.assertEqual(status, 1)
        analyze.assert_not_called()


if __name__ == "__main__":
    unittest.main()
