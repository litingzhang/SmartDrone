"""Artifact provenance for formal PX4/Gazebo SITL acceptance."""

from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import math
import os
import re
import sys
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping


RUN_MANIFEST_SCHEMA = "smartdrone.sitl.run_manifest.v1"
LAUNCH_ATTESTATION_SCHEMA = "smartdrone.sitl.launch_attestation.v1"
SUPPORTED_MODES = frozenset(("managed", "attached", "external"))
SUPPORTED_PROFILES = frozenset(("control", "truth", "vision"))
SUPPORTED_SCENARIOS = frozenset(("nominal", "impulse", "quality", "loss"))
SUPPORTED_ATTESTATION_ROLES = frozenset(("px4", "smart_drone"))
SUPPORTED_RUNNER_METRICS_SCHEMAS = frozenset((
    "smartdrone.sitl.hover.metrics.v1",
    "smartdrone.sitl.hover.runner_metrics.v1",
))
MANAGED_ATTESTATIONS = {
    "px4_attestation": "px4",
    "smart_drone_attestation": "smart_drone",
}
MANAGED_REQUIRED_ARTIFACTS = frozenset((*MANAGED_ATTESTATIONS, "runner_metrics"))
ANALYSIS_OPTION_FIELDS = (
    "profile", "scenario", "target_x", "target_y", "target_z",
    "hover_start_s", "hover_end_s", "disturbance_time_s",
    "horizontal_rmse_limit_m", "horizontal_max_limit_m",
    "vertical_rmse_limit_m", "vertical_max_limit_m",
    "odometry_rate_limit_hz", "latency_p95_limit_ms",
    "visual_availability_limit", "recovery_limit_s",
    "minimum_hover_duration_s",
)
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


class ProvenanceError(ValueError):
    """Raised when a supplied run manifest is malformed or does not match."""


@dataclass(frozen=True)
class RunProvenance:
    status: str
    mode: str
    run_id: str | None
    profile: str
    scenario: str
    seed: int | None
    manifest_path: Path | None
    ulog_path: Path | None
    ulog_sha256: str | None
    issues: tuple[str, ...]
    manifest: Mapping[str, Any] | None = None
    analysis_options: Mapping[str, Any] | None = None
    runner_metrics: Mapping[str, Any] | None = None

    @property
    def verified(self) -> bool:
        return self.status == "verified"

    def to_metrics(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "mode": self.mode,
            "run_id": self.run_id,
            "profile": self.profile,
            "scenario": self.scenario,
            "seed": self.seed,
            "manifest": str(self.manifest_path) if self.manifest_path is not None else None,
            "ulog": {
                "path": str(self.ulog_path) if self.ulog_path is not None else None,
                "sha256": self.ulog_sha256,
            },
            "issues": list(self.issues),
        }


def unverified_provenance(
    profile: str,
    scenario: str,
    reason: str = "run_manifest_not_supplied",
) -> RunProvenance:
    _validate_profile_scenario(profile, scenario)
    return RunProvenance(
        status="unverified",
        mode="external",
        run_id=None,
        profile=profile,
        scenario=scenario,
        seed=None,
        manifest_path=None,
        ulog_path=None,
        ulog_sha256=None,
        issues=(reason,),
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_launch_attestation(
    path: Path,
    *,
    role: str,
    run_id: str,
    profile: str,
    seed: int,
    details: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Atomically record the launcher's effective run identity."""
    _validate_attestation_identity(role, run_id, profile, seed)
    if details is not None and not isinstance(details, Mapping):
        raise ProvenanceError("attestation details must be an object")
    payload = {
        "schema": LAUNCH_ATTESTATION_SCHEMA,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "role": role,
        "run_id": run_id,
        "profile": profile,
        "seed": seed,
        "details": dict(details or {}),
    }
    _write_json_atomic(Path(path), payload)
    return payload


def create_run_manifest(
    manifest_path: Path,
    *,
    run_id: str,
    mode: str,
    profile: str,
    scenario: str,
    seed: int,
    ulog_path: Path,
    artifacts: Mapping[str, Path] | None = None,
    flight: Mapping[str, Any] | None = None,
    quality: Mapping[str, Any] | None = None,
    runner: Mapping[str, Any] | None = None,
    analysis_options: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Write an immutable manifest whose artifact paths are relative to it."""
    manifest_path = Path(manifest_path).resolve()
    _validate_manifest_identity(run_id, mode, profile, scenario, seed)
    artifact_paths = dict(artifacts or {})
    artifact_paths["px4_ulog"] = Path(ulog_path)
    if mode == "managed":
        missing = sorted(set(MANAGED_REQUIRED_ARTIFACTS) - set(artifact_paths))
        if missing:
            raise ProvenanceError(
                f"managed manifest is missing required artifact: {missing[0]}",
            )
    normalized_options = _validate_analysis_options(
        analysis_options, profile=profile, scenario=scenario, required=mode == "managed",
    )
    descriptors = {
        name: _describe_artifact(manifest_path.parent, path)
        for name, path in sorted(artifact_paths.items())
    }
    payload: dict[str, Any] = {
        "schema": RUN_MANIFEST_SCHEMA,
        "run_id": run_id,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "mode": mode,
        "profile": profile,
        "scenario": scenario,
        "seed": seed,
        "flight": dict(flight or {}),
        "quality": dict(quality or {}),
        "runner": dict(runner or {}),
        "analysis_options": normalized_options,
        "artifacts": descriptors,
    }
    if mode == "managed":
        _verify_attestations(
            manifest_path.parent,
            descriptors,
            run_id=run_id,
            profile=profile,
            seed=seed,
        )
        _load_runner_metrics(Path(artifact_paths["runner_metrics"]).resolve(), scenario)
    _write_json_atomic(manifest_path, payload)
    return payload


def verify_run_manifest(
    manifest_path: Path,
    ulog_path: Path | None = None,
    *,
    expected_profile: str | None = None,
    expected_scenario: str | None = None,
    expected_seed: int | None = None,
    expected_quality: Mapping[str, Any] | None = None,
    require_managed: bool = False,
) -> RunProvenance:
    manifest_path = Path(manifest_path).resolve()
    payload = _load_manifest(manifest_path)
    run_id, mode, profile, scenario, seed = _manifest_identity(payload)
    _check_expected("profile", profile, expected_profile)
    _check_expected("scenario", scenario, expected_scenario)
    _check_expected("seed", seed, expected_seed)
    if expected_quality is not None:
        _verify_quality(payload.get("quality"), expected_quality)
    analysis_options = _validate_analysis_options(
        payload.get("analysis_options"),
        profile=profile,
        scenario=scenario,
        required=mode == "managed",
    )
    resolved_artifacts = _verify_artifacts(manifest_path.parent, payload.get("artifacts"))
    manifest_ulog = resolved_artifacts.get("px4_ulog")
    if manifest_ulog is None:
        raise ProvenanceError("run manifest does not declare artifacts.px4_ulog")
    if ulog_path is not None and Path(ulog_path).resolve() != manifest_ulog:
        raise ProvenanceError("ULog path does not match the run manifest")
    descriptor = payload["artifacts"]["px4_ulog"]
    runner_metrics = None
    if mode == "managed":
        _verify_attestations(
            manifest_path.parent,
            payload["artifacts"],
            run_id=run_id,
            profile=profile,
            seed=seed,
        )
        runner_path = resolved_artifacts.get("runner_metrics")
        if runner_path is None:
            raise ProvenanceError("managed manifest is missing required artifact: runner_metrics")
        runner_metrics = _load_runner_metrics(runner_path, scenario)
    issues = () if mode == "managed" else (f"run_mode_{mode}_is_not_formal",)
    if require_managed and mode != "managed":
        raise ProvenanceError(f"formal acceptance requires managed mode, got {mode}")
    return RunProvenance(
        status="verified" if mode == "managed" else "unverified",
        mode=mode,
        run_id=run_id,
        profile=profile,
        scenario=scenario,
        seed=seed,
        manifest_path=manifest_path,
        ulog_path=manifest_ulog,
        ulog_sha256=descriptor["sha256"],
        issues=issues,
        manifest=payload,
        analysis_options=analysis_options,
        runner_metrics=runner_metrics,
    )


def apply_provenance_gate(
    metrics: dict[str, Any], provenance: RunProvenance,
) -> dict[str, Any]:
    """Attach provenance and make formal acceptance fail closed."""
    acceptance = metrics.get("acceptance")
    if not isinstance(acceptance, dict):
        raise ProvenanceError("metrics acceptance is missing or invalid")
    checks = acceptance.get("checks")
    if not isinstance(checks, list):
        checks = []
        acceptance["checks"] = checks
    checks[:] = [
        check for check in checks
        if not isinstance(check, dict) or check.get("name") not in {
            "run_provenance_verified", "runner_outcome_completed",
            "scenario_execution_confirmed",
        }
    ]
    actual = True if provenance.verified else None
    checks.append({
        "name": "run_provenance_verified",
        "actual": actual,
        "relation": "required",
        "limit": True,
        "passed": provenance.verified,
    })
    if provenance.verified:
        runner_metrics = provenance.runner_metrics
        runner_complete = isinstance(runner_metrics, Mapping)
        runner_passed = runner_complete and runner_metrics.get("outcome") == "completed"
        checks.append({
            "name": "runner_outcome_completed",
            "actual": runner_passed if runner_complete else None,
            "relation": "required",
            "limit": True,
            "passed": runner_passed,
        })
        scenario_confirmed = _scenario_execution_confirmed(
            provenance.scenario, runner_metrics,
        ) if runner_complete else None
        checks.append({
            "name": "scenario_execution_confirmed",
            "actual": scenario_confirmed,
            "relation": "required",
            "limit": True,
            "passed": scenario_confirmed is True,
        })
        metrics["runner"] = dict(runner_metrics) if runner_complete else None
    base_complete = acceptance.get("complete") is True
    gate_complete = all(
        check.get("actual") is not None
        for check in checks
        if isinstance(check, dict) and check.get("name") in {
            "run_provenance_verified", "runner_outcome_completed",
            "scenario_execution_confirmed",
        }
    )
    acceptance["complete"] = base_complete and provenance.verified and gate_complete
    acceptance["passed"] = (
        acceptance["complete"]
        and all(isinstance(check, dict) and check.get("passed") is True for check in checks)
    )
    if not provenance.verified:
        acceptance["reason"] = "formal acceptance requires verified managed run provenance"
    else:
        acceptance.pop("reason", None)
    metrics["provenance"] = provenance.to_metrics()
    return metrics


def _validate_profile_scenario(profile: str, scenario: str) -> None:
    if profile not in SUPPORTED_PROFILES:
        raise ProvenanceError(f"unsupported run profile: {profile}")
    if scenario not in SUPPORTED_SCENARIOS:
        raise ProvenanceError(f"unsupported run scenario: {scenario}")
    if scenario == "loss" and profile != "vision":
        raise ProvenanceError("loss scenario requires the vision profile")


def _validate_analysis_options(
    value: Any,
    *,
    profile: str,
    scenario: str,
    required: bool,
) -> dict[str, Any]:
    if (value is None or value == {}) and not required:
        return {}
    if not isinstance(value, Mapping):
        raise ProvenanceError("managed manifest analysis_options must be an object")
    missing = [name for name in ANALYSIS_OPTION_FIELDS if name not in value]
    unexpected = sorted(set(value) - set(ANALYSIS_OPTION_FIELDS))
    if missing:
        raise ProvenanceError(f"analysis_options is missing {missing[0]}")
    if unexpected:
        raise ProvenanceError(f"analysis_options contains unsupported field {unexpected[0]}")
    normalized = dict(value)
    _check_expected("analysis profile", normalized["profile"], profile)
    _check_expected("analysis scenario", normalized["scenario"], scenario)
    _validate_analysis_numbers(normalized)
    return normalized


def _validate_analysis_numbers(options: Mapping[str, Any]) -> None:
    optional = {"hover_start_s", "hover_end_s", "disturbance_time_s"}
    for name in ANALYSIS_OPTION_FIELDS[2:]:
        value = options[name]
        if value is None and name in optional:
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProvenanceError(f"analysis_options.{name} must be numeric")
        if not math.isfinite(float(value)):
            raise ProvenanceError(f"analysis_options.{name} must be finite")
    if (options["hover_start_s"] is None) != (options["hover_end_s"] is None):
        raise ProvenanceError("analysis hover start and end must be supplied together")
    if (
        options["hover_start_s"] is not None
        and float(options["hover_end_s"]) < float(options["hover_start_s"])
    ):
        raise ProvenanceError("analysis hover end must not precede hover start")
    non_negative = ANALYSIS_OPTION_FIELDS[8:]
    if any(float(options[name]) < 0.0 for name in non_negative):
        raise ProvenanceError("analysis acceptance limits must be non-negative")
    availability = float(options["visual_availability_limit"])
    if not 0.0 <= availability <= 1.0:
        raise ProvenanceError("analysis visual availability limit must be in [0, 1]")


def _load_runner_metrics(path: Path, scenario: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ProvenanceError(f"invalid runner metrics {path}: {error}") from error
    if not isinstance(payload, dict):
        raise ProvenanceError("runner metrics root must be an object")
    if payload.get("schema") not in SUPPORTED_RUNNER_METRICS_SCHEMAS:
        raise ProvenanceError(f"unsupported runner metrics schema: {payload.get('schema')}")
    if not isinstance(payload.get("outcome"), str):
        raise ProvenanceError("runner metrics outcome is missing or invalid")
    if not isinstance(payload.get("events"), list):
        raise ProvenanceError("runner metrics events are missing or invalid")
    if scenario == "loss" and not isinstance(payload.get("visual_loss_safety"), dict):
        raise ProvenanceError("loss runner metrics visual_loss_safety is missing or invalid")
    return payload


def _scenario_execution_confirmed(
    scenario: str, runner_metrics: Mapping[str, Any],
) -> bool:
    events = runner_metrics.get("events")
    if not isinstance(events, list):
        return False
    event_records = [event for event in events if isinstance(event, dict)]
    if not _has_single_event(event_records, "hover_started"):
        return False
    if not _has_single_event(event_records, "hover_finished"):
        return False
    if scenario == "nominal":
        fault_names = {"impulse", "quality_apply", "quality_clear", "blackout_short", "blackout_long"}
        return not any(event.get("event") in fault_names for event in event_records)
    if scenario == "impulse":
        return _has_single_applied_event(event_records, "impulse")
    if scenario == "quality":
        return (
            _has_single_applied_event(event_records, "quality_apply")
            and _has_single_applied_event(event_records, "quality_clear")
        )
    if scenario == "loss":
        return (
            _has_single_applied_event(event_records, "blackout_short")
            and _has_single_applied_event(event_records, "blackout_long")
            and _loss_safety_confirmed(runner_metrics.get("visual_loss_safety"))
        )
    return False


def _has_single_event(events: list[dict[str, Any]], name: str) -> bool:
    return sum(event.get("event") == name for event in events) == 1


def _has_single_applied_event(events: list[dict[str, Any]], name: str) -> bool:
    matching = [event for event in events if event.get("event") == name]
    return len(matching) == 1 and matching[0].get("applied") is True


def _loss_safety_confirmed(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    required_true = (
        "short_blackout_applied", "short_blackout_recovered",
        "long_blackout_applied", "tracking_loss_threshold_exceeded",
        "autonomous_land_observed",
        "autonomous_land_after_long_blackout",
        "disarmed_after_autonomous_land",
    )
    required_false = (
        "premature_autonomous_land_observed", "unexpected_disarm_observed",
        "flight_state_telemetry_lost", "cleanup_land_sent",
    )
    return (
        all(value.get(name) is True for name in required_true)
        and all(value.get(name) is False for name in required_false)
        and value.get("cleanup_land_attempt_count") == 0
    )


def _validate_manifest_identity(
    run_id: str, mode: str, profile: str, scenario: str, seed: int,
) -> None:
    try:
        uuid.UUID(run_id)
    except (AttributeError, TypeError, ValueError) as error:
        raise ProvenanceError("run_id must be a UUID") from error
    if mode not in SUPPORTED_MODES:
        raise ProvenanceError(f"unsupported run mode: {mode}")
    _validate_profile_scenario(profile, scenario)
    if isinstance(seed, bool) or not isinstance(seed, int) or seed <= 0:
        raise ProvenanceError("seed must be a positive integer")


def _validate_attestation_identity(
    role: str, run_id: str, profile: str, seed: int,
) -> None:
    if role not in SUPPORTED_ATTESTATION_ROLES:
        raise ProvenanceError(f"unsupported attestation role: {role}")
    _validate_manifest_identity(run_id, "managed", profile, "nominal", seed)


def _manifest_identity(payload: Mapping[str, Any]) -> tuple[str, str, str, str, int]:
    run_id = payload.get("run_id")
    mode = payload.get("mode")
    profile = payload.get("profile")
    scenario = payload.get("scenario")
    seed = payload.get("seed")
    _validate_manifest_identity(run_id, mode, profile, scenario, seed)
    return run_id, mode, profile, scenario, seed


def _load_manifest(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ProvenanceError(f"run manifest is missing: {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ProvenanceError(f"invalid run manifest: {error}") from error
    if not isinstance(payload, dict):
        raise ProvenanceError("run manifest root must be an object")
    if payload.get("schema") != RUN_MANIFEST_SCHEMA:
        raise ProvenanceError(f"unsupported run manifest schema: {payload.get('schema')}")
    return payload


def _load_attestation(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ProvenanceError(f"invalid launch attestation {path}: {error}") from error
    if not isinstance(payload, dict):
        raise ProvenanceError(f"launch attestation root must be an object: {path}")
    if payload.get("schema") != LAUNCH_ATTESTATION_SCHEMA:
        raise ProvenanceError(
            f"unsupported launch attestation schema: {payload.get('schema')}",
        )
    return payload


def _describe_artifact(root: Path, path: Path) -> dict[str, Any]:
    resolved = Path(path).resolve()
    try:
        relative = resolved.relative_to(root)
    except ValueError as error:
        raise ProvenanceError(f"artifact is outside the run directory: {resolved}") from error
    if not resolved.is_file():
        raise ProvenanceError(f"artifact is missing or not a regular file: {resolved}")
    return {
        "path": relative.as_posix(),
        "size_bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved),
    }


def _verify_artifacts(root: Path, value: Any) -> dict[str, Path]:
    if not isinstance(value, dict) or not value:
        raise ProvenanceError("run manifest artifacts must be a non-empty object")
    resolved: dict[str, Path] = {}
    for name, descriptor in value.items():
        if not isinstance(name, str) or not name or not isinstance(descriptor, dict):
            raise ProvenanceError("run manifest contains an invalid artifact descriptor")
        resolved[name] = _verify_artifact(root, name, descriptor)
    return resolved


def _verify_artifact(root: Path, name: str, descriptor: Mapping[str, Any]) -> Path:
    relative = descriptor.get("path")
    size_bytes = descriptor.get("size_bytes")
    expected_hash = descriptor.get("sha256")
    relative_path = Path(relative) if isinstance(relative, str) else Path()
    if (
        not isinstance(relative, str) or not relative
        or relative_path.is_absolute() or ".." in relative_path.parts
    ):
        raise ProvenanceError(f"artifact {name} path must be relative")
    if isinstance(size_bytes, bool) or not isinstance(size_bytes, int) or size_bytes < 0:
        raise ProvenanceError(f"artifact {name} has an invalid size")
    if not isinstance(expected_hash, str) or SHA256_PATTERN.fullmatch(expected_hash) is None:
        raise ProvenanceError(f"artifact {name} has an invalid SHA-256")
    path = (root / relative_path).resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise ProvenanceError(f"artifact {name} escapes the run directory") from error
    if not path.is_file():
        raise ProvenanceError(f"artifact {name} is missing or not a regular file")
    if path.stat().st_size != size_bytes:
        raise ProvenanceError(f"artifact {name} size does not match the run manifest")
    actual_hash = sha256_file(path)
    if not hmac.compare_digest(actual_hash, expected_hash):
        raise ProvenanceError(f"artifact {name} SHA-256 does not match the run manifest")
    return path


def _verify_attestations(
    root: Path,
    artifacts: Mapping[str, Any],
    *,
    run_id: str,
    profile: str,
    seed: int,
) -> None:
    for artifact_name, role in MANAGED_ATTESTATIONS.items():
        descriptor = artifacts.get(artifact_name)
        if not isinstance(descriptor, Mapping):
            raise ProvenanceError(
                f"managed manifest is missing required artifact: {artifact_name}",
            )
        path = (root / str(descriptor.get("path", ""))).resolve()
        payload = _load_attestation(path)
        _validate_attestation_identity(
            payload.get("role"), payload.get("run_id"),
            payload.get("profile"), payload.get("seed"),
        )
        _check_expected("attestation role", payload.get("role"), role)
        _check_expected("attestation run_id", payload.get("run_id"), run_id)
        _check_expected("attestation profile", payload.get("profile"), profile)
        _check_expected("attestation seed", payload.get("seed"), seed)
        if not isinstance(payload.get("details"), dict):
            raise ProvenanceError(f"attestation details must be an object: {path}")


def _check_expected(name: str, actual: Any, expected: Any) -> None:
    if expected is not None and actual != expected:
        raise ProvenanceError(
            f"run manifest {name} mismatch: expected {expected}, got {actual}",
        )


def _verify_quality(actual: Any, expected: Mapping[str, Any]) -> None:
    if not isinstance(actual, dict):
        raise ProvenanceError("run manifest quality is missing or invalid")
    for name, expected_value in expected.items():
        actual_value = actual.get(name)
        if _equal_scalar(actual_value, expected_value):
            continue
        raise ProvenanceError(
            f"run manifest quality.{name} mismatch: expected {expected_value}, got {actual_value}",
        )


def _equal_scalar(actual: Any, expected: Any) -> bool:
    if isinstance(actual, bool) or isinstance(expected, bool):
        return actual is expected
    if isinstance(actual, (int, float)) and isinstance(expected, (int, float)):
        return math.isfinite(float(actual)) and math.isclose(
            float(actual), float(expected), rel_tol=0.0, abs_tol=1e-12,
        )
    return actual == expected


def _write_json_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8",
    )
    os.replace(temporary, path)


def _attest_command(args: argparse.Namespace) -> int:
    details: dict[str, Any] = {}
    if args.details_json is not None:
        parsed = json.loads(args.details_json)
        if not isinstance(parsed, dict):
            raise ProvenanceError("--details-json must contain a JSON object")
        details.update(parsed)
    for assignment in args.detail:
        if "=" not in assignment:
            raise ProvenanceError("--detail must use KEY=VALUE syntax")
        name, value = assignment.split("=", 1)
        if not name:
            raise ProvenanceError("--detail key must not be empty")
        details[name] = value
    write_launch_attestation(
        args.output,
        role=args.role,
        run_id=args.run_id,
        profile=args.profile,
        seed=args.seed,
        details=details,
    )
    return 0


def _parse_cli(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Write SmartDrone SITL provenance records.")
    commands = parser.add_subparsers(dest="command", required=True)
    attest = commands.add_parser("attest", help="write a launcher attestation")
    attest.add_argument("--output", type=Path, required=True)
    attest.add_argument("--role", choices=tuple(sorted(SUPPORTED_ATTESTATION_ROLES)), required=True)
    attest.add_argument("--run-id", required=True)
    attest.add_argument("--profile", choices=tuple(sorted(SUPPORTED_PROFILES)), required=True)
    attest.add_argument("--seed", type=int, required=True)
    attest.add_argument("--details-json")
    attest.add_argument("--detail", action="append", default=[])
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    try:
        args = _parse_cli(argv)
        if args.command == "attest":
            return _attest_command(args)
    except (ProvenanceError, json.JSONDecodeError) as error:
        print(f"provenance error: {error}", file=sys.stderr)
        return 1
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
