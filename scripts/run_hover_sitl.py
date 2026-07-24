#!/usr/bin/env python3
"""Run a SmartDrone-controlled PX4/Gazebo hover scenario."""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from sitl_hover.metrics import (
    AnalysisOptions,
    analyze_ulog,
    error_summary,
    smart_drone_latency_summary,
)
from sitl_hover.protocol import (
    ACK_E_BAD_STATE,
    CMD_ARM,
    CMD_LAND,
    CMD_MOVE,
    CMD_OFFBOARD,
    CMD_RUNTIME_MODE,
    PX4_MAIN_MODE_AUTO,
    PX4_MAIN_MODE_OFFBOARD,
    PX4_SUB_MODE_AUTO_LAND,
    RUNTIME_MODE_SLAM,
    CommandError,
    TlvClient,
    VehicleState,
    build_move_payload,
    usable_tracking,
)
from sitl_hover.provenance import (
    RunProvenance,
    apply_provenance_gate,
    create_run_manifest,
    verify_run_manifest,
)
from sitl_hover.sim_clock import (
    FlightClock,
    FlightClockMark,
    GazeboClockReader,
    SteadyFlightClock,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
ASSIGNMENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*=.*$")
DEFAULT_GZ_WORLD = "smartdrone_hover"
DEFAULT_GZ_MODEL = "smartdrone_x500"
DEFAULT_WRENCH_SCRIPT = REPO_ROOT / "scripts" / "apply_gz_wrench.sh"
MAX_SIM_WALL_TIMEOUT_SCALE = 20.0
STATE_FRESHNESS_S = 0.5
DEFAULT_FAULT_COMMAND_TIMEOUT_S = 10.0
DEFAULT_WRENCH_WALL_TIMEOUT_S = 30.0
MINIMUM_WRENCH_RTF = 0.05
WRENCH_TIMEOUT_OVERHEAD_S = 10.0
FAULT_COMMAND_CLEANUP_GRACE_S = 5.0


def _neutral_image_fault(action: str = "clear") -> dict[str, Any]:
    return {
        "action": action, "blur_sigma": 0.0, "brightness": 1.0,
        "noise_stddev": 0.0, "drop_rate": 0.0, "delay_ms": 0,
        "blackout_ms": 0,
    }


@dataclass(frozen=True)
class QualitySettings:
    blur_sigma: float
    brightness: float
    noise_stddev: float
    drop_rate: float
    delay_ms: int
    short_blackout_ms: int
    long_blackout_ms: int


@dataclass(frozen=True)
class FlightSettings:
    target_x: float
    target_y: float
    target_z: float
    target_yaw: float
    max_velocity: float
    ascent_timeout_s: float
    control_ascent_s: float
    hover_duration_s: float
    short_blackout_at_s: float
    long_blackout_at_s: float
    tracking_stable_s: float
    tracking_loss_land_ms: int
    impulse_force_n: float
    impulse_duration_ms: int


@dataclass(frozen=True)
class RunContract:
    run_id: str
    mode: str
    profile: str
    scenario: str
    seed: int
    quality: QualitySettings
    flight: FlightSettings


@dataclass
class HoverOutcome:
    short_blackout_applied: bool = False
    short_blackout_recovered: bool = False
    long_blackout_applied: bool = False
    tracking_loss_threshold_exceeded: bool = False
    autonomous_land_observed: bool = False
    autonomous_land_after_long_blackout: bool = False
    disarmed_after_autonomous_land: bool = False
    premature_autonomous_land_observed: bool = False
    unexpected_disarm_observed: bool = False
    flight_state_telemetry_lost: bool = False
    land_main_mode: int | None = None
    land_sub_mode: int | None = None
    cleanup_land_sent: bool = False
    cleanup_land_attempt_count: int = 0
    cleanup_land_acknowledged: bool = False


class RunArtifacts:
    def __init__(self, output_dir: Path) -> None:
        self.output_dir = output_dir
        output_dir.mkdir(parents=True, exist_ok=True)
        self.started_monotonic_s = time.monotonic()
        self.phase = "startup"
        self.states: list[tuple[str, VehicleState]] = []
        self.events: list[dict[str, Any]] = []
        self.frame_record_count = 0
        self._frame_path = output_dir / "smart_drone_frames.jsonl"
        self._frame_path.touch(exist_ok=True)
        self._states = (output_dir / "smart_drone_states.jsonl").open("w", encoding="utf-8", buffering=1)
        self._run_log = (output_dir / "run.jsonl").open("w", encoding="utf-8", buffering=1)
        for name in ("px4.log", "smart_drone.log", "faults.log"):
            (output_dir / name).touch(exist_ok=True)

    def close(self) -> None:
        self._states.close()
        self._run_log.close()

    def elapsed_s(self) -> float:
        return time.monotonic() - self.started_monotonic_s

    def event(self, name: str, **details: Any) -> None:
        record = {"elapsed_s": self.elapsed_s(), "event": name, **details}
        self.events.append(record)
        self._run_log.write(json.dumps(record, sort_keys=True) + "\n")
        print(f"[{record['elapsed_s']:8.3f}] {name}", flush=True)

    def on_state(self, state: VehicleState) -> None:
        self.states.append((self.phase, state))
        record = state.to_dict()
        record.update({"phase": self.phase, "elapsed_s": self.elapsed_s()})
        self._states.write(json.dumps(record, sort_keys=True) + "\n")

    def collect_smart_drone_frames(self, log_path: Path) -> None:
        self.frame_record_count = 0
        with log_path.open("r", encoding="utf-8", errors="replace") as source, self._frame_path.open(
            "w", encoding="utf-8",
        ) as destination:
            for line in source:
                try:
                    record = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not isinstance(record, dict) or record.get("tag") not in ("slam_dfx", "odom_ts"):
                    continue
                destination.write(json.dumps(record, sort_keys=True) + "\n")
                self.frame_record_count += 1

    def write_live_trajectory(self) -> None:
        fields = [
            "elapsed_s", "phase", "source", "estimate_x", "estimate_y", "estimate_z",
            "tracking_state", "armed", "px4_flight_state_valid",
            "px4_main_mode", "px4_sub_mode", "reset_counter",
        ]
        with (self.output_dir / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for phase, state in self.states:
                writer.writerow({
                    "elapsed_s": state.received_monotonic_s - self.started_monotonic_s,
                    "phase": phase, "source": "smartdrone_state", "estimate_x": state.x,
                    "estimate_y": state.y, "estimate_z": state.z,
                    "tracking_state": state.tracking_state, "armed": int(state.armed),
                    "px4_flight_state_valid": int(state.px4_flight_state_valid),
                    "px4_main_mode": state.px4_main_mode, "px4_sub_mode": state.px4_sub_mode,
                    "reset_counter": state.reset_counter,
                })


class ManagedProcesses:
    def __init__(self, output_dir: Path, inherited_env: dict[str, str]) -> None:
        self._output_dir = output_dir
        self._inherited_env = inherited_env
        self._processes: list[tuple[str, subprocess.Popen[bytes], Any]] = []

    @staticmethod
    def _argv_and_env(command: str, inherited_env: dict[str, str]) -> tuple[list[str], dict[str, str]]:
        parts = shlex.split(command)
        environment = dict(inherited_env)
        while parts and ASSIGNMENT.match(parts[0]):
            key, value = parts.pop(0).split("=", 1)
            environment[key] = value
        if not parts:
            raise ValueError("launcher command contains no executable")
        return parts, environment

    def start(self, name: str, command: str) -> None:
        if not command:
            return
        argv, environment = self._argv_and_env(command, self._inherited_env)
        log = (self._output_dir / f"{name}.log").open("ab", buffering=0)
        process = subprocess.Popen(
            argv,
            cwd=REPO_ROOT,
            env=environment,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self._processes.append((name, process, log))

    def check(self) -> None:
        for name, process, _ in self._processes:
            status = process.poll()
            if status is not None:
                raise RuntimeError(f"{name} launcher exited early with status {status}")

    @staticmethod
    def _signal(process: subprocess.Popen[bytes], value: signal.Signals) -> None:
        if process.poll() is None:
            try:
                os.killpg(process.pid, value)
            except ProcessLookupError:
                pass

    def stop(self) -> None:
        for _, process, _ in reversed(self._processes):
            self._signal(process, signal.SIGINT)
        self._wait_or_signal(8.0, signal.SIGTERM)
        self._wait_or_signal(4.0, signal.SIGKILL)
        for _, process, log in self._processes:
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass
            log.close()

    def _wait_or_signal(self, timeout_s: float, next_signal: signal.Signals) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and any(process.poll() is None for _, process, _ in self._processes):
            time.sleep(0.05)
        for _, process, _ in reversed(self._processes):
            self._signal(process, next_signal)


class FaultCommands:
    def __init__(
        self,
        image_command: str,
        disturbance_command: str,
        log_path: Path,
        state_path: Path,
        *,
        environment: dict[str, str] | None = None,
        world_name: str = DEFAULT_GZ_WORLD,
        model_name: str = DEFAULT_GZ_MODEL,
    ) -> None:
        self._image_command = image_command
        self._disturbance_command = disturbance_command or self._default_disturbance_command()
        self._log_path = log_path
        self._state_path = state_path
        self._environment = dict(environment or os.environ)
        self._world_name = world_name
        self._model_name = model_name
        self._generation = 0
        self._write_state({"kind": "quality", **_neutral_image_fault()})

    @staticmethod
    def _default_disturbance_command() -> str:
        return (
            f"{shlex.quote(str(DEFAULT_WRENCH_SCRIPT))} "
            "--world {{world_name}} --model {{model_name}} "
            "--force-n {{force_n}} --duration-ms {{duration_ms}}"
        )

    @staticmethod
    def _expand(command: str, values: dict[str, Any]) -> list[str]:
        expanded = command
        for key, value in values.items():
            expanded = expanded.replace("{{" + key + "}}", str(value))
        return shlex.split(expanded)

    def execute(
        self,
        kind: str,
        values: dict[str, Any],
        keepalive: Callable[[], None] | None = None,
    ) -> bool:
        command = self._disturbance_command if kind == "impulse" else self._image_command
        if kind != "impulse":
            self._write_state({"kind": kind, **values})
        if not command:
            status = "skipped" if kind == "impulse" else "fault_state_file"
            self._append_log({"kind": kind, "status": status, **values})
            return kind != "impulse"
        payload = {"kind": kind, **values}
        if kind == "impulse":
            payload.update({"world_name": self._world_name, "model_name": self._model_name})
        environment = dict(self._environment)
        environment["SMART_DRONE_FAULT_JSON"] = json.dumps(payload, sort_keys=True)
        for key, value in payload.items():
            environment[f"SMART_DRONE_FAULT_{key.upper()}"] = str(value)
        status, output = self._run_command(command, payload, environment, keepalive)
        self._append_log({**payload, "status": status, "output": output})
        if status != 0:
            raise RuntimeError(f"{kind} command failed with status {status}")
        return True

    def _run_command(
        self,
        command: str,
        payload: dict[str, Any],
        environment: dict[str, str],
        keepalive: Callable[[], None] | None,
    ) -> tuple[int, str]:
        with tempfile.TemporaryFile(mode="w+", encoding="utf-8") as output:
            process = subprocess.Popen(
                self._expand(command, payload), cwd=REPO_ROOT, env=environment,
                stdout=output, stderr=subprocess.STDOUT, text=True,
                start_new_session=True,
            )
            deadline = time.monotonic() + _fault_command_timeout_s(
                payload, environment,
            )
            try:
                while process.poll() is None and time.monotonic() < deadline:
                    keepalive() if keepalive is not None else time.sleep(0.02)
            except BaseException:
                self._stop_command(process)
                raise
            if process.poll() is None:
                self._stop_command(process)
                status = 124
            else:
                status = int(process.returncode)
            output.seek(0)
            return status, output.read()

    @staticmethod
    def _stop_command(process: subprocess.Popen[str]) -> None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=FAULT_COMMAND_CLEANUP_GRACE_S)
            return
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            return
        process.wait(timeout=2.0)

    def _write_state(self, payload: dict[str, Any]) -> None:
        if payload.get("action") == "clear":
            payload = {"kind": payload.get("kind", "quality"), **_neutral_image_fault()}
        self._generation += 1
        state = {
            "schema": "smartdrone.sitl.image_fault.v1",
            "generation": self._generation,
            "updated_monotonic_ns": time.monotonic_ns(),
            **payload,
        }
        temporary = self._state_path.with_suffix(self._state_path.suffix + ".tmp")
        temporary.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(temporary, self._state_path)

    def _append_log(self, record: dict[str, Any]) -> None:
        with self._log_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, sort_keys=True) + "\n")


def _default_output(profile: str, scenario: str, seed: int) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return REPO_ROOT / "output" / "sitl_runs" / f"{stamp}_{profile}_{scenario}_seed{seed}"


def _prepare_run_output(output_dir: Path) -> None:
    if not output_dir.exists():
        return
    allowed = {"matrix_invocation.json", "matrix_runner.log"}
    existing = {item.name for item in output_dir.iterdir()}
    unexpected = sorted(existing - allowed)
    if unexpected:
        raise ValueError(
            f"output directory contains existing run artifacts: {output_dir} ({unexpected[0]})"
        )


def _uses_smartdrone_px4_launcher(command: str) -> bool:
    parts = shlex.split(command)
    while parts and ASSIGNMENT.match(parts[0]):
        parts.pop(0)
    return bool(parts) and Path(parts[0]).name == "run_px4_gz_sitl.sh"


def _wait_for_px4_launcher(
    processes: ManagedProcesses, ready_file: Path, timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        processes.check()
        if ready_file.is_file():
            return
        time.sleep(0.05)
    raise TimeoutError(f"PX4/Gazebo launcher did not become ready: {ready_file}")


def _configured_positive_float(
    environment: dict[str, str], name: str,
) -> float | None:
    raw_value = environment.get(name)
    if raw_value is None:
        return None
    try:
        value = float(raw_value)
    except ValueError:
        return None
    return value if math.isfinite(value) and value > 0.0 else None


def _fault_command_timeout_s(
    payload: dict[str, Any], environment: dict[str, str],
) -> float:
    if payload.get("kind") != "impulse":
        return DEFAULT_FAULT_COMMAND_TIMEOUT_S
    configured = _configured_positive_float(
        environment, "SMART_DRONE_GZ_WRENCH_TIMEOUT_S",
    )
    duration_ms = float(payload.get("duration_ms", 0.0))
    wrench_timeout_s = configured or max(
        DEFAULT_WRENCH_WALL_TIMEOUT_S,
        duration_ms / (MINIMUM_WRENCH_RTF * 1000.0)
        + WRENCH_TIMEOUT_OVERHEAD_S,
    )
    return max(
        DEFAULT_FAULT_COMMAND_TIMEOUT_S,
        wrench_timeout_s + FAULT_COMMAND_CLEANUP_GRACE_S,
    )


def _start_flight_clock(
    args: argparse.Namespace,
    environment: dict[str, str],
    artifacts: RunArtifacts,
    processes: ManagedProcesses,
) -> FlightClock:
    if args.flight_clock == "steady":
        artifacts.event("flight_clock_ready", source="steady", rate=1.0)
        return SteadyFlightClock()
    clock: GazeboClockReader | None = None
    try:
        clock = GazeboClockReader(
            args.gz_world, environment, artifacts.output_dir / "gz_clock.log",
        )
        snapshot = clock.wait_ready(args.clock_ready_timeout_s, processes.check)
    except Exception as error:
        if clock is not None:
            clock.close()
        if args.flight_clock == "auto" and not args.px4_command:
            artifacts.event(
                "flight_clock_fallback", source="steady", reason=str(error), rate=1.0,
            )
            return SteadyFlightClock()
        raise RuntimeError(f"Gazebo flight clock is unavailable: {error}") from error
    assert clock is not None
    artifacts.event(
        "flight_clock_ready", source="gazebo", topic=clock.topic,
        sim_time_s=snapshot.time_s, rate=snapshot.rate,
    )
    return clock


def _write_yaml(path: Path, payload: dict[str, Any]) -> None:
    try:
        import yaml  # type: ignore

        text = yaml.safe_dump(payload, sort_keys=False)
    except ImportError:
        text = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    path.write_text(text, encoding="utf-8")


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8",
    )
    os.replace(temporary, path)


def _rendering_metadata() -> dict[str, Any]:
    environment_keys = {
        "requested": "SMART_DRONE_GZ_RENDERING_REQUESTED",
        "selected": "SMART_DRONE_GZ_RENDERING_SELECTED",
        "reason": "SMART_DRONE_GZ_RENDERING_REASON",
        "render_node": "SMART_DRONE_GZ_RENDER_NODE",
        "renderer": "SMART_DRONE_GZ_RENDERER",
    }
    metadata: dict[str, Any] = {
        name: os.environ.get(key) or None
        for name, key in environment_keys.items()
    }
    metadata["verified"] = (
        os.environ.get("SMART_DRONE_GZ_RENDERING_VERIFIED") == "1"
    )
    return metadata


def _scenario_document(args: argparse.Namespace, contract: RunContract) -> dict[str, Any]:
    sim_config = args.sim_config or REPO_ROOT / "sim" / "px4_gz" / "config" / "smartdrone_sim.yaml"
    return {
        "schema": "smartdrone.sitl.scenario.v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "profile": args.profile,
        "scenario": args.scenario,
        "seed": args.seed,
        "provenance": {"run_id": contract.run_id, "mode": contract.mode},
        "command_endpoint": {"host": args.command_host, "port": args.command_port},
        "gz_partition": args.gz_partition,
        "gazebo_target": {"world": args.gz_world, "model": args.gz_model},
        "rendering": _rendering_metadata(),
        "flight_clock": {
            "requested": args.flight_clock,
            "topic": f"/world/{args.gz_world}/clock",
        },
        "visual_watchdog": {
            "clock": "steady",
            "simulation_time_scale": args.sim_watchdog_scale,
            "pose_max_age_base_ms": 100,
            "pose_max_age_wall_ms": round(100 * args.sim_watchdog_scale),
            "loss_land_base_ms": args.tracking_loss_land_ms,
            "loss_land_wall_ms": round(
                args.tracking_loss_land_ms * args.sim_watchdog_scale
            ),
        },
        "sim_config": str(sim_config.resolve()),
        "launchers": {
            "px4": args.px4_command or None,
            "smart_drone": args.smart_drone_command or None,
        },
        "ulog_input": str(args.ulog.resolve()) if args.ulog is not None else None,
        "fault_state": "fault_state.json",
        "quality": asdict(contract.quality),
        "flight": asdict(contract.flight),
    }


def _flight_state_valid(state: VehicleState) -> bool:
    return state.px4_flight_state_valid


def _state_fresh(state: VehicleState | None, now_s: float | None = None) -> bool:
    if state is None:
        return False
    current_s = time.monotonic() if now_s is None else now_s
    return current_s - state.received_monotonic_s <= STATE_FRESHNESS_S


def _record_flight_state_telemetry_loss(
    artifacts: RunArtifacts,
    outcome: HoverOutcome,
    reason: str,
) -> None:
    if outcome.flight_state_telemetry_lost:
        return
    outcome.flight_state_telemetry_lost = True
    artifacts.event("flight_state_telemetry_lost", reason=reason)


def _wait_initial_state(client: TlvClient, artifacts: RunArtifacts, timeout_s: float) -> None:
    artifacts.event("wait_smartdrone_state")
    state = client.wait_for_state(
        lambda value: _flight_state_valid(value) and value.px4_main_mode != 0,
        timeout_s,
    )
    artifacts.event("smartdrone_state_ready", px4_main_mode=state.px4_main_mode)


def _prepare_tracking(
    client: TlvClient,
    artifacts: RunArtifacts,
    profile: str,
    stable_s: float,
    timeout_s: float,
    flight_clock: FlightClock,
) -> None:
    if profile != "vision":
        return
    artifacts.event("start_slam")
    client.command(CMD_RUNTIME_MODE, bytes((RUNTIME_MODE_SLAM,)), timeout_s=timeout_s)
    state = _wait_for_tracking_stable(
        client, flight_clock, stable_s, timeout_s,
    )
    artifacts.event(
        "tracking_ready", tracking_state=state.tracking_state,
        stable_sim_s=stable_s, clock_rate=flight_clock.rate,
    )


def _wait_for_tracking_stable(
    client: TlvClient,
    flight_clock: FlightClock,
    stable_s: float,
    wall_timeout_s: float,
) -> VehicleState:
    deadline = time.monotonic() + wall_timeout_s
    condition_since: FlightClockMark | None = None
    while time.monotonic() < deadline:
        client.poll(min(0.05, max(0.0, deadline - time.monotonic())))
        state = client.latest_state
        state_fresh = _state_fresh(state)
        if state is None or not state_fresh or not usable_tracking(state):
            condition_since = None
            continue
        condition_since = flight_clock.mark() if condition_since is None else condition_since
        if flight_clock.elapsed_s(condition_since) >= stable_s:
            return state
    raise TimeoutError(
        f"visual tracking was not stable for {stable_s:.1f}s of simulation time "
        f"within {wall_timeout_s:.1f}s wall time"
    )


def _enter_flight(client: TlvClient, artifacts: RunArtifacts, flight: FlightSettings, timeout_s: float) -> None:
    artifacts.phase = "offboard"
    artifacts.event("command_offboard")
    client.command(CMD_OFFBOARD, timeout_s=timeout_s)
    client.wait_for_state(
        lambda state: (
            _flight_state_valid(state)
            and state.px4_main_mode == PX4_MAIN_MODE_OFFBOARD
        ),
        timeout_s,
    )
    artifacts.event("offboard_confirmed")
    artifacts.event("command_arm")
    _command_arm_when_ready(client, artifacts, timeout_s)
    artifacts.event("armed_confirmed")
    payload = build_move_payload(
        flight.target_x, flight.target_y, flight.target_z,
        flight.target_yaw, flight.max_velocity,
    )
    artifacts.phase = "ascent"
    artifacts.event("command_move", x=flight.target_x, y=flight.target_y, z=flight.target_z)
    _command_move_when_ready(client, artifacts, payload, timeout_s)


def _command_arm_when_ready(
    client: TlvClient,
    artifacts: RunArtifacts,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    attempts = 0
    while True:
        remaining_s = deadline - time.monotonic()
        if remaining_s <= 0.0:
            raise TimeoutError(f"PX4 did not arm within {timeout_s:.1f}s")
        try:
            client.command(CMD_ARM, timeout_s=min(1.0, remaining_s))
        except CommandError as error:
            if error.status != ACK_E_BAD_STATE:
                raise
        attempts += 1
        remaining_s = deadline - time.monotonic()
        try:
            client.wait_for_state(
                lambda state: _flight_state_valid(state) and state.armed,
                min(1.0, max(0.0, remaining_s)),
            )
        except TimeoutError:
            continue
        if attempts > 1:
            artifacts.event("arm_gate_recovered", attempts=attempts)
        return


def _command_move_when_ready(
    client: TlvClient,
    artifacts: RunArtifacts,
    payload: bytes,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    transient_rejections = 0
    while True:
        remaining_s = deadline - time.monotonic()
        if remaining_s <= 0.0:
            raise CommandError(
                "move localization gate did not recover before timeout",
                command=CMD_MOVE,
                status=ACK_E_BAD_STATE,
            )
        try:
            client.command(CMD_MOVE, payload, timeout_s=min(1.0, remaining_s))
            if transient_rejections > 0:
                artifacts.event(
                    "move_gate_recovered",
                    transient_rejections=transient_rejections,
                )
            return
        except CommandError as error:
            if error.status != ACK_E_BAD_STATE:
                raise
            transient_rejections += 1
            client.poll(min(0.05, max(0.0, deadline - time.monotonic())))


def _wait_for_hover_target(
    client: TlvClient,
    artifacts: RunArtifacts,
    profile: str,
    flight: FlightSettings,
    flight_clock: FlightClock,
) -> None:
    artifacts.event("wait_hover_target")
    if profile == "control":
        _wait_flight_duration(client, flight_clock, flight.control_ascent_s)
        artifacts.event(
            "hover_target_settle_complete", source="fixed_wait",
            clock_source=flight_clock.source, clock_rate=flight_clock.rate,
        )
        return

    def target_reached(state: VehicleState) -> bool:
        horizontal = math.hypot(state.x - flight.target_x, state.y - flight.target_y)
        vertical = abs(state.z - flight.target_z)
        tracking_ready = profile != "vision" or usable_tracking(state)
        flight_ready = (
            _flight_state_valid(state) and state.armed
            and state.px4_main_mode == PX4_MAIN_MODE_OFFBOARD
        )
        return flight_ready and tracking_ready and horizontal <= 0.25 and vertical <= 0.15

    state = _wait_for_state_by_flight_time(
        client, target_reached, flight_clock, flight.ascent_timeout_s, 0.5,
    )
    artifacts.event("hover_target_confirmed", x=state.x, y=state.y, z=state.z)


def _wait_flight_duration(
    client: TlvClient,
    flight_clock: FlightClock,
    duration_s: float,
) -> None:
    started = flight_clock.mark()
    while flight_clock.elapsed_s(started) < duration_s:
        client.poll(0.05)


def _wait_for_state_by_flight_time(
    client: TlvClient,
    condition: Callable[[VehicleState], bool],
    flight_clock: FlightClock,
    timeout_s: float,
    stable_s: float,
) -> VehicleState:
    started = flight_clock.mark()
    condition_since: FlightClockMark | None = None
    while flight_clock.elapsed_s(started) < timeout_s:
        client.poll(0.05)
        state = client.latest_state
        state_fresh = _state_fresh(state)
        if state is None or not state_fresh or not condition(state):
            condition_since = None
            continue
        condition_since = flight_clock.mark() if condition_since is None else condition_since
        if flight_clock.elapsed_s(condition_since) >= stable_s:
            return state
    raise TimeoutError(f"hover target was not stable within {timeout_s:.1f}s of simulation time")


def _quality_values(quality: QualitySettings, action: str, blackout_ms: int = 0) -> dict[str, Any]:
    if action == "clear":
        return _neutral_image_fault()
    return {
        "action": action,
        "blur_sigma": quality.blur_sigma,
        "brightness": quality.brightness,
        "noise_stddev": quality.noise_stddev,
        "drop_rate": quality.drop_rate,
        "delay_ms": quality.delay_ms,
        "blackout_ms": blackout_ms,
    }


def _blackout_values(blackout_ms: int) -> dict[str, Any]:
    return {**_neutral_image_fault("blackout"), "blackout_ms": blackout_ms}


def _scheduled_events(
    scenario: str,
    quality: QualitySettings,
    flight: FlightSettings,
    faults: FaultCommands,
    keepalive: Callable[[], None],
) -> list[tuple[float, str, Callable[[], bool]]]:
    if scenario == "impulse":
        values = {"action": "impulse", "force_n": flight.impulse_force_n, "duration_ms": flight.impulse_duration_ms}
        return [(15.0, "impulse", lambda: faults.execute("impulse", values, keepalive))]
    if scenario == "quality":
        return [
            (2.0, "quality_apply", lambda: faults.execute("quality", _quality_values(quality, "apply"), keepalive)),
            (max(3.0, flight.hover_duration_s - 2.0), "quality_clear", lambda: faults.execute("quality", _quality_values(quality, "clear"), keepalive)),
        ]
    if scenario == "loss":
        return [
            (flight.short_blackout_at_s, "blackout_short", lambda: faults.execute("blackout", _blackout_values(quality.short_blackout_ms), keepalive)),
            (flight.long_blackout_at_s, "blackout_long", lambda: faults.execute("blackout", _blackout_values(quality.long_blackout_ms), keepalive)),
        ]
    return []


def _simulation_mark_details(
    flight_clock: FlightClock,
    mark: FlightClockMark,
) -> dict[str, float | int]:
    if flight_clock.source != "gazebo":
        return {}
    return {
        "sim_time_s": mark.time_s,
        "sim_clock_reset_counter": mark.reset_counter,
    }


def _gazebo_hover_window(
    events: list[dict[str, Any]],
) -> tuple[float | None, float | None]:
    started = [event for event in events if event.get("event") == "hover_started"]
    finished = [event for event in events if event.get("event") == "hover_finished"]
    if len(started) != 1 or len(finished) != 1:
        return None, None
    first, last = started[0], finished[0]
    if first.get("clock_source") != "gazebo" or last.get("clock_source") != "gazebo":
        return None, None
    values = (first.get("sim_time_s"), last.get("sim_time_s"))
    resets = (
        first.get("sim_clock_reset_counter"),
        last.get("sim_clock_reset_counter"),
    )
    if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in values):
        return None, None
    if not all(math.isfinite(float(value)) for value in values):
        return None, None
    if any(isinstance(value, bool) or not isinstance(value, int) for value in resets):
        return None, None
    if resets[0] != resets[1] or float(values[1]) <= float(values[0]):
        return None, None
    return float(values[0]), float(values[1])


def _run_hover(
    client: TlvClient,
    artifacts: RunArtifacts,
    scenario: str,
    quality: QualitySettings,
    flight: FlightSettings,
    faults: FaultCommands,
    monitor_visual_tracking: bool,
    flight_clock: FlightClock,
) -> HoverOutcome:
    artifacts.phase = "hover"
    started = flight_clock.mark()
    artifacts.event(
        "hover_started", duration_s=flight.hover_duration_s,
        clock_source=flight_clock.source, clock_rate=flight_clock.rate,
        **_simulation_mark_details(flight_clock, started),
    )
    pending = _scheduled_events(scenario, quality, flight, faults, lambda: client.poll(0.02))
    tracking_lost: FlightClockMark | None = None
    last_tracking_state_s: float | None = None
    short_recovery_eligible_s: float | None = None
    short_recovery_stable: FlightClockMark | None = None
    outcome = HoverOutcome()
    while flight_clock.elapsed_s(started) < flight.hover_duration_s:
        client.poll(0.05)
        elapsed_s = flight_clock.elapsed_s(started)
        while pending and elapsed_s >= pending[0][0]:
            scheduled_s, name, callback = pending.pop(0)
            applied = callback()
            artifacts.event(
                name, applied=applied, scheduled_sim_s=scheduled_s,
                hover_elapsed_sim_s=elapsed_s, clock_rate=flight_clock.rate,
            )
            if name == "blackout_short":
                outcome.short_blackout_applied = applied
                short_recovery_eligible_s = elapsed_s + quality.short_blackout_ms / 1000.0
            elif name == "blackout_long":
                outcome.long_blackout_applied = applied
                tracking_lost = None
                last_tracking_state_s = None
        state = client.latest_state
        state_fresh = _state_fresh(state)
        if scenario == "loss" and (
            not state_fresh or state is None or not _flight_state_valid(state)
        ):
            reason = "state_stale" if not state_fresh else "px4_flight_state_invalid"
            _record_flight_state_telemetry_loss(artifacts, outcome, reason)
            break
        if (
            scenario == "loss" and outcome.short_blackout_applied
            and not outcome.short_blackout_recovered and not outcome.long_blackout_applied
            and short_recovery_eligible_s is not None and elapsed_s >= short_recovery_eligible_s
        ):
            if (
                state_fresh and state is not None and _flight_state_valid(state)
                and usable_tracking(state)
            ):
                short_recovery_stable = (
                    flight_clock.mark() if short_recovery_stable is None else short_recovery_stable
                )
                if flight_clock.elapsed_s(short_recovery_stable) >= 0.5:
                    outcome.short_blackout_recovered = True
                    artifacts.event(
                        "blackout_short_recovered", hover_elapsed_sim_s=elapsed_s,
                    )
            else:
                short_recovery_stable = None
        if not monitor_visual_tracking:
            continue
        if scenario == "loss":
            assert state is not None
            if state.received_monotonic_s != last_tracking_state_s:
                last_tracking_state_s = state.received_monotonic_s
                if usable_tracking(state):
                    tracking_lost = None
                elif outcome.long_blackout_applied:
                    tracking_lost = flight_clock.mark() if tracking_lost is None else tracking_lost
                    loss_ms = flight_clock.elapsed_s(tracking_lost) * 1000.0
                    if (
                        loss_ms >= flight.tracking_loss_land_ms
                        and not outcome.tracking_loss_threshold_exceeded
                    ):
                        artifacts.event(
                            "tracking_loss_threshold_exceeded",
                            tracking_state=state.tracking_state,
                        )
                        outcome.tracking_loss_threshold_exceeded = True
            if _autonomous_land_state(state) and state.armed:
                if (
                    outcome.long_blackout_applied
                    and outcome.tracking_loss_threshold_exceeded
                ):
                    _record_autonomous_land(artifacts, outcome, state)
                else:
                    _record_premature_autonomous_land(artifacts, outcome, state)
                break
            if not state.armed:
                _record_unexpected_disarm(artifacts, outcome, state)
                break
            continue
        if state is not None and _autonomous_land_state(state):
            _record_autonomous_land(artifacts, outcome, state)
            break
        if state_fresh and state is not None and usable_tracking(state):
            tracking_lost = None
            continue
        tracking_lost = flight_clock.mark() if tracking_lost is None else tracking_lost
        if flight_clock.elapsed_s(tracking_lost) * 1000.0 < flight.tracking_loss_land_ms:
            continue
        if not outcome.tracking_loss_threshold_exceeded:
            tracking_state = state.tracking_state if state is not None else None
            artifacts.event("tracking_loss_threshold_exceeded", tracking_state=tracking_state)
            outcome.tracking_loss_threshold_exceeded = True
    finished = flight_clock.mark()
    artifacts.event(
        "hover_finished",
        autonomous_land_observed=outcome.autonomous_land_observed,
        hover_elapsed_sim_s=finished.time_s - started.time_s,
        clock_source=flight_clock.source, clock_rate=flight_clock.rate,
        **_simulation_mark_details(flight_clock, finished),
    )
    return outcome


def _record_autonomous_land(
    artifacts: RunArtifacts,
    outcome: HoverOutcome,
    state: VehicleState,
) -> None:
    outcome.autonomous_land_observed = True
    outcome.autonomous_land_after_long_blackout = (
        outcome.long_blackout_applied
        and outcome.tracking_loss_threshold_exceeded
    )
    outcome.land_main_mode = state.px4_main_mode
    outcome.land_sub_mode = state.px4_sub_mode
    artifacts.event(
        "autonomous_land_observed",
        armed=state.armed,
        px4_main_mode=state.px4_main_mode,
        px4_sub_mode=state.px4_sub_mode,
        source="observed_without_runner_land_command",
    )


def _record_premature_autonomous_land(
    artifacts: RunArtifacts,
    outcome: HoverOutcome,
    state: VehicleState,
) -> None:
    outcome.premature_autonomous_land_observed = True
    outcome.land_main_mode = state.px4_main_mode
    outcome.land_sub_mode = state.px4_sub_mode
    artifacts.event(
        "premature_autonomous_land_observed",
        armed=state.armed,
        px4_main_mode=state.px4_main_mode,
        px4_sub_mode=state.px4_sub_mode,
    )


def _record_unexpected_disarm(
    artifacts: RunArtifacts,
    outcome: HoverOutcome,
    state: VehicleState,
) -> None:
    outcome.unexpected_disarm_observed = True
    outcome.land_main_mode = state.px4_main_mode
    outcome.land_sub_mode = state.px4_sub_mode
    artifacts.event(
        "unexpected_disarm_observed",
        px4_main_mode=state.px4_main_mode,
        px4_sub_mode=state.px4_sub_mode,
    )


def _autonomous_land_state(state: VehicleState) -> bool:
    return (
        state.px4_main_mode == PX4_MAIN_MODE_AUTO
        and state.px4_sub_mode == PX4_SUB_MODE_AUTO_LAND
    )


def _loss_safety_passed(outcome: HoverOutcome) -> bool:
    return (
        outcome.short_blackout_applied and outcome.short_blackout_recovered
        and outcome.long_blackout_applied
        and outcome.tracking_loss_threshold_exceeded
        and outcome.autonomous_land_observed
        and outcome.autonomous_land_after_long_blackout
        and outcome.disarmed_after_autonomous_land
        and not outcome.premature_autonomous_land_observed
        and not outcome.unexpected_disarm_observed
        and not outcome.flight_state_telemetry_lost
        and outcome.cleanup_land_attempt_count == 0
        and not outcome.cleanup_land_sent
    )


def _flight_stage_wall_timeout(timeout_s: float, flight_clock: FlightClock) -> float:
    rate = flight_clock.rate
    if (
        flight_clock.source != "gazebo" or rate is None
        or not math.isfinite(rate) or rate <= 0.0
    ):
        return timeout_s
    return timeout_s * min(MAX_SIM_WALL_TIMEOUT_SCALE, max(1.0, 1.0 / rate))


def _land(
    client: TlvClient,
    artifacts: RunArtifacts,
    command_event: str | None,
    timeout_s: float,
    flight_clock: FlightClock,
    loss_outcome: HoverOutcome | None = None,
) -> VehicleState:
    artifacts.phase = "landing"
    wall_timeout_s = _flight_stage_wall_timeout(timeout_s, flight_clock)
    if command_event is not None:
        artifacts.event(
            command_event, timeout_wall_s=wall_timeout_s,
            clock_rate=flight_clock.rate,
        )
        client.command(CMD_LAND, timeout_s=min(wall_timeout_s, 5.0))
    state = (
        _wait_for_loss_disarm(client, artifacts, loss_outcome, wall_timeout_s)
        if loss_outcome is not None
        else client.wait_for_state(
            lambda value: _flight_state_valid(value) and not value.armed,
            wall_timeout_s,
            stable_s=0.5,
        )
    )
    artifacts.event("disarmed_confirmed", px4_main_mode=state.px4_main_mode, px4_sub_mode=state.px4_sub_mode)
    return state


def _wait_for_loss_disarm(
    client: TlvClient,
    artifacts: RunArtifacts,
    outcome: HoverOutcome,
    timeout_s: float,
) -> VehicleState:
    deadline = time.monotonic() + timeout_s
    disarmed_since_s: float | None = None
    last_seen_s: float | None = None
    while time.monotonic() < deadline:
        client.poll(min(0.05, max(0.0, deadline - time.monotonic())))
        state = client.latest_state
        if not _state_fresh(state):
            _record_flight_state_telemetry_loss(artifacts, outcome, "state_stale")
            raise RuntimeError("CMD_STATE telemetry became stale while awaiting disarm")
        assert state is not None
        if not _flight_state_valid(state):
            _record_flight_state_telemetry_loss(
                artifacts, outcome, "px4_flight_state_invalid",
            )
            raise RuntimeError("PX4 flight-state telemetry became invalid while awaiting disarm")
        if state.received_monotonic_s == last_seen_s:
            continue
        last_seen_s = state.received_monotonic_s
        if state.armed:
            disarmed_since_s = None
            continue
        disarmed_since_s = (
            state.received_monotonic_s
            if disarmed_since_s is None else disarmed_since_s
        )
        if state.received_monotonic_s - disarmed_since_s >= 0.5:
            return state
    raise TimeoutError(f"state condition did not hold for 0.5s within {timeout_s:.1f}s")


def _request_cleanup_land(
    client: TlvClient,
    artifacts: RunArtifacts,
    outcome: HoverOutcome,
    event_name: str,
    timeout_s: float,
) -> None:
    outcome.cleanup_land_sent = True
    outcome.cleanup_land_attempt_count += 1
    artifacts.event(
        event_name,
        attempt=outcome.cleanup_land_attempt_count,
    )
    try:
        client.command(CMD_LAND, timeout_s=timeout_s)
    except Exception as error:
        artifacts.event(
            "cleanup_land_failed",
            attempt=outcome.cleanup_land_attempt_count,
            error=str(error),
        )
        raise
    outcome.cleanup_land_acknowledged = True
    artifacts.event(
        "cleanup_land_acknowledged",
        attempt=outcome.cleanup_land_attempt_count,
    )


def _basic_metrics(
    artifacts: RunArtifacts,
    flight: FlightSettings,
    outcome: str,
    error: str | None,
    hover_outcome: HoverOutcome,
    contract: RunContract,
) -> dict[str, Any]:
    hover_states = [state for phase, state in artifacts.states if phase == "hover"]
    horizontal = [math.hypot(state.x - flight.target_x, state.y - flight.target_y) for state in hover_states]
    vertical = [abs(state.z - flight.target_z) for state in hover_states]
    duration = 0.0
    if len(hover_states) > 1:
        duration = hover_states[-1].received_monotonic_s - hover_states[0].received_monotonic_s
    return {
        "schema": "smartdrone.sitl.hover.runner_metrics.v1",
        "source": "smartdrone_state_only",
        "run_id": contract.run_id,
        "mode": contract.mode,
        "profile": contract.profile,
        "scenario": contract.scenario,
        "seed": contract.seed,
        "rendering": _rendering_metadata(),
        "outcome": outcome,
        "error": error,
        "hover_accuracy": {"horizontal_m": error_summary(horizontal), "vertical_m": error_summary(vertical)},
        "state_frequency_hz": len(hover_states) / duration if duration > 0.0 else None,
        "smart_drone_frame_records": artifacts.frame_record_count,
        "protocol_note": "formal acceptance requires Gazebo ground truth from a PX4 ULog",
        "visual_loss_safety": asdict(hover_outcome),
        "acceptance": {"passed": False, "complete": False, "reason": "ground_truth_ulog_not_analyzed"},
        "events": artifacts.events,
    }


def _resolve_ulog(path: Path) -> Path:
    if path.is_file():
        return path
    candidates = sorted(path.rglob("*.ulg"), key=lambda item: item.stat().st_mtime, reverse=True)
    if not candidates:
        raise FileNotFoundError(f"no .ulg file found under {path}")
    return candidates[0]


def _apply_smart_drone_latency(
    metrics: dict[str, Any], jsonl_path: Path,
) -> None:
    hover_window = metrics["hover_window"]
    latency = smart_drone_latency_summary(
        jsonl_path,
        float(hover_window["start_s"]),
        float(hover_window["end_s"]),
    )
    visual = metrics["visual_odometry"]
    visual["ulog_transport_latency_p95_ms"] = visual["latency_p95_ms"]
    visual["latency_source"] = "smart_drone_frames_jsonl.wall_total_ms"
    visual["latency_sample_count"] = latency["sample_count"]
    visual["latency_p95_ms"] = latency["latency_p95_ms"]
    visual["timing_breakdown_p95_ms"] = latency["timing_p95_ms"]
    visual["timing_breakdown_sample_count"] = latency["timing_sample_count"]
    for check in metrics["acceptance"]["checks"]:
        if check["name"] != "visual_latency_p95_ms":
            continue
        check["actual"] = latency["latency_p95_ms"]
        check["passed"] = (
            latency["latency_p95_ms"] is not None
            and latency["latency_p95_ms"] <= check["limit"]
        )


def _run_mode(args: argparse.Namespace) -> str:
    if args.px4_command and args.smart_drone_command:
        return "managed"
    if args.px4_command or args.smart_drone_command:
        return "attached"
    return "external"


def _run_contract(
    args: argparse.Namespace,
    quality: QualitySettings,
    flight: FlightSettings,
) -> RunContract:
    return RunContract(
        run_id=str(uuid.uuid4()), mode=_run_mode(args),
        profile=args.profile, scenario=args.scenario, seed=args.seed,
        quality=quality, flight=flight,
    )


def _manifest_artifacts(output_dir: Path, mode: str) -> dict[str, Path]:
    artifacts = {"runner_metrics": output_dir / "runner_metrics.json"}
    if mode == "managed":
        artifacts.update({
            "px4_attestation": output_dir / "px4_attestation.json",
            "smart_drone_attestation": output_dir / "smart_drone_attestation.json",
        })
    return artifacts


def _create_run_provenance(
    output_dir: Path,
    ulog_path: Path,
    options: AnalysisOptions,
    contract: RunContract,
) -> RunProvenance:
    manifest_path = output_dir / "run_manifest.json"
    quality = asdict(contract.quality)
    create_run_manifest(
        manifest_path, run_id=contract.run_id, mode=contract.mode,
        profile=contract.profile, scenario=contract.scenario, seed=contract.seed,
        ulog_path=ulog_path, artifacts=_manifest_artifacts(output_dir, contract.mode),
        flight=asdict(contract.flight), quality=quality,
        runner={
            "tool": "scripts/run_hover_sitl.py",
            "rendering": _rendering_metadata(),
        },
        analysis_options=asdict(options),
    )
    return verify_run_manifest(
        manifest_path, ulog_path, expected_profile=contract.profile,
        expected_scenario=contract.scenario, expected_seed=contract.seed,
        expected_quality=quality, require_managed=contract.mode == "managed",
    )


def _read_runner_metrics(output_dir: Path) -> dict[str, Any]:
    payload = json.loads(
        (output_dir / "runner_metrics.json").read_text(encoding="utf-8"),
    )
    if not isinstance(payload, dict):
        raise ValueError("runner_metrics.json root is not an object")
    return payload


def _diagnostic_metrics(
    runner_metrics: dict[str, Any], contract: RunContract,
) -> dict[str, Any]:
    metrics = copy.deepcopy(runner_metrics)
    metrics["schema"] = "smartdrone.sitl.hover.metrics.v1"
    provenance = RunProvenance(
        status="unverified", mode=contract.mode, run_id=contract.run_id,
        profile=contract.profile, scenario=contract.scenario, seed=contract.seed,
        manifest_path=None, ulog_path=None, ulog_sha256=None,
        issues=("ground_truth_ulog_not_analyzed",),
    )
    apply_provenance_gate(metrics, provenance)
    return metrics


def _complete_analysis_checks(metrics: dict[str, Any]) -> None:
    checks = metrics["acceptance"]["checks"]
    metrics["acceptance"]["complete"] = (
        bool(metrics["acceptance"].get("complete"))
        and all(item.get("actual") is not None for item in checks)
    )
    metrics["acceptance"]["passed"] = (
        metrics["acceptance"]["complete"]
        and all(item["passed"] for item in checks)
    )


def _analyze_if_requested(
    args: argparse.Namespace,
    output_dir: Path,
    options: AnalysisOptions,
    contract: RunContract,
) -> bool | None:
    ulog_path = args.ulog
    if ulog_path is None and args.px4_command:
        ulog_path = output_dir / "px4_sitl"
    if ulog_path is None:
        return None
    source = _resolve_ulog(ulog_path)
    destination = output_dir / "px4.ulg"
    if source.resolve() != destination.resolve():
        shutil.copy2(source, destination)
    provenance = _create_run_provenance(output_dir, destination, options, contract)
    runner_metrics = _read_runner_metrics(output_dir)
    metrics_path = output_dir / "metrics.json"
    with tempfile.TemporaryDirectory(prefix=".analysis-", dir=output_dir) as staging:
        staging_dir = Path(staging)
        metrics = analyze_ulog(destination, staging_dir, options)
        metrics["runner"] = runner_metrics
        _augment_analysis_metrics(args, output_dir, metrics, runner_metrics)
        _complete_analysis_checks(metrics)
        apply_provenance_gate(metrics, provenance)
        _publish_analysis_artifacts(staging_dir, output_dir)
        _write_json_atomic(metrics_path, metrics)
    return bool(metrics["acceptance"]["passed"])


def _publish_analysis_artifacts(staging_dir: Path, output_dir: Path) -> None:
    for name in ("trajectory.csv", "hover_metrics.png"):
        source = staging_dir / name
        if source.is_file():
            os.replace(source, output_dir / name)


def _augment_analysis_metrics(
    args: argparse.Namespace,
    output_dir: Path,
    metrics: dict[str, Any],
    runner_metrics: dict[str, Any],
) -> None:
    if args.profile != "control":
        _apply_smart_drone_latency(
            metrics, output_dir / "smart_drone_frames.jsonl",
        )
        frame_count = runner_metrics.get("smart_drone_frame_records")
        metrics["acceptance"]["checks"].append({
            "name": "smart_drone_frame_records", "actual": frame_count,
            "relation": "min", "limit": 1,
            "passed": isinstance(frame_count, int) and frame_count >= 1,
        })
    if args.scenario == "loss":
        _augment_loss_checks(metrics, runner_metrics)


def _augment_loss_checks(
    metrics: dict[str, Any], runner_metrics: dict[str, Any],
) -> None:
    safety = runner_metrics["visual_loss_safety"]
    checks = []
    for name in (
        "short_blackout_applied", "short_blackout_recovered",
        "long_blackout_applied", "autonomous_land_after_long_blackout",
    ):
        actual = bool(safety[name])
        checks.append({
            "name": name, "actual": actual, "relation": "required",
            "limit": True, "passed": actual,
        })
    cleanup_sent = bool(safety["cleanup_land_sent"])
    checks.append({
        "name": "runner_cleanup_land_not_used", "actual": cleanup_sent,
        "relation": "required_false", "limit": False,
        "passed": not cleanup_sent,
    })
    metrics["acceptance"]["checks"].extend(checks)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Launch or attach to PX4/Gazebo and SmartDrone, then command a TLV-only hover test.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--profile", choices=("control", "truth", "vision"), required=True)
    parser.add_argument("--scenario", choices=("nominal", "impulse", "quality", "loss"), default="nominal")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--px4-command", default="", help="argv-style PX4/Gazebo launcher; leading KEY=VALUE assignments are accepted")
    parser.add_argument("--smart-drone-command", default="", help="argv-style SmartDrone launcher; omit to attach to an existing process")
    parser.add_argument("--sim-config", type=Path, help="sets SMART_DRONE_SIM_CONFIG for a launched SmartDrone")
    parser.add_argument(
        "--gz-partition",
        default=os.environ.get("SMART_DRONE_GZ_PARTITION", os.environ.get("GZ_PARTITION", "smartdrone_sitl")),
        help="Gazebo Transport partition shared by PX4 and SmartDrone",
    )
    parser.add_argument("--gz-world", default=DEFAULT_GZ_WORLD, help="Gazebo world containing the wrench system")
    parser.add_argument("--gz-model", default=DEFAULT_GZ_MODEL, help="Gazebo model receiving the impulse")
    parser.add_argument(
        "--flight-clock", choices=("auto", "gazebo", "steady"), default="auto",
        help="clock used for ascent, hover, and scheduled simulation events",
    )
    parser.add_argument(
        "--clock-ready-timeout-s", type=float, default=10.0,
        help="wall-clock timeout for the first advancing Gazebo clock samples",
    )
    parser.add_argument("--command-host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=14550)
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--bind-port", type=int, default=0)
    parser.add_argument("--launcher-warmup-s", type=float, default=2.0)
    parser.add_argument("--launcher-ready-timeout-s", type=float, default=600.0)
    parser.add_argument("--state-timeout-s", type=float, default=60.0)
    parser.add_argument("--command-timeout-s", type=float, default=20.0)
    parser.add_argument("--land-timeout-s", type=float, default=30.0)
    parser.add_argument("--target-x", type=float, default=0.0)
    parser.add_argument("--target-y", type=float, default=0.0)
    parser.add_argument("--target-z", type=float, default=-1.5)
    parser.add_argument("--target-yaw", type=float, default=0.0)
    parser.add_argument("--max-velocity", type=float, default=1.0)
    parser.add_argument("--ascent-timeout-s", type=float, default=30.0)
    parser.add_argument("--control-ascent-s", type=float, default=8.0, help="fixed settle time because CMD_STATE has no PX4 local position in control profile")
    parser.add_argument("--hover-duration-s", type=float, default=30.0)
    parser.add_argument("--short-blackout-at-s", type=float, default=10.0)
    parser.add_argument("--long-blackout-at-s", type=float, default=20.0)
    parser.add_argument("--tracking-stable-s", type=float, default=2.0)
    parser.add_argument("--tracking-loss-land-ms", type=int, default=500)
    parser.add_argument(
        "--sim-watchdog-scale", type=float,
        default=os.environ.get("SMART_DRONE_SIM_WATCHDOG_SCALE", "1.0"),
        help="wall-time dilation for simulation-only visual safety watchdogs",
    )
    parser.add_argument("--blur-sigma", type=float, default=1.5)
    parser.add_argument("--brightness", type=float, default=0.65)
    parser.add_argument("--noise-stddev", type=float, default=3.0)
    parser.add_argument("--drop-rate", type=float, default=0.05)
    parser.add_argument("--delay-ms", type=int, default=40)
    parser.add_argument("--short-blackout-ms", type=int, default=300)
    parser.add_argument("--long-blackout-ms", type=int, default=750)
    parser.add_argument("--impulse-force-n", type=float, default=4.0)
    parser.add_argument("--impulse-duration-ms", type=int, default=250)
    parser.add_argument("--image-command", default="", help="fault command template; use tokens such as {{action}} and {{blackout_ms}}")
    parser.add_argument("--disturbance-command", default="", help="impulse command template; supports {{world_name}}, {{model_name}}, {{force_n}}, and {{duration_ms}}")
    parser.add_argument("--ulog", type=Path, help="ULog file or directory to collect and analyze after shutdown")
    return parser.parse_args()


def _settings(args: argparse.Namespace) -> tuple[QualitySettings, FlightSettings]:
    if args.seed <= 0:
        raise ValueError("--seed must be a positive integer")
    if not 0.0 <= args.drop_rate <= 1.0:
        raise ValueError("--drop-rate must be in [0, 1]")
    if args.hover_duration_s <= 0.0 or args.max_velocity <= 0.0:
        raise ValueError("hover duration and maximum velocity must be positive")
    if args.ascent_timeout_s <= 0.0 or args.control_ascent_s < 0.0:
        raise ValueError("ascent timing values are invalid")
    if args.launcher_ready_timeout_s <= 0.0:
        raise ValueError("launcher ready timeout must be positive")
    if args.clock_ready_timeout_s <= 0.0:
        raise ValueError("clock ready timeout must be positive")
    timeouts = (args.state_timeout_s, args.command_timeout_s, args.land_timeout_s)
    if any(value <= 0.0 for value in timeouts) or args.launcher_warmup_s < 0.0:
        raise ValueError("launcher warmup and command timeouts are invalid")
    if not 0.0 <= args.blur_sigma <= 20.0 or not 0.0 <= args.noise_stddev <= 128.0:
        raise ValueError("blur or noise exceeds the simulator fault range")
    if not 0.0 <= args.brightness <= 4.0:
        raise ValueError("brightness must be in [0, 4]")
    if not 0 <= args.delay_ms <= 5000:
        raise ValueError("delay must be in [0, 5000] ms")
    if not 0 <= args.short_blackout_ms <= 60000 or not 0 <= args.long_blackout_ms <= 60000:
        raise ValueError("blackout durations must be in [0, 60000] ms")
    if not 1 <= args.command_port <= 65535 or not 0 <= args.bind_port <= 65535:
        raise ValueError("UDP ports are outside the valid range")
    if args.tracking_loss_land_ms <= 0 or args.tracking_stable_s < 0.0:
        raise ValueError("tracking timing values are invalid")
    if not math.isfinite(args.sim_watchdog_scale) or not 1.0 <= args.sim_watchdog_scale <= 20.0:
        raise ValueError("simulation watchdog scale must be in [1, 20]")
    if args.scenario == "loss":
        if args.profile != "vision":
            raise ValueError("loss scenario requires the vision profile")
        if not 0 < args.short_blackout_ms < args.tracking_loss_land_ms:
            raise ValueError("short blackout must stay below the visual-loss LAND timeout")
        if args.long_blackout_ms <= args.tracking_loss_land_ms:
            raise ValueError("long blackout must exceed the visual-loss LAND timeout")
        short_ends_s = args.short_blackout_at_s + args.short_blackout_ms / 1000.0
        if args.short_blackout_at_s < 0.0 or short_ends_s + 0.5 >= args.long_blackout_at_s:
            raise ValueError("loss scenario must leave 0.5 s to recover before the long blackout")
        if args.long_blackout_at_s >= args.hover_duration_s:
            raise ValueError("long blackout must start before the hover interval ends")
    if not 0.0 < abs(args.impulse_force_n) <= 100.0:
        raise ValueError("impulse force magnitude must be in (0, 100] N")
    if not 1 <= args.impulse_duration_ms <= 5000:
        raise ValueError("impulse duration must be in [1, 5000] ms")
    quality = QualitySettings(
        args.blur_sigma, args.brightness, args.noise_stddev, args.drop_rate,
        args.delay_ms, args.short_blackout_ms, args.long_blackout_ms,
    )
    flight = FlightSettings(
        args.target_x, args.target_y, args.target_z, args.target_yaw,
        args.max_velocity, args.ascent_timeout_s, args.control_ascent_s,
        args.hover_duration_s, args.short_blackout_at_s, args.long_blackout_at_s,
        args.tracking_stable_s, args.tracking_loss_land_ms,
        args.impulse_force_n, args.impulse_duration_ms,
    )
    return quality, flight


def _profile_launch_environment(args: argparse.Namespace, output_dir: Path) -> dict[str, str]:
    profile_values = {
        "control": ("idle", "none", "0"),
        "truth": ("idle", "position_velocity", "0"),
        "vision": ("slam", "position_velocity", "1"),
    }
    auto_mode, pose_output, require_vision = profile_values[args.profile]
    environment = dict(os.environ)
    environment.update({
        "GZ_PARTITION": args.gz_partition,
        "GZ_IP": "127.0.0.1",
        "SMART_DRONE_GZ_PARTITION": args.gz_partition,
        "SMART_DRONE_SIM_PROFILE": args.profile,
        "SMART_DRONE_SIM_SEED": str(args.seed),
        "SMART_DRONE_SIM_WORLD": args.gz_world,
        "SMART_DRONE_SIM_MODEL": args.gz_model,
        "SMART_DRONE_SIM_LOG_DIR": str(output_dir),
        "SMART_DRONE_PX4_LOG_DIR": str(output_dir / "px4_sitl"),
        "SMART_DRONE_PX4_READY_FILE": str(output_dir / "px4_launcher.ready"),
        "SMART_DRONE_AUTO_MODE": auto_mode,
        "SMART_DRONE_PX4_POSE_OUTPUT_MODE": pose_output,
        "SMART_DRONE_OFFBOARD_REQUIRES_VISION": require_vision,
        "SMART_DRONE_VISUAL_POSE_MAX_AGE_MS": "100",
        "SMART_DRONE_VISUAL_LOSS_LAND_MS": str(args.tracking_loss_land_ms),
        "SMART_DRONE_SIM_WATCHDOG_SCALE": f"{args.sim_watchdog_scale:g}",
        "SMART_DRONE_JSON_DIAGNOSTICS": "1",
        "SMART_DRONE_SIM_FAULT_FILE": str((output_dir / "fault_state.json").resolve()),
    })
    sim_config = args.sim_config or REPO_ROOT / "sim" / "px4_gz" / "config" / "smartdrone_sim.yaml"
    environment["SMART_DRONE_SIM_CONFIG"] = str(sim_config.resolve())
    return environment


def _configure_provenance_environment(
    environment: dict[str, str],
    output_dir: Path,
    contract: RunContract,
) -> None:
    keys = (
        "SMART_DRONE_SITL_RUN_ID",
        "SMART_DRONE_PX4_ATTESTATION_FILE",
        "SMART_DRONE_ATTESTATION_FILE",
    )
    for key in keys:
        environment.pop(key, None)
    if contract.mode != "managed":
        return
    environment.update({
        "SMART_DRONE_SITL_RUN_ID": contract.run_id,
        "SMART_DRONE_PX4_ATTESTATION_FILE": str(
            (output_dir / "px4_attestation.json").resolve()
        ),
        "SMART_DRONE_ATTESTATION_FILE": str(
            (output_dir / "smart_drone_attestation.json").resolve()
        ),
    })


def run(args: argparse.Namespace) -> int:
    quality, flight = _settings(args)
    contract = _run_contract(args, quality, flight)
    output_dir = (args.output_dir or _default_output(args.profile, args.scenario, args.seed)).resolve()
    _prepare_run_output(output_dir)
    artifacts = RunArtifacts(output_dir)
    scenario = _scenario_document(args, contract)
    _write_yaml(output_dir / "scenario.yaml", scenario)
    launch_env = _profile_launch_environment(args, output_dir)
    _configure_provenance_environment(launch_env, output_dir, contract)
    processes = ManagedProcesses(output_dir, launch_env)
    faults = FaultCommands(
        args.image_command, args.disturbance_command,
        output_dir / "faults.log", output_dir / "fault_state.json",
        environment=launch_env, world_name=args.gz_world,
        model_name=args.gz_model,
    )
    outcome, error_text = "failed", None
    hover_outcome = HoverOutcome()
    client: TlvClient | None = None
    flight_clock: FlightClock = SteadyFlightClock()
    vehicle_may_be_armed = False
    try:
        ready_file = Path(launch_env["SMART_DRONE_PX4_READY_FILE"])
        ready_file.unlink(missing_ok=True)
        processes.start("px4", args.px4_command)
        if args.px4_command and _uses_smartdrone_px4_launcher(args.px4_command):
            _wait_for_px4_launcher(
                processes, ready_file, args.launcher_ready_timeout_s,
            )
        flight_clock = _start_flight_clock(args, launch_env, artifacts, processes)
        processes.start("smart_drone", args.smart_drone_command)
        time.sleep(max(0.0, args.launcher_warmup_s))
        processes.check()
        client = TlvClient(
            args.command_host, args.command_port, args.bind_host, args.bind_port,
            sequence_seed=args.seed, state_handler=artifacts.on_state,
        )
        _wait_initial_state(client, artifacts, args.state_timeout_s)
        _prepare_tracking(
            client, artifacts, args.profile, flight.tracking_stable_s,
            args.state_timeout_s, flight_clock,
        )
        _enter_flight(client, artifacts, flight, args.command_timeout_s)
        vehicle_may_be_armed = True
        _wait_for_hover_target(client, artifacts, args.profile, flight, flight_clock)
        hover_outcome = _run_hover(
            client, artifacts, args.scenario, quality, flight, faults,
            args.profile == "vision", flight_clock,
        )
        land_event: str | None = "command_land"
        if args.scenario == "loss":
            land_event = None
            landing_already_observed = (
                hover_outcome.autonomous_land_observed
                or hover_outcome.premature_autonomous_land_observed
                or hover_outcome.unexpected_disarm_observed
            )
            if not landing_already_observed:
                artifacts.event("loss_failsafe_not_observed")
                _request_cleanup_land(
                    client,
                    artifacts,
                    hover_outcome,
                    "loss_cleanup_land_sent",
                    min(args.command_timeout_s, 5.0),
                )
        _land(
            client,
            artifacts,
            land_event,
            args.land_timeout_s,
            flight_clock,
            hover_outcome if args.scenario == "loss" else None,
        )
        if args.scenario == "loss" and hover_outcome.autonomous_land_observed:
            hover_outcome.disarmed_after_autonomous_land = True
            artifacts.event("autonomous_land_disarm_confirmed")
        vehicle_may_be_armed = False
        outcome = "completed"
    except (Exception, KeyboardInterrupt) as error:
        error_text = f"{type(error).__name__}: {error}"
        artifacts.event("run_failed", error=error_text)
        latest_state = client.latest_state if client is not None else None
        latest_confirms_armed = (
            latest_state is not None
            and _flight_state_valid(latest_state)
            and latest_state.armed
        )
        cleanup_needed = (
            client is not None
            and (vehicle_may_be_armed or latest_confirms_armed)
            and not hover_outcome.cleanup_land_acknowledged
        )
        if cleanup_needed:
            try:
                assert client is not None
                _request_cleanup_land(
                    client,
                    artifacts,
                    hover_outcome,
                    "cleanup_land_sent",
                    min(args.command_timeout_s, 3.0),
                )
            except Exception:
                pass
    finally:
        if client is not None:
            client.close()
        flight_clock.close()
        processes.stop()
        artifacts.collect_smart_drone_frames(output_dir / "smart_drone.log")
        artifacts.write_live_trajectory()
        basic_metrics = _basic_metrics(
            artifacts, flight, outcome, error_text, hover_outcome, contract,
        )
        _write_json_atomic(output_dir / "runner_metrics.json", basic_metrics)
        _write_json_atomic(
            output_dir / "metrics.json", _diagnostic_metrics(basic_metrics, contract),
        )
        scenario["result"] = {
            "outcome": outcome,
            "error": error_text,
            "flight_clock": {
                "source": flight_clock.source,
                "rate": flight_clock.rate,
            },
            "visual_loss_safety": asdict(hover_outcome),
            "events": artifacts.events,
        }
        _write_yaml(output_dir / "scenario.yaml", scenario)
        artifacts.close()
    hover_start_s, hover_end_s = _gazebo_hover_window(artifacts.events)
    options = AnalysisOptions(
        profile=args.profile, scenario=args.scenario, target_x=flight.target_x,
        target_y=flight.target_y, target_z=flight.target_z,
        hover_start_s=hover_start_s, hover_end_s=hover_end_s,
    )
    try:
        acceptance = _analyze_if_requested(args, output_dir, options, contract)
    except Exception as error:
        scenario["analysis"] = {"outcome": "failed", "error": f"{type(error).__name__}: {error}"}
        _write_yaml(output_dir / "scenario.yaml", scenario)
        print(f"ULog analysis failed: {error}", file=sys.stderr)
        return 1
    scenario["analysis"] = {
        "outcome": "not_requested" if acceptance is None else "completed",
        "acceptance_passed": acceptance,
    }
    _write_yaml(output_dir / "scenario.yaml", scenario)
    print(f"artifacts: {output_dir}")
    if outcome != "completed":
        return 1
    loss_safety_failed = (
        args.scenario == "loss"
        and not _loss_safety_passed(hover_outcome)
    )
    return 0 if acceptance is not False and not loss_safety_failed else 2


def main() -> int:
    try:
        return run(_parse_args())
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
