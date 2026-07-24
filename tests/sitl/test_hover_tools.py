#!/usr/bin/env python3
"""Unit tests for the dependency-light PX4 hover tooling."""

from __future__ import annotations

import json
import math
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest import mock

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from sitl_hover.metrics import (  # noqa: E402
    AnalysisOptions,
    AttitudeSample,
    POSE_FRAME_FRD,
    POSE_FRAME_NED,
    PositionSample,
    _align_visual_positions,
    _extract_position_samples,
    _flight_diagnostics,
    _land_command_time,
    _visual_health,
    analyze_ulog,
    aligned_errors,
    calculate_recovery_time,
    detect_hover_window,
    error_summary,
    hover_errors,
    percentile,
    smart_drone_latency_summary,
)
from sitl_hover.protocol import (  # noqa: E402
    CMD_MOVE,
    CMD_STATE,
    FRAME_NED,
    MOVE_PAYLOAD,
    STATE_FLAG_PX4_FLIGHT_STATE_VALID,
    STATE_PAYLOAD,
    ProtocolError,
    TlvFrame,
    build_move_payload,
    decode_datagram,
    encode_frame,
    parse_state,
    usable_tracking,
)
from run_hover_sitl import (  # noqa: E402
    FaultCommands,
    FlightSettings,
    HoverOutcome,
    QualitySettings,
    RunArtifacts,
    RunContract,
    _apply_smart_drone_latency,
    _blackout_values,
    _fault_command_timeout_s,
    _flight_stage_wall_timeout,
    _gazebo_hover_window,
    _autonomous_land_state,
    _configure_provenance_environment,
    _loss_safety_passed,
    _parse_args,
    _prepare_run_output,
    _profile_launch_environment,
    _rendering_metadata,
    _run_contract,
    _run_mode,
    _run_hover,
    _settings,
    _uses_smartdrone_px4_launcher,
    _wait_flight_duration,
    _wait_for_tracking_stable,
)
from sitl_hover.provenance import verify_run_manifest  # noqa: E402
from sitl_hover.sim_clock import (  # noqa: E402
    FlightClockMark,
    GazeboClockReader,
)


class ProtocolTest(unittest.TestCase):
    def test_detects_repository_px4_launcher_after_environment_prefix(self) -> None:
        self.assertTrue(_uses_smartdrone_px4_launcher(
            "GZ_PARTITION=test ./scripts/run_px4_gz_sitl.sh --headless",
        ))
        self.assertFalse(_uses_smartdrone_px4_launcher("python3 custom_px4.py"))

    def test_round_trip_current_state_payload(self) -> None:
        payload = STATE_PAYLOAD.pack(
            1, 3, 5, 1, 7, 9,
            1.25, -2.5, -1.5, 1.0, 0.0, 0.0, 0.0,
            6, 0,
        )
        encoded = encode_frame(TlvFrame(
            CMD_STATE, STATE_FLAG_PX4_FLIGHT_STATE_VALID, 42, 1234, payload,
        ))
        frame = decode_datagram(encoded)
        state = parse_state(frame, received_s=12.0)
        self.assertEqual(len(payload), 38)
        self.assertEqual(state.sequence, 42)
        self.assertTrue(state.armed)
        self.assertTrue(state.px4_flight_state_valid)
        self.assertEqual(state.tracking_state, 5)
        self.assertEqual(state.px4_main_mode, 6)
        self.assertAlmostEqual(state.z, -1.5)
        self.assertTrue(usable_tracking(state))

    def test_state_without_flight_state_valid_flag_is_fail_closed(self) -> None:
        payload = STATE_PAYLOAD.pack(
            1, 3, 5, 1, 0, 0,
            0.0, 0.0, -1.5, 1.0, 0.0, 0.0, 0.0,
            6, 0,
        )

        state = parse_state(TlvFrame(CMD_STATE, 0, 1, 0, payload))

        self.assertFalse(state.px4_flight_state_valid)

    def test_crc_corruption_is_rejected(self) -> None:
        encoded = bytearray(encode_frame(TlvFrame(CMD_MOVE, 0, 8, 99, b"abc")))
        encoded[-3] ^= 0x80
        with self.assertRaises(ProtocolError):
            decode_datagram(bytes(encoded))

    def test_move_is_ned_position_payload(self) -> None:
        payload = build_move_payload(1.0, 2.0, -1.5, 0.25, 1.2)
        frame, x, y, z, yaw, max_velocity = MOVE_PAYLOAD.unpack(payload)
        self.assertEqual(frame, FRAME_NED)
        self.assertEqual(len(payload), 21)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 2.0)
        self.assertAlmostEqual(z, -1.5)
        self.assertAlmostEqual(yaw, 0.25)
        self.assertAlmostEqual(max_velocity, 1.2)

    def test_non_current_state_payload_is_rejected(self) -> None:
        legacy = struct.pack("<BBBHH7f", 1, 3, 2, 0, 0, *(0.0,) * 7)
        frame = TlvFrame(CMD_STATE, 0, 1, 0, legacy)
        with self.assertRaises(ProtocolError):
            parse_state(frame)


class GazeboClockReaderTest(unittest.TestCase):
    def test_reads_fragmented_json_reports_rate_and_detects_reset(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            fake_gz = root / "gz"
            fake_gz.write_text(
                "#!/usr/bin/env python3\n"
                "import json, time\n"
                "first = {'real': {'sec': '0', 'nsec': 0}, "
                "'sim': {'sec': '1', 'nsec': 0}}\n"
                "print(json.dumps(first, indent=2), flush=True)\n"
                "time.sleep(0.05)\n"
                "second = {'real': {'sec': '0', 'nsec': 50000000}, "
                "'sim': {'sec': '1', 'nsec': 10000000}}\n"
                "print(json.dumps(second), flush=True)\n"
                "time.sleep(0.5)\n"
                "reset = {'real': {'sec': '0', 'nsec': 550000000}, "
                "'sim': {'sec': '0', 'nsec': 100000000}}\n"
                "print(json.dumps(reset), flush=True)\n"
                "time.sleep(2)\n",
                encoding="utf-8",
            )
            fake_gz.chmod(0o755)
            reader = GazeboClockReader(
                "test_world", dict(os.environ), root / "clock.log",
                executable=str(fake_gz), stall_timeout_s=1.0,
            )
            try:
                snapshot = reader.wait_ready(1.0)
                self.assertAlmostEqual(snapshot.time_s, 1.01, places=6)
                self.assertAlmostEqual(float(snapshot.rate), 0.2, places=6)
                mark = reader.mark()
                deadline = time.monotonic() + 1.0
                while reader.mark().reset_counter == mark.reset_counter:
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(0.01)
                with self.assertRaisesRegex(RuntimeError, "clock reset"):
                    reader.elapsed_s(mark)
            finally:
                reader.close()


class _ManualFlightClock:
    source = "manual"

    def __init__(self, rate: float = 0.2) -> None:
        self.time_s = 0.0
        self.rate = rate

    def mark(self) -> FlightClockMark:
        return FlightClockMark(self.time_s, 0)

    def elapsed_s(self, mark: FlightClockMark) -> float:
        return self.time_s - mark.time_s

    def close(self) -> None:
        pass


class _ClockAdvancingClient:
    def __init__(self, clock: _ManualFlightClock) -> None:
        self.clock = clock
        self.latest_state = None
        self.poll_count = 0

    def poll(self, _duration_s: float) -> None:
        self.clock.time_s += 0.1
        self.poll_count += 1
        if self.latest_state is not None:
            self.latest_state.received_monotonic_s = time.monotonic()


class _RecordingFaults:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str]] = []

    def execute(
        self,
        kind: str,
        values: dict[str, object],
        keepalive: object,
    ) -> bool:
        self.calls.append((kind, str(values["action"])))
        return True


class FlightStageClockTest(unittest.TestCase):
    @staticmethod
    def _settings(hover_duration_s: float = 4.0) -> tuple[QualitySettings, FlightSettings]:
        quality = QualitySettings(1.5, 0.65, 3.0, 0.05, 40, 300, 750)
        flight = FlightSettings(
            0.0, 0.0, -1.5, 0.0, 1.0, 30.0, 8.0,
            hover_duration_s, 1.0, 3.0, 2.0, 500, 4.0, 250,
        )
        return quality, flight

    def test_hover_duration_and_quality_schedule_use_flight_clock(self) -> None:
        clock = _ManualFlightClock()
        client = _ClockAdvancingClient(clock)
        faults = _RecordingFaults()
        quality, flight = self._settings()
        with tempfile.TemporaryDirectory() as directory:
            artifacts = RunArtifacts(Path(directory))
            started_wall_s = time.monotonic()
            try:
                _run_hover(
                    client, artifacts, "quality", quality, flight,
                    faults, False, clock,
                )
            finally:
                artifacts.close()
        self.assertLess(time.monotonic() - started_wall_s, 0.5)
        self.assertGreaterEqual(clock.time_s, flight.hover_duration_s)
        self.assertEqual(faults.calls, [("quality", "apply"), ("quality", "clear")])
        scheduled = {
            event["event"]: event["hover_elapsed_sim_s"]
            for event in artifacts.events if "scheduled_sim_s" in event
        }
        self.assertGreaterEqual(scheduled["quality_apply"], 2.0)
        self.assertGreaterEqual(scheduled["quality_clear"], 3.0)

    def test_hover_events_expose_absolute_gazebo_window(self) -> None:
        clock = _ManualFlightClock()
        clock.source = "gazebo"
        clock.time_s = 12.5
        client = _ClockAdvancingClient(clock)
        quality, flight = self._settings(hover_duration_s=0.2)
        with tempfile.TemporaryDirectory() as directory:
            artifacts = RunArtifacts(Path(directory))
            try:
                _run_hover(
                    client, artifacts, "nominal", quality, flight,
                    _RecordingFaults(), False, clock,
                )
            finally:
                artifacts.close()
        start_s, end_s = _gazebo_hover_window(artifacts.events)
        self.assertAlmostEqual(float(start_s), 12.5)
        self.assertGreaterEqual(float(end_s), 12.7)
        started = next(event for event in artifacts.events if event["event"] == "hover_started")
        self.assertEqual(started["sim_clock_reset_counter"], 0)

    def test_invalid_gazebo_hover_window_falls_back(self) -> None:
        events = [
            {"event": "hover_started", "clock_source": "gazebo", "sim_time_s": 10.0, "sim_clock_reset_counter": 0},
            {"event": "hover_finished", "clock_source": "gazebo", "sim_time_s": 20.0, "sim_clock_reset_counter": 1},
        ]
        self.assertEqual(_gazebo_hover_window(events), (None, None))

    def test_control_settle_and_tracking_loss_threshold_use_flight_clock(self) -> None:
        clock = _ManualFlightClock()
        client = _ClockAdvancingClient(clock)
        _wait_flight_duration(client, clock, 1.0)
        self.assertGreaterEqual(clock.time_s, 1.0)
        initial_polls = client.poll_count
        client.latest_state = types.SimpleNamespace(
            runtime_mode=1, tracking_state=4, armed=True,
            px4_main_mode=6, px4_sub_mode=0,
            received_monotonic_s=time.monotonic(),
        )
        quality, flight = self._settings(hover_duration_s=1.0)
        with tempfile.TemporaryDirectory() as directory:
            artifacts = RunArtifacts(Path(directory))
            try:
                outcome = _run_hover(
                    client, artifacts, "nominal", quality, flight,
                    _RecordingFaults(), True, clock,
                )
            finally:
                artifacts.close()
        self.assertGreater(client.poll_count, initial_polls)
        self.assertTrue(outcome.tracking_loss_threshold_exceeded)

    def test_tracking_warmup_uses_sim_time_with_wall_safety_limit(self) -> None:
        clock = _ManualFlightClock()
        client = _ClockAdvancingClient(clock)
        client.latest_state = types.SimpleNamespace(
            runtime_mode=1, tracking_state=5,
            received_monotonic_s=time.monotonic(),
        )
        state = _wait_for_tracking_stable(client, clock, 1.0, 0.5)
        self.assertIs(state, client.latest_state)
        self.assertGreaterEqual(clock.time_s, 1.0)

        frozen_clock = _ManualFlightClock()
        frozen_client = _ClockAdvancingClient(frozen_clock)
        frozen_client.clock = types.SimpleNamespace(time_s=0.0)
        frozen_client.latest_state = client.latest_state
        with self.assertRaisesRegex(TimeoutError, "wall time"):
            _wait_for_tracking_stable(frozen_client, frozen_clock, 1.0, 0.02)

    def test_landing_wall_timeout_tracks_slow_gazebo_rate(self) -> None:
        clock = _ManualFlightClock(rate=0.2)
        clock.source = "gazebo"
        self.assertAlmostEqual(_flight_stage_wall_timeout(30.0, clock), 150.0)
        clock.rate = 2.0
        self.assertAlmostEqual(_flight_stage_wall_timeout(30.0, clock), 30.0)


class MetricsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.options = AnalysisOptions()
        self.truth = [
            PositionSample(float(index) * 0.1, 0.01, -0.02, -1.5)
            for index in range(301)
        ]

    def test_percentile_and_error_summary(self) -> None:
        self.assertEqual(percentile([0.0, 1.0, 2.0, 3.0, 4.0], 95.0), 3.8)
        summary = error_summary([3.0, 4.0])
        self.assertEqual(summary["count"], 2)
        self.assertAlmostEqual(float(summary["rmse"]), math.sqrt(12.5))
        self.assertEqual(summary["max"], 4.0)

    def test_smart_drone_latency_uses_measurement_window(self) -> None:
        records = [
            {"tag": "odom_ts", "timing": 1, "cam_ns": 9_000_000_000, "total_ms": 5.0},
            {"tag": "odom_ts", "timing": 1, "cam_ns": 10_000_000_000, "total_ms": 10.0},
            {"tag": "odom_ts", "timing": 1, "cam_ns": 11_000_000_000, "total_ms": 20.0,
             "render_transport_ms": 12.0, "queue_ms": 3.0, "processing_ms": 4.0,
             "send_ms": 1.0, "pair_to_tx_ms": 8.0, "sim_age_ms": 5.0,
             "eye_skew_ms": 0.2},
            {"tag": "odom_ts", "timing": 1, "cam_ns": 12_000_000_000, "total_ms": 30.0},
            {"tag": "odom_ts", "timing": 1, "cam_ns": 13_000_000_000, "total_ms": 40.0},
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "smart_drone_frames.jsonl"
            path.write_text(
                "".join(json.dumps(record) + "\n" for record in records),
                encoding="utf-8",
            )
            summary = smart_drone_latency_summary(path, 10.0, 12.0)
        self.assertEqual(summary["sample_count"], 3)
        self.assertAlmostEqual(float(summary["latency_p95_ms"]), 29.0)
        self.assertEqual(summary["timing_p95_ms"]["wall_total_ms"], 29.0)
        self.assertEqual(summary["timing_p95_ms"]["render_transport_ms"], 12.0)
        self.assertEqual(summary["timing_sample_count"]["render_transport_ms"], 1)

    def test_smart_drone_latency_skips_invalid_records(self) -> None:
        lines = [
            '{"tag":"odom_ts","timing":1,"cam_ns":1000000000,"total_ms":25.0}',
            '{"tag":"slam_dfx","timing":1,"cam_ns":1000000000,"total_ms":90.0}',
            '{"tag":"odom_ts","timing":0,"cam_ns":1000000000,"total_ms":80.0}',
            '{"tag":"odom_ts","timing":1,"cam_ns":1000000000,"total_ms":-1.0}',
            '{"tag":"odom_ts","timing":1,"cam_ns":1000000000,"total_ms":NaN}',
            '{"tag":"odom_ts","timing":1,"total_ms":15.0}',
            "not json",
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "smart_drone_frames.jsonl"
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            all_records = smart_drone_latency_summary(path)
            windowed = smart_drone_latency_summary(path, 0.0, 2.0)
        self.assertEqual(all_records["sample_count"], 2)
        self.assertEqual(all_records["latency_p95_ms"], 24.5)
        self.assertEqual(all_records["timing_p95_ms"]["wall_total_ms"], 24.5)
        self.assertEqual(windowed["sample_count"], 1)
        self.assertEqual(windowed["latency_p95_ms"], 25.0)

    def test_smart_drone_latency_empty_and_invalid_window(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "smart_drone_frames.jsonl"
            path.write_text("{}\n", encoding="utf-8")
            summary = smart_drone_latency_summary(path)
            self.assertEqual(summary["sample_count"], 0)
            self.assertIsNone(summary["latency_p95_ms"])
            self.assertTrue(all(
                value is None for value in summary["timing_p95_ms"].values()
            ))
            with self.assertRaisesRegex(ValueError, "window end"):
                smart_drone_latency_summary(path, 2.0, 1.0)

    def test_hover_metrics_use_ground_truth_and_target(self) -> None:
        metrics = hover_errors(self.truth, self.options, 0.0, 30.0)
        self.assertAlmostEqual(float(metrics["horizontal_m"]["rmse"]), math.hypot(0.01, 0.02))
        self.assertAlmostEqual(float(metrics["vertical_m"]["rmse"]), 0.0)

    def test_estimator_alignment_interpolates_timestamps(self) -> None:
        estimate = [
            PositionSample(sample.timestamp_s + 0.025, sample.x + 0.1, sample.y, sample.z + 0.05)
            for sample in self.truth[:-1]
        ]
        metrics = aligned_errors(self.truth, estimate, 0.0, 30.0)
        self.assertAlmostEqual(float(metrics["horizontal_m"]["rmse"]), 0.1, places=6)
        self.assertAlmostEqual(float(metrics["vertical_m"]["rmse"]), 0.05, places=6)

    def test_frd_visual_positions_use_truth_attitude_and_origin(self) -> None:
        truth = [
            PositionSample(0.0, 10.0, 20.0, -3.0),
            PositionSample(1.0, 10.0, 21.0, -3.5),
        ]
        visual = [
            PositionSample(0.0, 5.0, -2.0, 0.4, 100.0, POSE_FRAME_FRD),
            PositionSample(1.0, 6.0, -2.0, -0.1, 100.0, POSE_FRAME_FRD),
        ]
        yaw_ninety = math.sqrt(0.5)
        attitude = [AttitudeSample(0.0, yaw_ninety, 0.0, 0.0, yaw_ninety)]
        aligned, details = _align_visual_positions(truth, visual, attitude)
        self.assertEqual(details["source_pose_frame_name"], "FRD")
        self.assertEqual(aligned[1].pose_frame, POSE_FRAME_NED)
        self.assertAlmostEqual(aligned[1].x, 10.0, places=6)
        self.assertAlmostEqual(aligned[1].y, 21.0, places=6)
        self.assertAlmostEqual(aligned[1].z, -3.5, places=6)
        self.assertEqual(aligned_errors(truth, aligned, 0.0, 1.0)["spatial_m"]["max"], 0.0)

    def test_ned_visual_positions_use_truth_origin_without_rotation(self) -> None:
        truth = [PositionSample(0.0, 1.0, 2.0, -3.0), PositionSample(1.0, 1.5, 1.0, -2.5)]
        visual = [
            PositionSample(0.0, 50.0, -20.0, 7.0, 100.0, POSE_FRAME_NED),
            PositionSample(1.0, 50.5, -21.0, 7.5, 100.0, POSE_FRAME_NED),
        ]
        aligned, details = _align_visual_positions(truth, visual, [])
        self.assertEqual(details["source_pose_frame_name"], "NED")
        self.assertEqual(aligned_errors(truth, aligned, 0.0, 1.0)["spatial_m"]["max"], 0.0)

    def test_missing_visual_pose_frame_is_not_compared_as_ned(self) -> None:
        truth = [PositionSample(0.0, 1.0, 2.0, -3.0)]
        visual = [PositionSample(0.0, 1.0, 2.0, -3.0, 100.0)]
        aligned, details = _align_visual_positions(truth, visual, [])
        self.assertEqual(aligned, [])
        self.assertEqual(details["status"], "unavailable")
        self.assertIn("missing or unsupported", details["reason"])

    def test_hover_window_autodetection(self) -> None:
        samples = [PositionSample(0.0, 0.0, 0.0, 0.0)] + self.truth + [PositionSample(31.0, 0.0, 0.0, 0.0)]
        start, end, source = detect_hover_window(samples, self.options)
        self.assertEqual(source, "target_altitude")
        self.assertEqual(start, 0.0)
        self.assertEqual(end, 30.0)

    def test_hover_window_autodetection_excludes_settling_boundary(self) -> None:
        samples = [
            PositionSample(0.0, 0.0, 0.0, -1.201),
            PositionSample(0.1, 0.0, 0.0, -1.30),
            PositionSample(0.2, 0.0, 0.0, -1.34),
            PositionSample(0.3, 0.0, 0.0, -1.36),
            PositionSample(0.4, 0.0, 0.0, -1.45),
            PositionSample(0.5, 0.0, 0.0, -1.50),
        ]
        start, _, _ = detect_hover_window(samples, self.options)
        self.assertEqual(start, 0.3)
        self.assertLessEqual(abs(samples[3].z - self.options.target_z), 0.15)

    def test_explicit_hover_window_separates_settling_accuracy(self) -> None:
        timestamps = [index * 100_000 for index in range(51)]
        altitude = [
            -1.21 - min(index, 30) * (0.29 / 30.0)
            for index in range(51)
        ]
        zeros = [0.0] * len(timestamps)
        truth = FakeDataset("vehicle_local_position_groundtruth", {
            "timestamp": timestamps, "x": zeros, "y": zeros, "z": altitude,
        })
        estimate = FakeDataset("vehicle_local_position", {
            "timestamp": timestamps, "x": zeros, "y": zeros, "z": altitude,
        })
        status = FakeDataset("vehicle_status", {
            "timestamp": [0], "nav_state": [14],
            "arming_state": [2], "failsafe": [0],
        })
        fake_module = types.SimpleNamespace(ULog=lambda _: FakeUlog([truth, estimate, status]))
        options = AnalysisOptions(
            profile="control", hover_start_s=3.0, hover_end_s=5.0,
        )
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(sys.modules, {"pyulog": fake_module}):
            metrics = analyze_ulog(Path(directory) / "fixture.ulg", Path(directory), options)
        self.assertEqual(metrics["hover_window"]["source"], "explicit")
        self.assertEqual(metrics["settling_window"]["source"], "target_approach")
        self.assertAlmostEqual(float(metrics["hover_accuracy"]["vertical_m"]["max"]), 0.0)
        self.assertGreater(float(metrics["settling_accuracy"]["vertical_m"]["max"]), 0.28)

    def test_land_command_ends_hover_before_descent(self) -> None:
        commands = FakeDataset(
            "vehicle_command",
            {"timestamp": [10_000_000, 31_000_000], "command": [400, 21]},
        )
        self.assertEqual(_land_command_time(FakeUlog([commands]), 0.0, 32.0), 31.0)

    def test_flight_status_excludes_transition_at_hover_end(self) -> None:
        status = FakeDataset(
            "vehicle_status",
            {
                "timestamp": [0, 30_000_000],
                "nav_state": [14, 18],
                "arming_state": [2, 2],
                "failsafe": [0, 0],
            },
        )
        diagnostics = _flight_diagnostics(FakeUlog([status]), 0.0, 30.0)
        self.assertTrue(diagnostics["offboard_at_window_start"])
        self.assertEqual(diagnostics["offboard_exit_count"], 0)

    def test_impulse_recovery_requires_stable_reentry(self) -> None:
        samples: list[PositionSample] = []
        for index in range(51):
            timestamp = index * 0.1
            x = 0.4 if 1.2 <= timestamp < 2.0 else 0.0
            samples.append(PositionSample(timestamp, x, 0.0, -1.5))
        recovery = calculate_recovery_time(samples, self.options, event_time_s=1.0, end_s=5.0)
        self.assertAlmostEqual(float(recovery), 1.0)

    def test_ulog_position_and_visual_health_fields(self) -> None:
        dataset = FakeDataset(
            "vehicle_visual_odometry",
            {
                "timestamp": [10_000, 60_000, 110_000],
                "timestamp_sample": [0, 50_000, 100_000],
                "position[0]": [0.0, 0.1, 0.2],
                "position[1]": [0.0, 0.0, 0.0],
                "position[2]": [-1.5, -1.5, -1.5],
                "quality": [100, -1, 100],
                "pose_frame": [POSE_FRAME_FRD] * 3,
                "reset_counter": [0, 0, 1],
            },
        )
        samples = _extract_position_samples(dataset)
        health = _visual_health(dataset, samples, 0.0, 0.1)
        self.assertEqual(len(samples), 3)
        self.assertEqual(samples[0].pose_frame, POSE_FRAME_FRD)
        self.assertAlmostEqual(float(health["frequency_hz"]), 30.0)
        self.assertAlmostEqual(float(health["usable_frequency_hz"]), 20.0)
        self.assertAlmostEqual(float(health["availability"]), 2.0 / 3.0)
        self.assertAlmostEqual(float(health["latency_p95_ms"]), 10.0)
        self.assertEqual(health["reset_count"], 1)
        self.assertEqual(health["pose_frame_names"], ["FRD"])

    def test_flight_status_uses_pre_window_offboard_baseline(self) -> None:
        status = FakeDataset(
            "vehicle_status",
            {
                "timestamp": [5_000_000, 20_000_000, 35_000_000],
                "nav_state": [14, 3, 3],
                "arming_state": [2, 2, 1],
                "failsafe": [0, 0, 0],
            },
        )
        diagnostics = _flight_diagnostics(FakeUlog([status]), 10.0, 30.0)
        self.assertTrue(diagnostics["offboard_seen"])
        self.assertTrue(diagnostics["offboard_at_window_start"])
        self.assertEqual(diagnostics["offboard_exit_count"], 1)

    def test_complete_ulog_analysis_fixture_passes_thresholds(self) -> None:
        timestamps = [index * 50_000 for index in range(601)]
        east_displacement = [index * 0.05 / 600.0 for index in range(601)]
        positions = {
            "position[0]": [0.01] * len(timestamps),
            "position[1]": [-0.01 + value for value in east_displacement],
            "position[2]": [-1.5] * len(timestamps),
        }
        truth = FakeDataset("vehicle_local_position_groundtruth", {"timestamp": timestamps, **positions})
        estimate = FakeDataset(
            "vehicle_local_position",
            {"timestamp": timestamps, "x": positions["position[0]"], "y": positions["position[1]"], "z": positions["position[2]"]},
        )
        visual = FakeDataset(
            "vehicle_visual_odometry",
            {
                "timestamp": [value + 10_000 for value in timestamps],
                "timestamp_sample": timestamps,
                "position[0]": [3.0 + value for value in east_displacement],
                "position[1]": [-4.0] * len(timestamps),
                "position[2]": [2.0] * len(timestamps),
                "quality": [100] * len(timestamps),
                "pose_frame": [POSE_FRAME_FRD] * len(timestamps),
                "reset_counter": [0] * len(timestamps),
            },
        )
        yaw_ninety = math.sqrt(0.5)
        attitude = FakeDataset(
            "vehicle_attitude_groundtruth",
            {
                "timestamp": timestamps,
                "q[0]": [yaw_ninety] * len(timestamps),
                "q[1]": [0.0] * len(timestamps),
                "q[2]": [0.0] * len(timestamps),
                "q[3]": [yaw_ninety] * len(timestamps),
            },
        )
        status = FakeDataset(
            "vehicle_status",
            {"timestamp": [0], "nav_state": [14], "arming_state": [2], "failsafe": [0]},
        )
        fake_module = types.SimpleNamespace(ULog=lambda _: FakeUlog([
            truth, attitude, estimate, visual, status,
        ]))
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(sys.modules, {"pyulog": fake_module}):
            output = Path(directory)
            metrics = analyze_ulog(output / "fixture.ulg", output, self.options)
            self.assertTrue(metrics["acceptance"]["passed"])
            self.assertTrue(metrics["acceptance"]["complete"])
            self.assertEqual(metrics["visual_alignment"]["source_pose_frame_name"], "FRD")
            self.assertAlmostEqual(float(metrics["visual_accuracy"]["spatial_m"]["max"]), 0.0, places=6)
            self.assertTrue((output / "metrics.json").exists())
            self.assertTrue((output / "trajectory.csv").exists())

    def test_control_analysis_does_not_require_visual_odometry(self) -> None:
        timestamps = [index * 50_000 for index in range(601)]
        truth = FakeDataset("vehicle_local_position_groundtruth", {
            "timestamp": timestamps, "position[0]": [0.0] * len(timestamps),
            "position[1]": [0.0] * len(timestamps),
            "position[2]": [-1.5] * len(timestamps),
        })
        estimate = FakeDataset("vehicle_local_position", {
            "timestamp": timestamps, "x": [0.0] * len(timestamps),
            "y": [0.0] * len(timestamps), "z": [-1.5] * len(timestamps),
        })
        status = FakeDataset("vehicle_status", {
            "timestamp": [0], "nav_state": [14],
            "arming_state": [2], "failsafe": [0],
        })
        fake_module = types.SimpleNamespace(ULog=lambda _: FakeUlog([truth, estimate, status]))
        options = AnalysisOptions(profile="control")
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(sys.modules, {"pyulog": fake_module}):
            metrics = analyze_ulog(Path(directory) / "fixture.ulg", Path(directory), options)
        self.assertIsNone(metrics["topics"]["visual_odometry"])
        self.assertTrue(metrics["acceptance"]["complete"])
        self.assertTrue(metrics["acceptance"]["passed"])

    def test_truth_analysis_requires_external_visual_odometry(self) -> None:
        timestamps = [index * 50_000 for index in range(601)]
        truth = FakeDataset("vehicle_local_position_groundtruth", {
            "timestamp": timestamps, "position[0]": [0.0] * len(timestamps),
            "position[1]": [0.0] * len(timestamps),
            "position[2]": [-1.5] * len(timestamps),
        })
        estimate = FakeDataset("vehicle_local_position", {
            "timestamp": timestamps, "x": [0.0] * len(timestamps),
            "y": [0.0] * len(timestamps), "z": [-1.5] * len(timestamps),
        })
        fake_module = types.SimpleNamespace(ULog=lambda _: FakeUlog([truth, estimate]))
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(sys.modules, {"pyulog": fake_module}):
            with self.assertRaisesRegex(ValueError, "visual odometry"):
                analyze_ulog(
                    Path(directory) / "fixture.ulg", Path(directory),
                    AnalysisOptions(profile="truth"),
                )

    def test_loss_analysis_does_not_require_thirty_second_hover(self) -> None:
        timestamps = [index * 50_000 for index in range(401)]
        positions = {
            "position[0]": [0.0] * len(timestamps),
            "position[1]": [0.0] * len(timestamps),
            "position[2]": [-1.5] * len(timestamps),
        }
        truth = FakeDataset("vehicle_local_position_groundtruth", {"timestamp": timestamps, **positions})
        estimate = FakeDataset("vehicle_local_position", {
            "timestamp": timestamps, "x": positions["position[0]"],
            "y": positions["position[1]"], "z": positions["position[2]"],
        })
        visual = FakeDataset("vehicle_visual_odometry", {
            "timestamp": [value + 10_000 for value in timestamps],
            "timestamp_sample": timestamps, **positions,
            "quality": [100] * len(timestamps),
            "pose_frame": [POSE_FRAME_NED] * len(timestamps),
            "reset_counter": [0] * len(timestamps),
        })
        status = FakeDataset("vehicle_status", {
            "timestamp": [0], "nav_state": [14],
            "arming_state": [2], "failsafe": [0],
        })
        fake_module = types.SimpleNamespace(ULog=lambda _: FakeUlog([truth, estimate, visual, status]))
        options = AnalysisOptions(profile="vision", scenario="loss")
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(sys.modules, {"pyulog": fake_module}):
            metrics = analyze_ulog(Path(directory) / "fixture.ulg", Path(directory), options)
        check_names = {check["name"] for check in metrics["acceptance"]["checks"]}
        self.assertNotIn("hover_duration_s", check_names)
        self.assertTrue(metrics["acceptance"]["passed"])


class FakeDataset:
    def __init__(self, name: str, data: dict[str, list[float | int]], multi_id: int = 0) -> None:
        self.name = name
        self.data = data
        self.multi_id = multi_id


class FakeUlog:
    def __init__(self, datasets: list[FakeDataset]) -> None:
        self.data_list = datasets


class RunnerIntegrationTest(unittest.TestCase):
    @staticmethod
    def _free_port() -> int:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        port = int(probe.getsockname()[1])
        probe.close()
        return port

    @staticmethod
    def _events(output: Path) -> list[str]:
        return [
            json.loads(line)["event"]
            for line in (output / "run.jsonl").read_text().splitlines()
        ]

    @staticmethod
    def _loss_command(
        port: int,
        output: Path,
        fake_options: str,
        *,
        hover_duration_s: float = 2.2,
        land_timeout_s: float = 1.0,
    ) -> list[str]:
        fake_command = (
            f"{sys.executable} tests/sitl/fake_smartdrone_server.py --port {port} "
            f"{fake_options}"
        )
        return [
            sys.executable, "scripts/run_hover_sitl.py", "--profile", "vision",
            "--scenario", "loss", "--flight-clock", "steady",
            "--command-port", str(port),
            "--smart-drone-command", fake_command, "--output-dir", str(output),
            "--launcher-warmup-s", "0.05",
            "--hover-duration-s", str(hover_duration_s),
            "--short-blackout-at-s", "0.05", "--long-blackout-at-s", "0.9",
            "--tracking-stable-s", "0.05", "--ascent-timeout-s", "2",
            "--state-timeout-s", "2", "--command-timeout-s", "1",
            "--land-timeout-s", str(land_timeout_s),
        ]

    @staticmethod
    def _write_fake_pyulog(directory: Path) -> None:
        (directory / "pyulog.py").write_text(
            """\
class Dataset:
    def __init__(self, name, data):
        self.name = name
        self.data = data
        self.multi_id = 0

class ULog:
    def __init__(self, _path):
        timestamps = [index * 50000 for index in range(601)]
        zeros = [0.0] * len(timestamps)
        altitude = [-1.5] * len(timestamps)
        truth = Dataset("vehicle_local_position_groundtruth", {
            "timestamp": timestamps, "x": zeros, "y": zeros, "z": altitude,
        })
        estimate = Dataset("vehicle_local_position", {
            "timestamp": timestamps, "x": zeros, "y": zeros, "z": altitude,
        })
        status_times = [index * 1000000 for index in range(31)]
        status = Dataset("vehicle_status", {
            "timestamp": status_times, "nav_state": [14] * len(status_times),
            "arming_state": [2] * len(status_times),
            "failsafe": [0] * len(status_times),
        })
        self.data_list = [truth, estimate, status]
""",
            encoding="utf-8",
        )

    def test_truth_profile_launch_environment(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            args = types.SimpleNamespace(
                profile="truth",
                seed=1,
                gz_partition="smartdrone_sitl",
                gz_world="alternate_world",
                gz_model="alternate_vehicle",
                tracking_loss_land_ms=500,
                sim_watchdog_scale=1.0,
                sim_config=None,
            )
            environment = _profile_launch_environment(args, Path(directory))
            self.assertEqual(environment["SMART_DRONE_AUTO_MODE"], "idle")
            self.assertEqual(environment["SMART_DRONE_PX4_POSE_OUTPUT_MODE"], "position_velocity")
            self.assertEqual(environment["SMART_DRONE_OFFBOARD_REQUIRES_VISION"], "0")
            self.assertEqual(environment["SMART_DRONE_SIM_WORLD"], "alternate_world")
            self.assertEqual(environment["SMART_DRONE_SIM_MODEL"], "alternate_vehicle")
            self.assertTrue(environment["SMART_DRONE_SIM_CONFIG"].endswith("sim/px4_gz/config/smartdrone_sim.yaml"))

    def test_run_mode_and_provenance_environment_fail_closed(self) -> None:
        args = types.SimpleNamespace(
            px4_command="px4", smart_drone_command="smart", profile="control",
            scenario="nominal", seed=3,
        )
        quality = QualitySettings(0.0, 1.0, 0.0, 0.0, 0, 300, 750)
        flight = FlightSettings(
            0.0, 0.0, -1.5, 0.0, 1.0, 30.0, 0.0, 30.0,
            10.0, 20.0, 2.0, 500, 4.0, 250,
        )
        contract = _run_contract(args, quality, flight)
        self.assertEqual(_run_mode(args), "managed")
        with tempfile.TemporaryDirectory() as directory:
            environment = {
                "SMART_DRONE_SITL_RUN_ID": "stale",
                "SMART_DRONE_PX4_ATTESTATION_FILE": "stale-px4",
                "SMART_DRONE_ATTESTATION_FILE": "stale-smart",
            }
            output = Path(directory)
            _configure_provenance_environment(environment, output, contract)
            self.assertEqual(environment["SMART_DRONE_SITL_RUN_ID"], contract.run_id)
            self.assertEqual(
                environment["SMART_DRONE_PX4_ATTESTATION_FILE"],
                str((output / "px4_attestation.json").resolve()),
            )
            attached_args = types.SimpleNamespace(
                px4_command="", smart_drone_command="smart",
            )
            self.assertEqual(_run_mode(attached_args), "attached")
            attached_contract = RunContract(
                contract.run_id, "attached", "control", "nominal", 3,
                quality, flight,
            )
            _configure_provenance_environment(environment, output, attached_contract)
            self.assertNotIn("SMART_DRONE_SITL_RUN_ID", environment)

    def test_managed_runner_generates_and_gates_provenance(self) -> None:
        port = self._free_port()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "run"
            environment_output = root / "environment.json"
            self._write_fake_pyulog(root)
            px4_command = f"{sys.executable} tests/sitl/fake_px4_launcher.py"
            smart_command = (
                f"{sys.executable} tests/sitl/fake_smartdrone_server.py "
                f"--port {port} --environment-output {environment_output}"
            )
            command = [
                sys.executable, "scripts/run_hover_sitl.py", "--profile", "control",
                "--scenario", "nominal", "--flight-clock", "steady",
                "--command-port", str(port), "--px4-command", px4_command,
                "--smart-drone-command", smart_command,
                "--output-dir", str(output), "--launcher-warmup-s", "0.05",
                "--hover-duration-s", "0.1", "--control-ascent-s", "0",
                "--state-timeout-s", "2", "--command-timeout-s", "2",
                "--land-timeout-s", "2",
            ]
            environment = dict(os.environ)
            environment["PYTHONPATH"] = (
                str(root) + os.pathsep + environment.get("PYTHONPATH", "")
            )
            environment.update({
                "SMART_DRONE_GZ_RENDERING_REQUESTED": "auto",
                "SMART_DRONE_GZ_RENDERING_SELECTED": "hardware",
                "SMART_DRONE_GZ_RENDERING_REASON": "hardware_egl",
                "SMART_DRONE_GZ_RENDERING_VERIFIED": "1",
                "SMART_DRONE_GZ_RENDER_NODE": "/dev/dri/renderD128",
                "SMART_DRONE_GZ_RENDERER": "AMD Radeon RX 6800",
            })
            result = subprocess.run(
                command, cwd=REPO_ROOT, env=environment,
                capture_output=True, text=True, timeout=15.0,
            )

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            required = (
                "runner_metrics.json", "px4.ulg", "px4_attestation.json",
                "smart_drone_attestation.json", "run_manifest.json", "metrics.json",
            )
            self.assertTrue(all((output / name).is_file() for name in required))
            provenance = verify_run_manifest(
                output / "run_manifest.json", output / "px4.ulg",
                expected_profile="control", expected_scenario="nominal",
                expected_seed=1, require_managed=True,
            )
            self.assertTrue(provenance.verified)
            metrics = json.loads((output / "metrics.json").read_text())
            checks = {item["name"]: item for item in metrics["acceptance"]["checks"]}
            for name in (
                "run_provenance_verified", "runner_outcome_completed",
                "scenario_execution_confirmed",
            ):
                self.assertTrue(checks[name]["passed"])
            self.assertTrue(metrics["acceptance"]["passed"])
            launched = json.loads(environment_output.read_text())
            self.assertEqual(launched["SMART_DRONE_SITL_RUN_ID"], provenance.run_id)
            self.assertEqual(
                Path(launched["SMART_DRONE_ATTESTATION_FILE"]),
                output / "smart_drone_attestation.json",
            )
            expected_rendering = {
                "requested": "auto",
                "selected": "hardware",
                "reason": "hardware_egl",
                "render_node": "/dev/dri/renderD128",
                "renderer": "AMD Radeon RX 6800",
                "verified": True,
            }
            scenario = yaml.safe_load(
                (output / "scenario.yaml").read_text(encoding="utf-8"),
            )
            runner_metrics = json.loads(
                (output / "runner_metrics.json").read_text(encoding="utf-8"),
            )
            manifest = json.loads(
                (output / "run_manifest.json").read_text(encoding="utf-8"),
            )
            self.assertEqual(scenario["rendering"], expected_rendering)
            self.assertEqual(runner_metrics["rendering"], expected_rendering)
            self.assertEqual(
                manifest["runner"]["rendering"], expected_rendering,
            )
            self.assertNotIn("rendering", manifest["artifacts"])
            self.assertFalse((output / "metrics.json.tmp").exists())

    def test_runner_uses_frame_jsonl_for_formal_latency(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "smart_drone_frames.jsonl"
            path.write_text(
                '{"tag":"odom_ts","timing":1,"cam_ns":10000000000,"total_ms":20}\n'
                '{"tag":"odom_ts","timing":1,"cam_ns":11000000000,"total_ms":40}\n'
                '{"tag":"odom_ts","timing":1,"cam_ns":13000000000,"total_ms":500}\n',
                encoding="utf-8",
            )
            metrics = {
                "hover_window": {"start_s": 10.0, "end_s": 12.0},
                "visual_odometry": {"latency_p95_ms": 0.0},
                "acceptance": {"checks": [{
                    "name": "visual_latency_p95_ms", "actual": 0.0,
                    "relation": "max", "limit": 80.0, "passed": True,
                }]},
            }
            _apply_smart_drone_latency(metrics, path)
            visual = metrics["visual_odometry"]
            self.assertEqual(visual["ulog_transport_latency_p95_ms"], 0.0)
            self.assertEqual(visual["latency_source"], "smart_drone_frames_jsonl.wall_total_ms")
            self.assertEqual(visual["latency_sample_count"], 2)
            self.assertAlmostEqual(visual["latency_p95_ms"], 39.0)
            self.assertEqual(
                visual["timing_breakdown_p95_ms"]["wall_total_ms"], 39.0,
            )
            self.assertTrue(metrics["acceptance"]["checks"][0]["passed"])

    def test_existing_run_artifacts_are_not_overwritten(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "metrics.json").write_text("{}", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "existing run artifacts"):
                _prepare_run_output(output)

    def test_matrix_invocation_files_are_allowed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "matrix_invocation.json").touch()
            (output / "matrix_runner.log").touch()
            _prepare_run_output(output)

    def test_quality_scenario_defaults_apply_real_degradation(self) -> None:
        with mock.patch.object(sys, "argv", ["run_hover_sitl.py", "--profile", "vision", "--scenario", "quality"]):
            quality, flight = _settings(_parse_args())
        self.assertGreater(quality.blur_sigma, 0.0)
        self.assertLess(quality.brightness, 1.0)
        self.assertGreater(quality.noise_stddev, 0.0)
        self.assertGreater(quality.drop_rate, 0.0)
        self.assertGreater(quality.delay_ms, 0)
        self.assertEqual(flight.impulse_force_n, 4.0)
        self.assertEqual(flight.impulse_duration_ms, 250)

    def test_quality_and_blackout_use_fault_state_without_command(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            faults = FaultCommands(
                "", "true", output / "faults.log", output / "fault_state.json",
            )
            initial = json.loads((output / "fault_state.json").read_text())
            self.assertEqual(initial["brightness"], 1.0)
            self.assertEqual(initial["blur_sigma"], 0.0)
            self.assertEqual(initial["drop_rate"], 0.0)
            applied = faults.execute("quality", {
                "action": "apply", "blur_sigma": 1.5, "brightness": 0.65,
                "noise_stddev": 3.0, "drop_rate": 0.05, "delay_ms": 40,
                "blackout_ms": 0,
            })
            self.assertTrue(applied)
            state = json.loads((output / "fault_state.json").read_text())
            self.assertEqual(state["action"], "apply")
            self.assertEqual(state["generation"], 2)
            faults.execute("blackout", _blackout_values(750))
            state = json.loads((output / "fault_state.json").read_text())
            self.assertEqual(state["action"], "blackout")
            self.assertEqual(state["blackout_ms"], 750)
            self.assertEqual(state["generation"], 3)
            self.assertEqual(state["brightness"], 1.0)
            self.assertEqual(state["delay_ms"], 0)
            faults.execute("quality", {
                "action": "clear", "blur_sigma": 1.5, "brightness": 0.65,
                "noise_stddev": 3.0, "drop_rate": 0.05, "delay_ms": 40,
                "blackout_ms": 0,
            })
            state = json.loads((output / "fault_state.json").read_text())
            self.assertEqual(state["generation"], 4)
            self.assertEqual(state["blur_sigma"], 0.0)
            self.assertEqual(state["brightness"], 1.0)
            self.assertEqual(state["noise_stddev"], 0.0)
            self.assertEqual(state["drop_rate"], 0.0)
            self.assertEqual(state["delay_ms"], 0)

    def test_frame_artifact_contains_smartdrone_json_diagnostics(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            artifacts = RunArtifacts(output)
            (output / "smart_drone.log").write_text(
                "startup text\n"
                '{"tag":"slam_dfx","frame":7,"quality":2}\n'
                '{"tag":"odom_ts","frame":7,"total_ms":18.0}\n'
                '{"tag":"unrelated","frame":7}\n',
                encoding="utf-8",
            )
            artifacts.collect_smart_drone_frames(output / "smart_drone.log")
            artifacts.close()
            records = [
                json.loads(line)
                for line in (output / "smart_drone_frames.jsonl").read_text().splitlines()
            ]
            self.assertEqual([record["tag"] for record in records], ["slam_dfx", "odom_ts"])
            self.assertEqual(artifacts.frame_record_count, 2)

    def test_default_impulse_targets_partition_and_keeps_heartbeat(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            fake_bin = output / "bin"
            fake_bin.mkdir()
            gz_log = output / "gz.log"
            fake_gz = fake_bin / "gz"
            fake_gz.write_text(
                "#!/usr/bin/env bash\n"
                "printf '%s|%s\\n' \"$GZ_PARTITION\" \"$*\" >> \"$FAKE_GZ_LOG\"\n"
                "if [[ \"$*\" == *--json-output* ]]; then\n"
                "  printf '%s\\n' '{\"sim\":{\"sec\":1,\"nsec\":0}}'\n"
                "  sleep 0.01\n"
                "  printf '%s\\n' '{\"sim\":{\"sec\":1,\"nsec\":10000000}}'\n"
                "  sleep 0.01\n"
                "  printf '%s\\n' '{\"sim\":{\"sec\":1,\"nsec\":30000000}}'\n"
                "fi\n",
                encoding="utf-8",
            )
            fake_gz.chmod(0o755)
            environment = dict(os.environ)
            environment.update({
                "PATH": f"{fake_bin}:{environment['PATH']}",
                "GZ_PARTITION": "runner_partition",
                "FAKE_GZ_LOG": str(gz_log),
            })
            faults = FaultCommands(
                "", "", output / "faults.log", output / "fault_state.json",
                environment=environment, world_name="test_world", model_name="test_model",
            )
            heartbeats = 0

            def keepalive() -> None:
                nonlocal heartbeats
                heartbeats += 1
                time.sleep(0.002)

            self.assertTrue(faults.execute(
                "impulse", {"action": "impulse", "force_n": 4.0, "duration_ms": 20}, keepalive,
            ))
            self.assertGreater(heartbeats, 0)
            lines = gz_log.read_text().splitlines()
            self.assertEqual(len(lines), 4)
            self.assertTrue(all(line.startswith("runner_partition|") for line in lines))
            self.assertIn("/world/test_world/clock", lines[1])
            self.assertIn("--json-output", lines[1])
            self.assertIn("/world/test_world/wrench/persistent", lines[2])
            self.assertIn("gz.msgs.EntityWrench", lines[2])
            self.assertIn('name: "test_model"', lines[2])
            self.assertIn("x: 4.0", lines[2])
            self.assertIn("/world/test_world/wrench/clear", lines[3])
            self.assertIn("gz.msgs.Entity", lines[3])
            self.assertIn('name: "test_model"', lines[3])
            self.assertNotIn("entity {", lines[3])
            fault_log = json.loads((output / "faults.log").read_text().splitlines()[-1])
            self.assertEqual(fault_log["status"], 0)
            self.assertEqual(fault_log["world_name"], "test_world")

    def test_impulse_command_timeout_covers_low_rtf_and_cleanup(self) -> None:
        environment: dict[str, str] = {}
        quality = {"kind": "quality"}
        short_impulse = {"kind": "impulse", "duration_ms": 250}
        long_impulse = {"kind": "impulse", "duration_ms": 5000}

        self.assertEqual(_fault_command_timeout_s(quality, environment), 10.0)
        self.assertEqual(_fault_command_timeout_s(short_impulse, environment), 35.0)
        self.assertEqual(_fault_command_timeout_s(long_impulse, environment), 115.0)
        self.assertEqual(
            _fault_command_timeout_s(
                short_impulse, {"SMART_DRONE_GZ_WRENCH_TIMEOUT_S": "42"},
            ),
            47.0,
        )

    def test_rendering_metadata_marks_probe_verification(self) -> None:
        environment = {
            "SMART_DRONE_GZ_RENDERING_REQUESTED": "auto",
            "SMART_DRONE_GZ_RENDERING_SELECTED": "hardware",
            "SMART_DRONE_GZ_RENDERING_REASON": "hardware_egl",
            "SMART_DRONE_GZ_RENDERING_VERIFIED": "1",
            "SMART_DRONE_GZ_RENDER_NODE": "/dev/dri/renderD128",
            "SMART_DRONE_GZ_RENDERER": "AMD Radeon RX 6800",
        }
        with mock.patch.dict(os.environ, environment, clear=False):
            rendering = _rendering_metadata()

        self.assertTrue(rendering["verified"])
        self.assertEqual(rendering["selected"], "hardware")
        self.assertEqual(rendering["render_node"], "/dev/dri/renderD128")

    def test_nominal_control_sequence_and_artifacts(self) -> None:
        port = self._free_port()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            environment_output = Path(directory) / "environment.json"
            fake_command = (
                f"{sys.executable} tests/sitl/fake_smartdrone_server.py --port {port} "
                f"--environment-output {environment_output}"
            )
            command = [
                sys.executable, "scripts/run_hover_sitl.py", "--profile", "control",
                "--scenario", "nominal", "--flight-clock", "steady",
                "--command-port", str(port),
                "--smart-drone-command", fake_command, "--output-dir", str(output),
                "--launcher-warmup-s", "0.05", "--hover-duration-s", "0.1",
                "--control-ascent-s", "0",
                "--state-timeout-s", "2", "--command-timeout-s", "2",
                "--land-timeout-s", "2",
            ]
            result = subprocess.run(command, cwd=REPO_ROOT, capture_output=True, text=True, timeout=10.0)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            events = self._events(output)
            ordered = ["command_offboard", "offboard_confirmed", "command_arm", "armed_confirmed", "command_move", "command_land", "disarmed_confirmed"]
            positions = [events.index(name) for name in ordered]
            self.assertEqual(positions, sorted(positions))
            for name in (
                "scenario.yaml", "fault_state.json", "smart_drone_frames.jsonl",
                "smart_drone_states.jsonl", "trajectory.csv", "runner_metrics.json",
                "metrics.json",
                "px4.log", "smart_drone.log",
            ):
                self.assertTrue((output / name).exists(), name)
            environment = json.loads(environment_output.read_text())
            self.assertEqual(environment["GZ_PARTITION"], "smartdrone_sitl")
            self.assertEqual(environment["GZ_IP"], "127.0.0.1")
            self.assertEqual(environment["SMART_DRONE_GZ_PARTITION"], "smartdrone_sitl")
            self.assertEqual(environment["SMART_DRONE_SIM_PROFILE"], "control")
            self.assertEqual(environment["SMART_DRONE_SIM_SEED"], "1")
            self.assertEqual(environment["SMART_DRONE_SIM_WORLD"], "smartdrone_hover")
            self.assertEqual(environment["SMART_DRONE_SIM_MODEL"], "smartdrone_x500")
            self.assertEqual(environment["SMART_DRONE_PX4_LOG_DIR"], str(output / "px4_sitl"))
            self.assertEqual(environment["SMART_DRONE_PX4_READY_FILE"], str(output / "px4_launcher.ready"))
            self.assertEqual(environment["SMART_DRONE_AUTO_MODE"], "idle")
            self.assertEqual(environment["SMART_DRONE_PX4_POSE_OUTPUT_MODE"], "none")
            self.assertEqual(environment["SMART_DRONE_OFFBOARD_REQUIRES_VISION"], "0")
            self.assertEqual(environment["SMART_DRONE_VISUAL_LOSS_LAND_MS"], "500")
            self.assertEqual(environment["SMART_DRONE_JSON_DIAGNOSTICS"], "1")
            self.assertTrue(environment["SMART_DRONE_SIM_CONFIG"].endswith("sim/px4_gz/config/smartdrone_sim.yaml"))
            self.assertEqual(environment["SMART_DRONE_SIM_FAULT_FILE"], str((output / "fault_state.json").resolve()))
            self.assertIsNone(environment["SMART_DRONE_SITL_RUN_ID"])
            self.assertIsNone(environment["SMART_DRONE_ATTESTATION_FILE"])
            runner_metrics = json.loads((output / "runner_metrics.json").read_text())
            diagnostic_metrics = json.loads((output / "metrics.json").read_text())
            self.assertEqual(runner_metrics["mode"], "attached")
            self.assertEqual(diagnostic_metrics["provenance"]["status"], "unverified")
            self.assertFalse(diagnostic_metrics["acceptance"]["passed"])

    def test_move_retries_transient_localization_gate(self) -> None:
        port = self._free_port()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            fake_command = (
                f"{sys.executable} tests/sitl/fake_smartdrone_server.py --port {port} "
                "--reject-move-count 2"
            )
            command = [
                sys.executable, "scripts/run_hover_sitl.py", "--profile", "vision",
                "--scenario", "nominal", "--flight-clock", "steady",
                "--command-port", str(port),
                "--smart-drone-command", fake_command, "--output-dir", str(output),
                "--launcher-warmup-s", "0.05", "--hover-duration-s", "0.1",
                "--tracking-stable-s", "0.05", "--ascent-timeout-s", "2",
                "--state-timeout-s", "2", "--command-timeout-s", "2",
                "--land-timeout-s", "2",
            ]
            result = subprocess.run(
                command, cwd=REPO_ROOT, capture_output=True, text=True, timeout=10.0,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            events = [
                json.loads(line)
                for line in (output / "run.jsonl").read_text().splitlines()
            ]
            recovered = [event for event in events if event["event"] == "move_gate_recovered"]
            self.assertEqual(recovered[0]["transient_rejections"], 2)

    def test_arm_retries_until_px4_reports_armed(self) -> None:
        port = self._free_port()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            fake_command = (
                f"{sys.executable} tests/sitl/fake_smartdrone_server.py --port {port} "
                "--arm-after-count 3"
            )
            command = [
                sys.executable, "scripts/run_hover_sitl.py", "--profile", "control",
                "--scenario", "nominal", "--flight-clock", "steady",
                "--command-port", str(port),
                "--smart-drone-command", fake_command, "--output-dir", str(output),
                "--launcher-warmup-s", "0.05", "--hover-duration-s", "0.1",
                "--control-ascent-s", "0", "--state-timeout-s", "2",
                "--command-timeout-s", "4", "--land-timeout-s", "2",
            ]
            result = subprocess.run(
                command, cwd=REPO_ROOT, capture_output=True, text=True, timeout=10.0,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            events = [
                json.loads(line)
                for line in (output / "run.jsonl").read_text().splitlines()
            ]
            recovered = [event for event in events if event["event"] == "arm_gate_recovered"]
            self.assertEqual(recovered[0]["attempts"], 3)

    def test_loss_waits_for_autonomous_land(self) -> None:
        port = self._free_port()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            environment_output = Path(directory) / "environment.json"
            fake_command = (
                f"{sys.executable} tests/sitl/fake_smartdrone_server.py --port {port} "
                "--tracking-loss-after-move-s 1.6 --auto-land-after-move-s 2.2 "
                f"--disarm-after-move-s 2.3 --environment-output {environment_output}"
            )
            command = [
                sys.executable, "scripts/run_hover_sitl.py", "--profile", "vision",
                "--scenario", "loss", "--flight-clock", "steady",
                "--command-port", str(port),
                "--smart-drone-command", fake_command, "--output-dir", str(output),
                "--launcher-warmup-s", "0.05", "--hover-duration-s", "3",
                "--short-blackout-at-s", "0.05", "--long-blackout-at-s", "1.0",
                "--tracking-stable-s", "0.05", "--ascent-timeout-s", "2",
                "--state-timeout-s", "2", "--command-timeout-s", "2",
                "--land-timeout-s", "2",
            ]
            result = subprocess.run(command, cwd=REPO_ROOT, capture_output=True, text=True, timeout=10.0)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            events = self._events(output)
            self.assertIn("blackout_short", events)
            self.assertIn("blackout_short_recovered", events)
            self.assertIn("blackout_long", events)
            self.assertIn("tracking_loss_threshold_exceeded", events)
            self.assertIn("autonomous_land_observed", events)
            self.assertIn("autonomous_land_disarm_confirmed", events)
            self.assertNotIn("command_land", events)
            self.assertNotIn("loss_cleanup_land_sent", events)
            ordered = [
                "blackout_long", "tracking_loss_threshold_exceeded",
                "autonomous_land_observed", "disarmed_confirmed",
            ]
            self.assertEqual(
                [events.index(name) for name in ordered],
                sorted(events.index(name) for name in ordered),
            )
            metrics = json.loads((output / "runner_metrics.json").read_text())
            self.assertTrue(metrics["visual_loss_safety"]["short_blackout_recovered"])
            self.assertTrue(metrics["visual_loss_safety"]["long_blackout_applied"])
            self.assertTrue(metrics["visual_loss_safety"]["autonomous_land_observed"])
            self.assertTrue(metrics["visual_loss_safety"]["autonomous_land_after_long_blackout"])
            self.assertTrue(metrics["visual_loss_safety"]["disarmed_after_autonomous_land"])
            self.assertFalse(metrics["visual_loss_safety"]["cleanup_land_sent"])
            environment = json.loads(environment_output.read_text())
            self.assertEqual(environment["SMART_DRONE_SIM_PROFILE"], "vision")
            self.assertEqual(environment["SMART_DRONE_AUTO_MODE"], "slam")
            self.assertEqual(environment["SMART_DRONE_PX4_POSE_OUTPUT_MODE"], "position_velocity")
            self.assertEqual(environment["SMART_DRONE_OFFBOARD_REQUIRES_VISION"], "1")

    def test_early_land_after_short_blackout_is_not_a_safety_pass(self) -> None:
        early_land = HoverOutcome(
            short_blackout_applied=True,
            autonomous_land_observed=True,
        )
        self.assertFalse(_loss_safety_passed(early_land))
        expected_land = HoverOutcome(
            short_blackout_applied=True,
            short_blackout_recovered=True,
            long_blackout_applied=True,
            tracking_loss_threshold_exceeded=True,
            autonomous_land_observed=True,
            autonomous_land_after_long_blackout=True,
            disarmed_after_autonomous_land=True,
        )
        self.assertTrue(_loss_safety_passed(expected_land))

    def test_disarm_is_not_autonomous_land_mode(self) -> None:
        state = types.SimpleNamespace(
            armed=False,
            px4_main_mode=6,
            px4_sub_mode=0,
        )

        self.assertFalse(_autonomous_land_state(state))

    def test_loss_rejects_offboard_disarm_without_cleanup(self) -> None:
        port = self._free_port()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            command = self._loss_command(
                port,
                output,
                "--tracking-loss-after-move-s 1.0 --disarm-after-move-s 2.1",
            )

            result = subprocess.run(
                command, cwd=REPO_ROOT, capture_output=True, text=True, timeout=10.0,
            )

            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
            events = self._events(output)
            self.assertIn("tracking_loss_threshold_exceeded", events)
            self.assertIn("unexpected_disarm_observed", events)
            self.assertNotIn("autonomous_land_observed", events)
            self.assertNotIn("loss_cleanup_land_sent", events)
            safety = json.loads((output / "runner_metrics.json").read_text())["visual_loss_safety"]
            self.assertTrue(safety["unexpected_disarm_observed"])
            self.assertEqual(safety["cleanup_land_attempt_count"], 0)

    def test_loss_rejects_autonomous_land_before_tracking_threshold(self) -> None:
        port = self._free_port()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            command = self._loss_command(
                port,
                output,
                "--tracking-loss-after-move-s 1.0 --auto-land-after-move-s 1.2 "
                "--disarm-after-move-s 1.4",
            )

            result = subprocess.run(
                command, cwd=REPO_ROOT, capture_output=True, text=True, timeout=10.0,
            )

            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
            events = self._events(output)
            self.assertIn("premature_autonomous_land_observed", events)
            self.assertNotIn("tracking_loss_threshold_exceeded", events)
            self.assertNotIn("autonomous_land_observed", events)
            self.assertNotIn("loss_cleanup_land_sent", events)

    def test_loss_fails_closed_when_px4_flight_state_becomes_invalid(self) -> None:
        port = self._free_port()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            command = self._loss_command(
                port,
                output,
                "--tracking-loss-after-move-s 1.0 "
                "--flight-state-invalid-after-move-s 1.1",
            )

            result = subprocess.run(
                command, cwd=REPO_ROOT, capture_output=True, text=True, timeout=10.0,
            )

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            events = self._events(output)
            self.assertIn("flight_state_telemetry_lost", events)
            self.assertIn("loss_cleanup_land_sent", events)
            self.assertNotIn("autonomous_land_observed", events)
            safety = json.loads((output / "runner_metrics.json").read_text())["visual_loss_safety"]
            self.assertTrue(safety["flight_state_telemetry_lost"])
            self.assertEqual(safety["cleanup_land_attempt_count"], 1)
            self.assertTrue(safety["cleanup_land_acknowledged"])

    def test_loss_fails_closed_when_flight_state_is_lost_after_auto_land(self) -> None:
        port = self._free_port()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            command = self._loss_command(
                port,
                output,
                "--tracking-loss-after-move-s 1.0 --auto-land-after-move-s 2.05 "
                "--flight-state-invalid-after-move-s 2.15 --disarm-after-move-s 2.4",
                hover_duration_s=2.5,
            )

            result = subprocess.run(
                command, cwd=REPO_ROOT, capture_output=True, text=True, timeout=10.0,
            )

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            events = self._events(output)
            self.assertIn("tracking_loss_threshold_exceeded", events)
            self.assertIn("autonomous_land_observed", events)
            self.assertIn("flight_state_telemetry_lost", events)
            self.assertIn("cleanup_land_sent", events)
            safety = json.loads((output / "runner_metrics.json").read_text())["visual_loss_safety"]
            self.assertFalse(safety["disarmed_after_autonomous_land"])
            self.assertTrue(safety["flight_state_telemetry_lost"])
            self.assertEqual(safety["cleanup_land_attempt_count"], 1)

    def test_loss_fails_closed_when_state_stream_stops(self) -> None:
        port = self._free_port()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            command = self._loss_command(
                port,
                output,
                "--tracking-loss-after-move-s 1.0 --stop-state-after-move-s 1.1",
            )

            result = subprocess.run(
                command, cwd=REPO_ROOT, capture_output=True, text=True, timeout=10.0,
            )

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            events = self._events(output)
            self.assertIn("flight_state_telemetry_lost", events)
            self.assertIn("loss_cleanup_land_sent", events)
            safety = json.loads((output / "runner_metrics.json").read_text())["visual_loss_safety"]
            self.assertTrue(safety["flight_state_telemetry_lost"])
            self.assertEqual(safety["cleanup_land_attempt_count"], 1)
            self.assertTrue(safety["cleanup_land_acknowledged"])

    def test_loss_records_rejected_cleanup_attempts(self) -> None:
        port = self._free_port()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            command = self._loss_command(
                port,
                output,
                "--tracking-loss-after-move-s 1.0 --reject-land",
                hover_duration_s=2.0,
            )

            result = subprocess.run(
                command, cwd=REPO_ROOT, capture_output=True, text=True, timeout=10.0,
            )

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            events = self._events(output)
            self.assertEqual(events.count("cleanup_land_failed"), 2)
            safety = json.loads((output / "runner_metrics.json").read_text())["visual_loss_safety"]
            self.assertEqual(safety["cleanup_land_attempt_count"], 2)
            self.assertFalse(safety["cleanup_land_acknowledged"])

    def test_loss_cleanup_land_only_after_observation_window(self) -> None:
        port = self._free_port()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            fake_command = (
                f"{sys.executable} tests/sitl/fake_smartdrone_server.py --port {port} "
                "--tracking-loss-after-move-s 1.6"
            )
            command = [
                sys.executable, "scripts/run_hover_sitl.py", "--profile", "vision",
                "--scenario", "loss", "--flight-clock", "steady",
                "--command-port", str(port),
                "--smart-drone-command", fake_command, "--output-dir", str(output),
                "--launcher-warmup-s", "0.05", "--hover-duration-s", "2.0",
                "--short-blackout-at-s", "0.05", "--long-blackout-at-s", "1.0",
                "--tracking-stable-s", "0.05", "--ascent-timeout-s", "2",
                "--state-timeout-s", "2", "--command-timeout-s", "2",
                "--land-timeout-s", "2",
            ]
            result = subprocess.run(command, cwd=REPO_ROOT, capture_output=True, text=True, timeout=10.0)
            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
            events = self._events(output)
            self.assertIn("tracking_loss_threshold_exceeded", events)
            self.assertIn("loss_failsafe_not_observed", events)
            self.assertIn("loss_cleanup_land_sent", events)
            self.assertNotIn("runner_tracking_loss_land", events)
            metrics = json.loads((output / "runner_metrics.json").read_text())
            self.assertFalse(metrics["visual_loss_safety"]["autonomous_land_observed"])
            self.assertTrue(metrics["visual_loss_safety"]["cleanup_land_sent"])
            self.assertEqual(metrics["visual_loss_safety"]["cleanup_land_attempt_count"], 1)
            self.assertTrue(metrics["visual_loss_safety"]["cleanup_land_acknowledged"])


if __name__ == "__main__":
    unittest.main()
