"""ULog-backed hover accuracy and estimator health metrics."""

from __future__ import annotations

import bisect
import contextlib
import csv
import io
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence


GROUND_TRUTH_TOPICS = (
    "vehicle_local_position_groundtruth",
    "vehicle_odometry_groundtruth",
    "vehicle_ground_truth",
)
ESTIMATE_TOPICS = ("vehicle_local_position", "vehicle_odometry")
VISUAL_TOPICS = ("vehicle_visual_odometry",)
GROUND_TRUTH_ATTITUDE_TOPICS = ("vehicle_attitude_groundtruth",)
OFFBOARD_NAV_STATE = 14
MAV_CMD_NAV_LAND = 21
POSE_FRAME_UNKNOWN = 0
POSE_FRAME_NED = 1
POSE_FRAME_FRD = 2
SUPPORTED_PROFILES = frozenset(("control", "truth", "vision"))
HOVER_ENTRY_VERTICAL_TOLERANCE_M = 0.15
SETTLING_ENTRY_VERTICAL_TOLERANCE_M = 0.30
ALTITUDE_GROUP_MAX_GAP_S = 0.5
SMART_DRONE_TIMING_FIELDS = {
    "eye_skew_ms": ("eye_skew_ms",),
    "render_transport_ms": ("render_transport_ms",),
    "queue_ms": ("queue_ms",),
    "processing_ms": ("processing_ms",),
    "send_ms": ("send_ms",),
    "pair_to_tx_ms": ("pair_to_tx_ms",),
    "wall_total_ms": ("wall_total_ms", "total_ms"),
    "sim_age_ms": ("sim_age_ms",),
}


@dataclass(frozen=True)
class PositionSample:
    timestamp_s: float
    x: float
    y: float
    z: float
    quality: float | None = None
    pose_frame: int | None = None


@dataclass(frozen=True)
class AttitudeSample:
    timestamp_s: float
    w: float
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class AnalysisOptions:
    profile: str = "vision"
    scenario: str = "nominal"
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = -1.5
    hover_start_s: float | None = None
    hover_end_s: float | None = None
    disturbance_time_s: float | None = None
    horizontal_rmse_limit_m: float = 0.20
    horizontal_max_limit_m: float = 0.50
    vertical_rmse_limit_m: float = 0.15
    vertical_max_limit_m: float = 0.30
    odometry_rate_limit_hz: float = 18.0
    latency_p95_limit_ms: float = 80.0
    visual_availability_limit: float = 0.99
    recovery_limit_s: float = 3.0
    minimum_hover_duration_s: float = 30.0


def percentile(values: Sequence[float], percent: float) -> float | None:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return None
    index = (len(finite) - 1) * percent / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return finite[lower]
    fraction = index - lower
    return finite[lower] * (1.0 - fraction) + finite[upper] * fraction


def _finite_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _first_nonnegative_float(record: dict[str, Any], names: Sequence[str]) -> float | None:
    for name in names:
        value = _finite_float(record.get(name))
        if value is not None and value >= 0.0:
            return value
    return None


def _smart_drone_timing_sample(
    record: Any, start_s: float | None, end_s: float | None,
) -> dict[str, float] | None:
    if not isinstance(record, dict) or record.get("tag") != "odom_ts":
        return None
    if record.get("timing") != 1:
        return None
    wall_total_ms = _first_nonnegative_float(
        record, SMART_DRONE_TIMING_FIELDS["wall_total_ms"],
    )
    if wall_total_ms is None:
        return None
    if start_s is not None or end_s is not None:
        camera_ns = _finite_float(record.get("cam_ns"))
        if camera_ns is None or camera_ns < 0.0:
            return None
        measurement_s = camera_ns / 1_000_000_000.0
        if start_s is not None and measurement_s < start_s:
            return None
        if end_s is not None and measurement_s > end_s:
            return None
    sample = {"wall_total_ms": wall_total_ms}
    for output_name, input_names in SMART_DRONE_TIMING_FIELDS.items():
        value = _first_nonnegative_float(record, input_names)
        if value is not None:
            sample[output_name] = value
    return sample


def _smart_drone_latency_sample(
    record: Any, start_s: float | None, end_s: float | None,
) -> float | None:
    sample = _smart_drone_timing_sample(record, start_s, end_s)
    return sample["wall_total_ms"] if sample is not None else None


def smart_drone_latency_summary(
    jsonl_path: Path, start_s: float | None = None, end_s: float | None = None,
) -> dict[str, float | int | None]:
    """Summarize measured camera-to-MAVLink latency from SmartDrone JSONL."""
    if start_s is not None and end_s is not None and end_s < start_s:
        raise ValueError("latency window end must not precede its start")
    samples = {name: [] for name in SMART_DRONE_TIMING_FIELDS}
    with Path(jsonl_path).open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            try:
                record = json.loads(line)
            except (json.JSONDecodeError, TypeError):
                continue
            timing = _smart_drone_timing_sample(record, start_s, end_s)
            if timing is None:
                continue
            for name, value in timing.items():
                samples[name].append(value)
    return {
        "sample_count": len(samples["wall_total_ms"]),
        "latency_p95_ms": percentile(samples["wall_total_ms"], 95.0),
        "timing_p95_ms": {
            name: percentile(values, 95.0) for name, values in samples.items()
        },
        "timing_sample_count": {
            name: len(values) for name, values in samples.items()
        },
    }


def error_summary(values: Sequence[float]) -> dict[str, float | int | None]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return {"count": 0, "rmse": None, "p95": None, "max": None}
    return {
        "count": len(finite),
        "rmse": math.sqrt(sum(value * value for value in finite) / len(finite)),
        "p95": percentile(finite, 95.0),
        "max": max(finite),
    }


def _interpolate(samples: Sequence[PositionSample], timestamp_s: float, max_gap_s: float) -> PositionSample | None:
    if not samples:
        return None
    timestamps = [sample.timestamp_s for sample in samples]
    index = bisect.bisect_left(timestamps, timestamp_s)
    if index < len(samples) and samples[index].timestamp_s == timestamp_s:
        return samples[index]
    if index == 0 or index == len(samples):
        return None
    before, after = samples[index - 1], samples[index]
    if timestamp_s - before.timestamp_s > max_gap_s or after.timestamp_s - timestamp_s > max_gap_s:
        return None
    span = after.timestamp_s - before.timestamp_s
    if span <= 0.0:
        return before
    ratio = (timestamp_s - before.timestamp_s) / span
    return PositionSample(
        timestamp_s,
        before.x + ratio * (after.x - before.x),
        before.y + ratio * (after.y - before.y),
        before.z + ratio * (after.z - before.z),
        before.quality if ratio < 0.5 else after.quality,
        before.pose_frame if before.pose_frame == after.pose_frame else None,
    )


def aligned_errors(
    truth: Sequence[PositionSample],
    estimate: Sequence[PositionSample],
    start_s: float,
    end_s: float,
    max_gap_s: float = 0.10,
) -> dict[str, dict[str, float | int | None]]:
    horizontal: list[float] = []
    vertical: list[float] = []
    spatial: list[float] = []
    for sample in estimate:
        if not start_s <= sample.timestamp_s <= end_s:
            continue
        reference = _interpolate(truth, sample.timestamp_s, max_gap_s)
        if reference is None:
            continue
        dx, dy, dz = sample.x - reference.x, sample.y - reference.y, sample.z - reference.z
        horizontal.append(math.hypot(dx, dy))
        vertical.append(abs(dz))
        spatial.append(math.sqrt(dx * dx + dy * dy + dz * dz))
    return {
        "horizontal_m": error_summary(horizontal),
        "vertical_m": error_summary(vertical),
        "spatial_m": error_summary(spatial),
    }


def hover_errors(
    truth: Sequence[PositionSample],
    options: AnalysisOptions,
    start_s: float,
    end_s: float,
) -> dict[str, dict[str, float | int | None]]:
    horizontal: list[float] = []
    vertical: list[float] = []
    for sample in truth:
        if not start_s <= sample.timestamp_s <= end_s:
            continue
        horizontal.append(math.hypot(sample.x - options.target_x, sample.y - options.target_y))
        vertical.append(abs(sample.z - options.target_z))
    return {"horizontal_m": error_summary(horizontal), "vertical_m": error_summary(vertical)}


def detect_hover_window(
    truth: Sequence[PositionSample], options: AnalysisOptions
) -> tuple[float, float, str]:
    if not truth:
        raise ValueError("ground-truth topic has no finite position samples")
    if (options.hover_start_s is None) != (options.hover_end_s is None):
        raise ValueError("hover start and end must be supplied together")
    if options.hover_start_s is not None and options.hover_end_s is not None:
        if options.hover_end_s <= options.hover_start_s:
            raise ValueError("hover end must be greater than hover start")
        return options.hover_start_s, options.hover_end_s, "explicit"
    groups = _altitude_groups(
        truth, options.target_z, HOVER_ENTRY_VERTICAL_TOLERANCE_M,
    )
    if not groups:
        raise ValueError("could not detect a hover interval near the configured target altitude")
    longest = max(groups, key=lambda group: group[-1].timestamp_s - group[0].timestamp_s)
    return longest[0].timestamp_s, longest[-1].timestamp_s, "target_altitude"


def _altitude_groups(
    truth: Sequence[PositionSample],
    target_z: float,
    tolerance_m: float,
    end_s: float | None = None,
) -> list[list[PositionSample]]:
    candidates = [
        sample for sample in truth
        if (end_s is None or sample.timestamp_s <= end_s)
        and abs(sample.z - target_z) <= tolerance_m
    ]
    if len(candidates) < 2:
        return []
    groups: list[list[PositionSample]] = [[candidates[0]]]
    for sample in candidates[1:]:
        if sample.timestamp_s - groups[-1][-1].timestamp_s > ALTITUDE_GROUP_MAX_GAP_S:
            groups.append([])
        groups[-1].append(sample)
    return [group for group in groups if len(group) >= 2]


def detect_settling_window(
    truth: Sequence[PositionSample],
    options: AnalysisOptions,
    hover_start_s: float,
) -> tuple[float, float, str] | None:
    groups = _altitude_groups(
        truth, options.target_z, SETTLING_ENTRY_VERTICAL_TOLERANCE_M,
        hover_start_s,
    )
    if not groups:
        return None
    latest = max(groups, key=lambda group: group[-1].timestamp_s)
    if hover_start_s - latest[-1].timestamp_s > ALTITUDE_GROUP_MAX_GAP_S:
        return None
    if latest[0].timestamp_s >= hover_start_s:
        return None
    return latest[0].timestamp_s, hover_start_s, "target_approach"


def _settling_window_metrics(
    window: tuple[float, float, str] | None,
    hover_start_s: float,
) -> dict[str, float | str | None]:
    if window is None:
        return {
            "start_s": None, "end_s": hover_start_s,
            "duration_s": 0.0, "source": "unavailable",
        }
    start_s, end_s, source = window
    return {
        "start_s": start_s, "end_s": end_s,
        "duration_s": end_s - start_s, "source": source,
    }


def _settling_accuracy(
    truth: Sequence[PositionSample],
    options: AnalysisOptions,
    window: tuple[float, float, str] | None,
) -> dict[str, dict[str, float | int | None]]:
    if window is None:
        empty = error_summary([])
        return {"horizontal_m": dict(empty), "vertical_m": dict(empty)}
    return hover_errors(truth, options, window[0], window[1])


def calculate_recovery_time(
    truth: Sequence[PositionSample],
    options: AnalysisOptions,
    event_time_s: float,
    end_s: float,
    stable_s: float = 0.5,
) -> float | None:
    after = [sample for sample in truth if event_time_s <= sample.timestamp_s <= end_s]
    if not after:
        return None
    outside_seen = False
    stable_since: float | None = None
    for sample in after:
        horizontal = math.hypot(sample.x - options.target_x, sample.y - options.target_y)
        vertical = abs(sample.z - options.target_z)
        inside = horizontal <= 0.25 and vertical <= 0.15
        outside_seen = outside_seen or not inside
        if not outside_seen:
            continue
        if not inside:
            stable_since = None
            continue
        stable_since = sample.timestamp_s if stable_since is None else stable_since
        if sample.timestamp_s - stable_since >= stable_s:
            return stable_since - event_time_s
    return 0.0 if not outside_seen else None


def _topic(ulog: Any, candidates: Iterable[str]) -> Any | None:
    by_name: dict[str, list[Any]] = {}
    for dataset in ulog.data_list:
        by_name.setdefault(dataset.name, []).append(dataset)
    for name in candidates:
        datasets = by_name.get(name, [])
        if datasets:
            return min(datasets, key=lambda item: getattr(item, "multi_id", 0))
    return None


def _field(data: dict[str, Any], names: Iterable[str]) -> Any | None:
    for name in names:
        if name in data:
            return data[name]
    return None


def _normalized_quaternion(values: Sequence[float]) -> tuple[float, float, float, float] | None:
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm <= 1e-9:
        return None
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def _extract_attitude_samples(dataset: Any) -> list[AttitudeSample]:
    data = dataset.data
    timestamp = _field(data, ("timestamp_sample", "timestamp"))
    components = [
        _field(data, (f"q[{index}]", f"q.{index}"))
        for index in range(4)
    ]
    if timestamp is None or any(component is None for component in components):
        return []
    samples: list[AttitudeSample] = []
    for index, value in enumerate(timestamp):
        quaternion = _normalized_quaternion([float(component[index]) for component in components])
        if quaternion is not None:
            samples.append(AttitudeSample(float(value) / 1e6, *quaternion))
    return sorted(samples, key=lambda sample: sample.timestamp_s)


def _extract_heading_attitudes(dataset: Any) -> list[AttitudeSample]:
    timestamp = _field(dataset.data, ("timestamp_sample", "timestamp"))
    heading = _field(dataset.data, ("heading",))
    if timestamp is None or heading is None:
        return []
    samples: list[AttitudeSample] = []
    for value, yaw in zip(timestamp, heading):
        yaw_value = float(yaw)
        if math.isfinite(yaw_value):
            samples.append(AttitudeSample(
                float(value) / 1e6, math.cos(yaw_value / 2.0), 0.0, 0.0,
                math.sin(yaw_value / 2.0),
            ))
    return sorted(samples, key=lambda sample: sample.timestamp_s)


def _interpolate_attitude(
    samples: Sequence[AttitudeSample], timestamp_s: float, max_gap_s: float
) -> AttitudeSample | None:
    if not samples:
        return None
    timestamps = [sample.timestamp_s for sample in samples]
    index = bisect.bisect_left(timestamps, timestamp_s)
    if index < len(samples) and samples[index].timestamp_s == timestamp_s:
        return samples[index]
    if index == 0 or index == len(samples):
        return None
    before, after = samples[index - 1], samples[index]
    if timestamp_s - before.timestamp_s > max_gap_s or after.timestamp_s - timestamp_s > max_gap_s:
        return None
    span = after.timestamp_s - before.timestamp_s
    ratio = 0.0 if span <= 0.0 else (timestamp_s - before.timestamp_s) / span
    first = (before.w, before.x, before.y, before.z)
    second = (after.w, after.x, after.y, after.z)
    if sum(left * right for left, right in zip(first, second)) < 0.0:
        second = tuple(-value for value in second)
    interpolated = _normalized_quaternion([
        left + ratio * (right - left) for left, right in zip(first, second)
    ])
    return AttitudeSample(timestamp_s, *interpolated) if interpolated is not None else None


def _rotate_by_quaternion(
    attitude: AttitudeSample, vector: tuple[float, float, float]
) -> tuple[float, float, float]:
    w, x, y, z = attitude.w, attitude.x, attitude.y, attitude.z
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def _land_command_time(ulog: Any, start_s: float, end_s: float) -> float | None:
    commands = _topic(ulog, ("vehicle_command",))
    if commands is None:
        return None
    timestamp = commands.data.get("timestamp", [])
    command = commands.data.get("command", [])
    candidates = [
        float(value) / 1e6
        for index, value in enumerate(timestamp)
        if index < len(command)
        and int(command[index]) == MAV_CMD_NAV_LAND
        and start_s < float(value) / 1e6 <= end_s
    ]
    return min(candidates) if candidates else None


def _extract_position_samples(dataset: Any) -> list[PositionSample]:
    data = dataset.data
    timestamp = _field(data, ("timestamp_sample", "timestamp"))
    x = _field(data, ("x", "position[0]", "position.0"))
    y = _field(data, ("y", "position[1]", "position.1"))
    z = _field(data, ("z", "position[2]", "position.2"))
    quality = _field(data, ("quality",))
    pose_frame = _field(data, ("pose_frame",))
    if timestamp is None or x is None or y is None or z is None:
        raise ValueError(f"ULog topic {dataset.name} has no recognized position fields")
    samples: list[PositionSample] = []
    for index in range(len(timestamp)):
        values = (float(x[index]), float(y[index]), float(z[index]))
        if not all(math.isfinite(value) for value in values):
            continue
        quality_value = float(quality[index]) if quality is not None else None
        frame_value = int(pose_frame[index]) if pose_frame is not None else None
        samples.append(PositionSample(
            float(timestamp[index]) / 1_000_000.0, *values, quality_value, frame_value,
        ))
    return sorted(samples, key=lambda sample: sample.timestamp_s)


def _pose_frame_name(pose_frame: int | None) -> str:
    names = {POSE_FRAME_UNKNOWN: "UNKNOWN", POSE_FRAME_NED: "NED", POSE_FRAME_FRD: "FRD"}
    return names.get(pose_frame, "MISSING" if pose_frame is None else f"UNSUPPORTED_{pose_frame}")


def _visual_alignment_result(
    status: str, source_frame: int | None, reason: str | None = None,
    anchor_timestamp_s: float | None = None,
) -> dict[str, Any]:
    return {
        "status": status,
        "source_pose_frame": source_frame,
        "source_pose_frame_name": _pose_frame_name(source_frame),
        "output_pose_frame": POSE_FRAME_NED if status == "aligned" else None,
        "output_pose_frame_name": "NED" if status == "aligned" else None,
        "anchor_timestamp_s": anchor_timestamp_s,
        "reason": reason,
    }


def _align_visual_positions(
    truth: Sequence[PositionSample],
    visual: Sequence[PositionSample],
    truth_attitude: Sequence[AttitudeSample],
    max_gap_s: float = 0.10,
) -> tuple[list[PositionSample], dict[str, Any]]:
    if not visual:
        return [], _visual_alignment_result("unavailable", None, "visual odometry has no finite samples")
    frames = {sample.pose_frame for sample in visual}
    if len(frames) != 1:
        return [], _visual_alignment_result("unavailable", None, "visual odometry contains mixed pose frames")
    source_frame = next(iter(frames))
    if source_frame not in (POSE_FRAME_NED, POSE_FRAME_FRD):
        return [], _visual_alignment_result(
            "unavailable", source_frame, "visual odometry pose frame is missing or unsupported",
        )
    for index, anchor in enumerate(visual):
        truth_anchor = _interpolate(truth, anchor.timestamp_s, max_gap_s)
        attitude = (
            _interpolate_attitude(truth_attitude, anchor.timestamp_s, max_gap_s)
            if source_frame == POSE_FRAME_FRD else None
        )
        if truth_anchor is None or (source_frame == POSE_FRAME_FRD and attitude is None):
            continue
        aligned = _transform_visual_positions(visual[index:], anchor, truth_anchor, attitude)
        return aligned, _visual_alignment_result(
            "aligned", source_frame, anchor_timestamp_s=anchor.timestamp_s,
        )
    reason = "no same-time ground-truth position"
    if source_frame == POSE_FRAME_FRD:
        reason += " and attitude"
    return [], _visual_alignment_result("unavailable", source_frame, reason)


def _transform_visual_positions(
    visual: Sequence[PositionSample],
    visual_anchor: PositionSample,
    truth_anchor: PositionSample,
    attitude: AttitudeSample | None,
) -> list[PositionSample]:
    aligned: list[PositionSample] = []
    for sample in visual:
        displacement = (
            sample.x - visual_anchor.x,
            sample.y - visual_anchor.y,
            sample.z - visual_anchor.z,
        )
        if attitude is not None:
            displacement = _rotate_by_quaternion(attitude, displacement)
        aligned.append(PositionSample(
            sample.timestamp_s,
            truth_anchor.x + displacement[0],
            truth_anchor.y + displacement[1],
            truth_anchor.z + displacement[2],
            sample.quality,
            POSE_FRAME_NED,
        ))
    return aligned


def _reset_count(dataset: Any, start_s: float, end_s: float) -> int | None:
    reset_values = _field(dataset.data, ("reset_counter", "reset_counter_pos"))
    measurement_times = dataset.data.get("timestamp_sample", dataset.data.get("timestamp", []))
    if reset_values is None:
        return None
    previous: int | None = None
    resets = 0
    for timestamp, value in zip(measurement_times, reset_values):
        time_s = float(timestamp) / 1e6
        current = int(value)
        if time_s > end_s:
            break
        if time_s >= start_s and previous is not None and current != previous:
            resets += 1
        previous = current
    return resets


def _empty_visual_health() -> dict[str, Any]:
    return {
        "topic": None, "sample_count": 0, "usable_count": None,
        "frequency_hz": None, "usable_frequency_hz": None,
        "availability": None, "latency_p95_ms": None, "reset_count": None,
        "pose_frames": [], "pose_frame_names": [],
    }


def _visual_health(dataset: Any, samples: Sequence[PositionSample], start_s: float, end_s: float) -> dict[str, Any]:
    selected = [sample for sample in samples if start_s <= sample.timestamp_s <= end_s]
    pose_frames = sorted({sample.pose_frame for sample in selected if sample.pose_frame is not None})
    duration_s = max(0.0, end_s - start_s)
    quality_available = any(sample.quality is not None for sample in selected)
    usable = [sample for sample in selected if sample.quality is not None and sample.quality > 0.0]
    timestamp = dataset.data.get("timestamp")
    timestamp_sample = dataset.data.get("timestamp_sample")
    latencies: list[float] = []
    if timestamp is not None and timestamp_sample is not None:
        for published, measured in zip(timestamp, timestamp_sample):
            measured_s = float(measured) / 1_000_000.0
            if start_s <= measured_s <= end_s and published >= measured:
                latencies.append((float(published) - float(measured)) / 1000.0)
    return {
        "topic": dataset.name,
        "sample_count": len(selected),
        "usable_count": len(usable) if quality_available else None,
        "frequency_hz": len(selected) / duration_s if duration_s > 0.0 else None,
        "usable_frequency_hz": len(usable) / duration_s if duration_s > 0.0 and quality_available else None,
        "availability": len(usable) / len(selected) if selected and quality_available else None,
        "latency_p95_ms": percentile(latencies, 95.0),
        "reset_count": _reset_count(dataset, start_s, end_s),
        "pose_frames": pose_frames,
        "pose_frame_names": [_pose_frame_name(frame) for frame in pose_frames],
    }


def _estimator_diagnostics(ulog: Any, start_s: float, end_s: float) -> dict[str, Any]:
    output: dict[str, Any] = {}
    flags = _topic(ulog, ("estimator_status_flags",))
    if flags is not None:
        timestamp = flags.data.get("timestamp", [])
        selected = [index for index, value in enumerate(timestamp) if start_s <= float(value) / 1e6 <= end_s]
        fractions: dict[str, float] = {}
        for name, values in flags.data.items():
            if name.startswith("timestamp") or not selected:
                continue
            fractions[name] = sum(bool(values[index]) for index in selected) / len(selected)
        output["flags_true_fraction"] = fractions
    status = _topic(ulog, ("estimator_status",))
    if status is not None:
        timestamp = status.data.get("timestamp", [])
        ratios: dict[str, dict[str, float | int | None]] = {}
        for name, values in status.data.items():
            if not name.endswith("_ratio"):
                continue
            selected_values = [
                float(values[index])
                for index, value in enumerate(timestamp)
                if start_s <= float(value) / 1e6 <= end_s
            ]
            ratios[name] = error_summary(selected_values)
        output["innovation_ratios"] = ratios
    return output


def _flight_diagnostics(ulog: Any, start_s: float, end_s: float) -> dict[str, Any]:
    status = _topic(ulog, ("vehicle_status",))
    if status is None:
        return {
            "topic": None, "offboard_seen": False,
            "offboard_at_window_start": None, "offboard_exit_count": None,
            "failsafe_at_window_start": None, "failsafe_sample_count": None,
        }
    data = status.data
    timestamp = data.get("timestamp", [])
    nav_state = data.get("nav_state", [])
    armed = data.get("armed")
    arming_state = data.get("arming_state")
    failsafe = data.get("failsafe")
    previous_nav: int | None = None
    exits = 0
    failsafe_samples = 0
    offboard_seen = False
    offboard_at_start: bool | None = None
    failsafe_at_start: bool | None = None
    for index, value in enumerate(timestamp):
        time_s = float(value) / 1e6
        if time_s >= end_s or index >= len(nav_state):
            continue
        nav = int(nav_state[index])
        is_armed = bool(armed[index]) if armed is not None else (
            int(arming_state[index]) == 2 if arming_state is not None else True
        )
        current_failsafe = bool(failsafe[index]) if failsafe is not None else None
        offboard_seen = offboard_seen or (time_s <= end_s and nav == OFFBOARD_NAV_STATE)
        if time_s <= start_s:
            offboard_at_start = nav == OFFBOARD_NAV_STATE
            failsafe_at_start = current_failsafe
        if time_s >= start_s and previous_nav == OFFBOARD_NAV_STATE and nav != OFFBOARD_NAV_STATE and is_armed:
            exits += 1
        if time_s >= start_s and current_failsafe is True:
            failsafe_samples += 1
        previous_nav = nav
    return {
        "topic": status.name,
        "offboard_seen": offboard_seen,
        "offboard_at_window_start": offboard_at_start,
        "offboard_exit_count": exits,
        "failsafe_at_window_start": failsafe_at_start,
        "failsafe_sample_count": failsafe_samples,
    }


def _check(name: str, actual: float | int | None, limit: float, relation: str) -> dict[str, Any]:
    passed = actual is not None and (actual <= limit if relation == "max" else actual >= limit)
    return {"name": name, "actual": actual, "relation": relation, "limit": limit, "passed": passed}


def _acceptance(metrics: dict[str, Any], options: AnalysisOptions) -> dict[str, Any]:
    hover = metrics["hover_accuracy"]
    visual = metrics["visual_odometry"]
    flight = metrics["flight"]
    checks = [
        _check("horizontal_rmse_m", hover["horizontal_m"]["rmse"], options.horizontal_rmse_limit_m, "max"),
        _check("horizontal_max_m", hover["horizontal_m"]["max"], options.horizontal_max_limit_m, "max"),
        _check("vertical_rmse_m", hover["vertical_m"]["rmse"], options.vertical_rmse_limit_m, "max"),
        _check("vertical_max_m", hover["vertical_m"]["max"], options.vertical_max_limit_m, "max"),
        _check("offboard_at_window_start", _optional_bool_number(flight.get("offboard_at_window_start")), 1.0, "min"),
    ]
    if options.scenario != "loss":
        checks.append(_check(
            "hover_duration_s", metrics["hover_window"]["duration_s"],
            options.minimum_hover_duration_s, "min",
        ))
    if options.profile != "control":
        checks.extend([
            _check("visual_odometry_frequency_hz", visual["frequency_hz"], options.odometry_rate_limit_hz, "min"),
            _check("visual_reset_count", visual["reset_count"], 0.0, "max"),
            _check("visual_latency_p95_ms", visual["latency_p95_ms"], options.latency_p95_limit_ms, "max"),
        ])
    if options.scenario != "loss":
        checks.extend([
            _check("offboard_exit_count", flight["offboard_exit_count"], 0.0, "max"),
            _check("failsafe_at_window_start", _optional_bool_number(flight.get("failsafe_at_window_start")), 0.0, "max"),
            _check("failsafe_sample_count", flight["failsafe_sample_count"], 0.0, "max"),
        ])
        if options.profile != "control":
            checks.append(_check("visual_availability", visual["availability"], options.visual_availability_limit, "min"))
    if options.scenario == "impulse":
        checks.append(_check("disturbance_recovery_s", metrics["recovery_time_s"], options.recovery_limit_s, "max"))
    complete = all(check["actual"] is not None for check in checks)
    return {"passed": complete and all(check["passed"] for check in checks), "complete": complete, "checks": checks}


def _optional_bool_number(value: Any) -> int | None:
    return int(value) if isinstance(value, bool) else None


def _write_trajectory(
    path: Path,
    truth: Sequence[PositionSample],
    estimate: Sequence[PositionSample],
    visual: Sequence[PositionSample],
) -> None:
    columns = [
        "timestamp_s", "truth_x", "truth_y", "truth_z", "estimate_x", "estimate_y",
        "estimate_z", "visual_x", "visual_y", "visual_z", "visual_quality",
        "estimate_horizontal_error_m", "estimate_vertical_error_m",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for sample in truth:
            estimate_sample = _interpolate(estimate, sample.timestamp_s, 0.10)
            visual_sample = _interpolate(visual, sample.timestamp_s, 0.10)
            row: dict[str, Any] = {
                "timestamp_s": sample.timestamp_s, "truth_x": sample.x,
                "truth_y": sample.y, "truth_z": sample.z,
            }
            if estimate_sample is not None:
                row.update({
                    "estimate_x": estimate_sample.x, "estimate_y": estimate_sample.y,
                    "estimate_z": estimate_sample.z,
                    "estimate_horizontal_error_m": math.hypot(estimate_sample.x - sample.x, estimate_sample.y - sample.y),
                    "estimate_vertical_error_m": abs(estimate_sample.z - sample.z),
                })
            if visual_sample is not None:
                row.update({
                    "visual_x": visual_sample.x, "visual_y": visual_sample.y,
                    "visual_z": visual_sample.z, "visual_quality": visual_sample.quality,
                })
            writer.writerow(row)


def _try_plot(
    output_dir: Path,
    truth: Sequence[PositionSample],
    visual: Sequence[PositionSample],
    options: AnalysisOptions,
) -> dict[str, Any]:
    if not truth:
        return {"generated": False, "reason": "hover window contains no ground-truth samples"}
    stderr = io.StringIO()
    try:
        with contextlib.redirect_stderr(stderr), contextlib.redirect_stdout(io.StringIO()):
            import matplotlib.pyplot as plt  # type: ignore
    except Exception as error:
        return {"generated": False, "reason": f"matplotlib unavailable: {type(error).__name__}"}
    times = [sample.timestamp_s - truth[0].timestamp_s for sample in truth]
    horizontal = [math.hypot(sample.x - options.target_x, sample.y - options.target_y) for sample in truth]
    vertical = [sample.z - options.target_z for sample in truth]
    figure, axes = plt.subplots(3, 1, figsize=(9, 8))
    axes[0].plot(times, horizontal, label="horizontal error")
    axes[0].axhline(options.horizontal_rmse_limit_m, color="tab:red", linestyle="--", label="RMSE limit")
    axes[0].set_ylabel("error (m)")
    axes[0].legend()
    axes[1].plot(times, vertical, label="vertical signed error")
    axes[1].axhline(options.vertical_rmse_limit_m, color="tab:red", linestyle="--")
    axes[1].axhline(-options.vertical_rmse_limit_m, color="tab:red", linestyle="--")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("error (m)")
    axes[1].legend()
    quality_error = []
    for sample, error in zip(truth, horizontal):
        visual_sample = _interpolate(visual, sample.timestamp_s, 0.10)
        if visual_sample is not None and visual_sample.quality is not None:
            quality_error.append((visual_sample.quality, error))
    if quality_error:
        axes[2].scatter(
            [item[0] for item in quality_error],
            [item[1] for item in quality_error],
            s=8,
            alpha=0.5,
        )
        axes[2].set_xlabel("visual quality")
        axes[2].set_ylabel("horizontal error (m)")
    else:
        axes[2].text(0.5, 0.5, "visual quality unavailable", ha="center", va="center")
        axes[2].set_axis_off()
    figure.tight_layout()
    path = output_dir / "hover_metrics.png"
    figure.savefig(path, dpi=140)
    plt.close(figure)
    return {"generated": True, "path": path.name}


def analyze_ulog(ulog_path: Path, output_dir: Path, options: AnalysisOptions) -> dict[str, Any]:
    if options.profile not in SUPPORTED_PROFILES:
        raise ValueError(f"unsupported analysis profile: {options.profile}")
    try:
        from pyulog import ULog  # type: ignore
    except ImportError as error:
        raise RuntimeError("pyulog is required to analyze PX4 ULog files") from error
    ulog = ULog(str(ulog_path))
    truth_topic = _topic(ulog, GROUND_TRUTH_TOPICS)
    truth_attitude_topic = _topic(ulog, GROUND_TRUTH_ATTITUDE_TOPICS)
    estimate_topic = _topic(ulog, ESTIMATE_TOPICS)
    visual_topic = _topic(ulog, VISUAL_TOPICS)
    required = [("ground truth", truth_topic), ("estimate", estimate_topic)]
    if options.profile != "control":
        required.append(("visual odometry", visual_topic))
    missing = [name for name, topic in required if topic is None]
    if missing:
        raise ValueError(f"ULog is missing required topics: {', '.join(missing)}")
    truth = _extract_position_samples(truth_topic)
    estimate = _extract_position_samples(estimate_topic)
    visual = _extract_position_samples(visual_topic) if visual_topic is not None else []
    truth_attitude = (
        _extract_attitude_samples(truth_attitude_topic)
        if truth_attitude_topic is not None else _extract_attitude_samples(truth_topic)
    )
    if not truth_attitude:
        truth_attitude = _extract_heading_attitudes(truth_topic)
    aligned_visual, visual_alignment = _align_visual_positions(truth, visual, truth_attitude)
    start_s, end_s, window_source = detect_hover_window(truth, options)
    settling_window = detect_settling_window(truth, options, start_s)
    land_command_s = _land_command_time(ulog, start_s, end_s)
    if land_command_s is not None:
        end_s = land_command_s
        window_source += "_until_land_command"
    event_s = options.disturbance_time_s if options.disturbance_time_s is not None else start_s + 15.0
    metrics: dict[str, Any] = {
        "schema": "smartdrone.sitl.hover.metrics.v1",
        "ulog": str(ulog_path),
        "profile": options.profile,
        "scenario": options.scenario,
        "topics": {
            "ground_truth": truth_topic.name, "estimate": estimate_topic.name,
            "visual_odometry": visual_topic.name if visual_topic is not None else None,
            "ground_truth_attitude": (
                truth_attitude_topic.name if truth_attitude_topic is not None else None
            ),
        },
        "settling_window": _settling_window_metrics(settling_window, start_s),
        "settling_accuracy": _settling_accuracy(
            truth, options, settling_window,
        ),
        "hover_window": {"start_s": start_s, "end_s": end_s, "duration_s": end_s - start_s, "source": window_source},
        "hover_accuracy": hover_errors(truth, options, start_s, end_s),
        "estimator_accuracy": aligned_errors(truth, estimate, start_s, end_s),
        "visual_accuracy": aligned_errors(truth, aligned_visual, start_s, end_s),
        "visual_alignment": visual_alignment,
        "visual_odometry": (
            _visual_health(visual_topic, visual, start_s, end_s)
            if visual_topic is not None else _empty_visual_health()
        ),
        "estimator": _estimator_diagnostics(ulog, start_s, end_s),
        "flight": _flight_diagnostics(ulog, start_s, end_s),
        "recovery_time_s": calculate_recovery_time(truth, options, event_s, end_s),
    }
    metrics["acceptance"] = _acceptance(metrics, options)
    output_dir.mkdir(parents=True, exist_ok=True)
    _write_trajectory(output_dir / "trajectory.csv", truth, estimate, aligned_visual)
    hover_truth = [sample for sample in truth if start_s <= sample.timestamp_s <= end_s]
    metrics["plot"] = _try_plot(output_dir, hover_truth, aligned_visual, options)
    (output_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return metrics
