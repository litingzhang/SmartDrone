#!/usr/bin/env python3
"""Evaluate offline replay pose CSV against EuRoC ground truth."""

from __future__ import annotations

import argparse
import csv
import json
import math
from bisect import bisect_left
from pathlib import Path
from typing import Tuple

import numpy as np


Vec3 = Tuple[float, float, float]
Mat3 = Tuple[Vec3, Vec3, Vec3]


def _resolve_mav0(dataset: Path) -> Path:
    if (dataset / "mav0").is_dir():
        return dataset / "mav0"
    return dataset


def _read_ground_truth(dataset: Path) -> tuple[list[int], list[Vec3]]:
    gt_csv = _resolve_mav0(dataset) / "state_groundtruth_estimate0" / "data.csv"
    if not gt_csv.exists():
        raise FileNotFoundError(f"EuRoC ground truth CSV not found: {gt_csv}")

    timestamps: list[int] = []
    positions: list[Vec3] = []
    with gt_csv.open(newline="") as fp:
        reader = csv.reader(fp)
        for row in reader:
            if not row or row[0].startswith("#") or len(row) < 4:
                continue
            timestamps.append(int(row[0].strip()))
            positions.append((float(row[1]), float(row[2]), float(row[3])))
    return timestamps, positions


def _read_estimate(path: Path) -> tuple[list[int], list[Vec3]]:
    timestamps: list[int] = []
    positions: list[Vec3] = []
    with path.open(newline="") as fp:
        reader = csv.DictReader(fp)
        required = {"capture_timestamp_ns", "pose_valid", "x", "y", "z"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"estimate CSV missing columns: {sorted(missing)}")
        for row in reader:
            if row["pose_valid"] != "1":
                continue
            timestamps.append(int(row["capture_timestamp_ns"]))
            positions.append((float(row["x"]), float(row["y"]), float(row["z"])))
    return timestamps, positions


def _associate_by_nearest(
    estimate_ts: list[int],
    estimate_pos: list[Vec3],
    gt_ts: list[int],
    gt_pos: list[Vec3],
    max_delta_ns: int,
) -> tuple[list[Vec3], list[Vec3]]:
    matched_est: list[Vec3] = []
    matched_gt: list[Vec3] = []
    used_gt: set[int] = set()
    for ts, pos in zip(estimate_ts, estimate_pos):
        idx = bisect_left(gt_ts, ts)
        candidates = []
        if idx < len(gt_ts):
            candidates.append(idx)
        if idx > 0:
            candidates.append(idx - 1)
        if not candidates:
            continue
        best = min(candidates, key=lambda i: abs(gt_ts[i] - ts))
        if best in used_gt or abs(gt_ts[best] - ts) > max_delta_ns:
            continue
        used_gt.add(best)
        matched_est.append(pos)
        matched_gt.append(gt_pos[best])
    return matched_est, matched_gt


def _mean(points: list[Vec3]) -> Vec3:
    n = float(len(points))
    return (sum(p[0] for p in points) / n, sum(p[1] for p in points) / n, sum(p[2] for p in points) / n)


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _norm(a: Vec3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _mat_vec_mul(matrix: Mat3, point: Vec3) -> Vec3:
    return (
        matrix[0][0] * point[0] + matrix[0][1] * point[1] + matrix[0][2] * point[2],
        matrix[1][0] * point[0] + matrix[1][1] * point[1] + matrix[1][2] * point[2],
        matrix[2][0] * point[0] + matrix[2][1] * point[1] + matrix[2][2] * point[2],
    )


def _power_iteration_symmetric4(matrix: list[list[float]], iterations: int = 80) -> tuple[float, float, float, float]:
    vector = [1.0, 0.0, 0.0, 0.0]
    for _ in range(iterations):
        next_vector = [sum(matrix[row][col] * vector[col] for col in range(4)) for row in range(4)]
        length = math.sqrt(sum(v * v for v in next_vector))
        if length == 0.0:
            break
        vector = [v / length for v in next_vector]
    return (vector[0], vector[1], vector[2], vector[3])


def _quat_to_matrix(q: tuple[float, float, float, float]) -> Mat3:
    w, x, y, z = q
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def _align_se3(source: list[Vec3], target: list[Vec3]) -> list[Vec3]:
    source_np = np.asarray(source, dtype=np.float64)
    target_np = np.asarray(target, dtype=np.float64)
    source_mean = source_np.mean(axis=0)
    target_mean = target_np.mean(axis=0)
    source_centered = source_np - source_mean
    target_centered = target_np - target_mean
    covariance = target_centered.T @ source_centered
    u, _, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(u @ vt) < 0.0:
        correction[-1, -1] = -1.0
    rotation = u @ correction @ vt
    aligned = (rotation @ source_centered.T).T + target_mean
    return [tuple(map(float, row)) for row in aligned]


def _rmse(values: list[float]) -> float:
    if not values:
        return math.nan
    return math.sqrt(sum(value * value for value in values) / float(len(values)))


def _trajectory_metrics(aligned_est: list[Vec3], gt: list[Vec3], rpe_delta: int) -> dict[str, float | int]:
    errors = [_norm(_sub(est, truth)) for est, truth in zip(aligned_est, gt)]
    rpe_errors: list[float] = []
    for idx in range(0, len(aligned_est) - rpe_delta):
        est_delta = _sub(aligned_est[idx + rpe_delta], aligned_est[idx])
        gt_delta = _sub(gt[idx + rpe_delta], gt[idx])
        rpe_errors.append(_norm(_sub(est_delta, gt_delta)))

    return {
        "matched_pairs": len(aligned_est),
        "ate_rmse_m": _rmse(errors),
        "ate_mean_m": sum(errors) / float(len(errors)) if errors else math.nan,
        "ate_max_m": max(errors) if errors else math.nan,
        "rpe_delta_frames": rpe_delta,
        "rpe_trans_rmse_m": _rmse(rpe_errors),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--estimate", required=True, type=Path)
    parser.add_argument("--out-json", required=True, type=Path)
    parser.add_argument("--max-association-dt-ms", type=float, default=50.0)
    parser.add_argument("--min-pairs", type=int, default=50)
    parser.add_argument("--rpe-delta-frames", type=int, default=10)
    parser.add_argument("--max-ate-rmse", type=float, default=2.5)
    parser.add_argument("--max-rpe-trans-rmse", type=float, default=1.0)
    args = parser.parse_args()

    gt_ts, gt_pos = _read_ground_truth(args.dataset)
    estimate_ts, estimate_pos = _read_estimate(args.estimate)
    matched_est, matched_gt = _associate_by_nearest(
        estimate_ts,
        estimate_pos,
        gt_ts,
        gt_pos,
        int(args.max_association_dt_ms * 1_000_000.0),
    )
    if len(matched_est) < args.min_pairs:
        raise RuntimeError(f"not enough timestamp matches: {len(matched_est)} < {args.min_pairs}")

    aligned_est = _align_se3(matched_est, matched_gt)
    metrics = _trajectory_metrics(aligned_est, matched_gt, max(1, args.rpe_delta_frames))
    metrics["max_association_dt_ms"] = float(args.max_association_dt_ms)
    metrics["threshold_max_ate_rmse_m"] = float(args.max_ate_rmse)
    metrics["threshold_max_rpe_trans_rmse_m"] = float(args.max_rpe_trans_rmse)

    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(
        "EuRoC regression metrics: "
        f"pairs={metrics['matched_pairs']} "
        f"ate_rmse={metrics['ate_rmse_m']:.4f}m "
        f"rpe_rmse={metrics['rpe_trans_rmse_m']:.4f}m"
    )

    if metrics["ate_rmse_m"] > args.max_ate_rmse:
        raise RuntimeError(f"ATE RMSE too high: {metrics['ate_rmse_m']:.4f} > {args.max_ate_rmse:.4f}")
    if metrics["rpe_trans_rmse_m"] > args.max_rpe_trans_rmse:
        raise RuntimeError(
            f"RPE translation RMSE too high: {metrics['rpe_trans_rmse_m']:.4f} > {args.max_rpe_trans_rmse:.4f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
