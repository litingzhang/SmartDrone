#!/usr/bin/env python3
"""Minimal XFeat smoke test for CM5 / Linux aarch64."""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

import torch


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a small XFeat CPU benchmark.")
    parser.add_argument(
        "--repo",
        type=Path,
        default=Path.home() / "third_party" / "accelerated_features",
        help="Path to cloned verlab/accelerated_features repository.",
    )
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--top-k", type=int, default=1024)
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--runs", type=int, default=10)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    repo_root = args.repo.resolve()
    if not repo_root.exists():
        print(f"[xfeat] repo not found: {repo_root}", file=sys.stderr)
        return 1

    sys.path.insert(0, str(repo_root))

    try:
        from modules.xfeat import XFeat
    except Exception as exc:  # pragma: no cover - diagnostic path
        print(f"[xfeat] failed to import XFeat from {repo_root}: {exc}", file=sys.stderr)
        return 2

    torch.set_grad_enabled(False)
    device = torch.device("cpu")
    model = XFeat(top_k=args.top_k).to(device).eval()

    image = torch.rand(1, 3, args.height, args.width, device=device)

    for _ in range(args.warmup):
        _ = model.detectAndCompute(image, top_k=args.top_k)[0]

    times_ms = []
    last_count = 0
    for _ in range(args.runs):
        start = time.perf_counter()
        output = model.detectAndCompute(image, top_k=args.top_k)[0]
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        times_ms.append(elapsed_ms)
        last_count = int(output["keypoints"].shape[0])

    mean_ms = statistics.mean(times_ms)
    p95_ms = max(times_ms) if len(times_ms) < 20 else statistics.quantiles(times_ms, n=20)[18]
    fps = 1000.0 / mean_ms if mean_ms > 0 else 0.0

    print(f"[xfeat] repo={repo_root}")
    print(f"[xfeat] torch={torch.__version__} device={device.type}")
    print(f"[xfeat] input=1x3x{args.height}x{args.width} top_k={args.top_k}")
    print(f"[xfeat] keypoints={last_count}")
    print(f"[xfeat] mean_ms={mean_ms:.2f} p95_ms={p95_ms:.2f} fps={fps:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
