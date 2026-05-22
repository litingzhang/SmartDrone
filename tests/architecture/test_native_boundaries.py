#!/usr/bin/env python3
"""Architecture boundary checks for the native runtime source tree."""

from __future__ import annotations

import sys
from pathlib import Path


ALLOWED_ADAPTER_APPLICATION_INCLUDES = {
    "src/native/adapters/camera/camera_provider_factory.h",
    "src/native/adapters/imu/icm42688/icm42688_imu.h",
    "src/native/adapters/imu/icm42688_imu_provider.h",
    "src/native/adapters/imu/icm42688_sample_source.cpp",
    "src/native/adapters/imu/imu_sample_source_provider.cpp",
    "src/native/adapters/slam/dpvo_tensorrt_engine.h",
    "src/native/adapters/slam/orb_slam3_runtime.h",
    "src/native/adapters/slam/slam_engine_factory.h",
    "src/native/adapters/slam/slam_mode_strategy.h",
    "src/native/adapters/slam/visual_feature_frontend_client.h",
    "src/native/adapters/telemetry/px4_mavlink_gateway.h",
}


def relative_path(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def source_files(root: Path, directory: str) -> list[Path]:
    base = root / directory
    return [
        path
        for path in base.rglob("*")
        if path.suffix in {".h", ".hpp", ".c", ".cc", ".cpp"}
        and "adapters/slam/orb_slam3" not in path.as_posix()
    ]


def adapter_application_include_violations(root: Path) -> list[str]:
    violations: list[str] = []
    for path in source_files(root, "src/native/adapters"):
        rel = relative_path(path, root)
        if rel in ALLOWED_ADAPTER_APPLICATION_INCLUDES:
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        if '#include "core/application/' in text:
            violations.append(rel)
    return violations


def misplaced_application_composition(root: Path) -> list[str]:
    misplaced: list[str] = []
    runtime_dir = root / "src/native/adapters/runtime"
    if runtime_dir.exists():
        misplaced.extend(
            relative_path(path, root)
            for path in source_files(root, "src/native/adapters/runtime"))
    return misplaced


def main() -> int:
    root = Path(sys.argv[1]).resolve()
    violations = []
    for rel in adapter_application_include_violations(root):
        violations.append(f"adapter must not include core/application directly: {rel}")
    for rel in misplaced_application_composition(root):
        violations.append(f"application composition must live under src/native/app: {rel}")
    if violations:
        sys.stderr.write("\n".join(violations) + "\n")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
