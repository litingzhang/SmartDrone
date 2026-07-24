#!/usr/bin/env python3
"""Minimal managed PX4 launcher used by runner provenance tests."""

from __future__ import annotations

import os
import signal
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from sitl_hover.provenance import write_launch_attestation  # noqa: E402


def main() -> int:
    log_dir = Path(os.environ["SMART_DRONE_PX4_LOG_DIR"])
    log_dir.mkdir(parents=True, exist_ok=True)
    (log_dir / "fake.ulg").write_bytes(b"managed fixture ULog")
    write_launch_attestation(
        Path(os.environ["SMART_DRONE_PX4_ATTESTATION_FILE"]),
        role="px4", run_id=os.environ["SMART_DRONE_SITL_RUN_ID"],
        profile=os.environ["SMART_DRONE_SIM_PROFILE"],
        seed=int(os.environ["SMART_DRONE_SIM_SEED"]),
        details={"launcher": "fake_px4_launcher.py"},
    )
    Path(os.environ["SMART_DRONE_PX4_READY_FILE"]).write_text(
        "ready\n", encoding="utf-8",
    )
    stopped = False

    def stop_process(_number: int, _frame: object) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop_process)
    signal.signal(signal.SIGTERM, stop_process)
    while not stopped:
        time.sleep(0.02)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
