#!/usr/bin/env python3
"""Tests for the Gazebo simulation-time wrench helper."""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
import time
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "apply_gz_wrench.sh"
FAKE_GZ = """#!/usr/bin/env python3
import json
import os
import signal
import sys
import time

args = sys.argv[1:]
with open(os.environ["FAKE_GZ_LOG"], "a", encoding="utf-8") as stream:
    stream.write(json.dumps({"args": args, "time": time.monotonic()}) + "\\n")
joined = " ".join(args)
marker = os.environ.get("FAKE_FORCE_MARKER", "")
if "--json-output" not in args:
    if marker and "wrench/persistent" in joined:
        time.sleep(float(os.environ.get("FAKE_PERSISTENT_DELAY_S", "0")))
        with open(marker, "w", encoding="utf-8") as stream:
            stream.write("active\\n")
    if marker and "wrench/clear" in joined and os.path.exists(marker):
        counter_path = marker + ".clear_count"
        count = 0
        if os.path.exists(counter_path):
            with open(counter_path, encoding="utf-8") as stream:
                count = int(stream.read())
        count += 1
        with open(counter_path, "w", encoding="utf-8") as stream:
            stream.write(str(count))
        if count <= int(os.environ.get("FAKE_CLEAR_STALL_COUNT", "0")):
            time.sleep(float(os.environ.get("FAKE_CLEAR_DELAY_AFTER_FORCE_S", "0")))
        os.unlink(marker)
    raise SystemExit(0)
mode = os.environ.get("FAKE_CLOCK_MODE", "advance")
if mode == "unavailable":
    raise SystemExit(1)
time.sleep(float(os.environ.get("FAKE_CLOCK_START_DELAY_S", "0")))
for value in json.loads(os.environ.get("FAKE_CLOCK_SAMPLES", "[0, 100000000]")):
    print(json.dumps({"sim": {"sec": value // 1000000000, "nsec": value % 1000000000}}), flush=True)
    time.sleep(float(os.environ.get("FAKE_CLOCK_DELAY_S", "0")))
if mode == "hold":
    signal.pause()
"""


class ApplyGazeboWrenchTest(unittest.TestCase):
    def _environment(self, root: Path) -> dict[str, str]:
        fake_bin = root / "bin"
        fake_bin.mkdir()
        fake_gz = fake_bin / "gz"
        fake_gz.write_text(FAKE_GZ, encoding="utf-8")
        fake_gz.chmod(0o755)
        environment = dict(os.environ)
        environment.update({
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "FAKE_GZ_LOG": str(root / "gz.log"),
            "FAKE_FORCE_MARKER": str(root / "force.active"),
            "SMART_DRONE_GZ_CLOCK_READY_TIMEOUT_S": "0.2",
            "SMART_DRONE_GZ_WRENCH_TIMEOUT_S": "0.4",
        })
        return environment

    def _run(self, root: Path, environment: dict[str, str], duration_ms: int = 100) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                str(SCRIPT), "--world", "test_world", "--model", "test_model",
                "--force-n", "4.0", "--duration-ms", str(duration_ms),
            ],
            cwd=REPO_ROOT,
            env=environment,
            capture_output=True,
            text=True,
            timeout=3.0,
        )

    @staticmethod
    def _calls(root: Path) -> list[dict[str, object]]:
        return [json.loads(line) for line in (root / "gz.log").read_text(encoding="utf-8").splitlines()]

    def test_clock_ready_precedes_wrench_and_controls_duration(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            environment = self._environment(root)
            environment.update({
                "FAKE_CLOCK_SAMPLES": json.dumps([
                    1_000_000_000, 1_000_000_000, 1_040_000_000, 1_100_000_000,
                ]),
                "FAKE_CLOCK_DELAY_S": "0.06",
                "FAKE_CLOCK_START_DELAY_S": "0.12",
            })
            result = self._run(root, environment)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            calls = self._calls(root)
            clock_index = next(
                index for index, call in enumerate(calls)
                if "/clock" in " ".join(call["args"])
            )
            persistent_index = next(
                index for index, call in enumerate(calls)
                if "persistent" in " ".join(call["args"])
            )
            persistent = calls[persistent_index]
            final_clear = calls[-1]
            self.assertLess(clock_index, persistent_index)
            self.assertGreaterEqual(
                float(persistent["time"]) - float(calls[clock_index]["time"]), 0.1,
            )
            self.assertIn("/world/test_world/wrench/clear", " ".join(final_clear["args"]))
            self.assertGreaterEqual(float(final_clear["time"]) - float(persistent["time"]), 0.15)

    def test_unavailable_clock_fails_and_clears_wrench(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            environment = self._environment(root)
            environment["FAKE_CLOCK_MODE"] = "unavailable"
            result = self._run(root, environment)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("clock", result.stderr)
            calls = self._calls(root)
            self.assertFalse(any(
                "persistent" in " ".join(call["args"]) for call in calls
            ))
            self.assertIn("/world/test_world/wrench/clear", " ".join(calls[-1]["args"]))

    def test_stalled_clock_times_out_and_clears_wrench(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            environment = self._environment(root)
            environment.update({
                "FAKE_CLOCK_MODE": "hold",
                "FAKE_CLOCK_SAMPLES": "[1000000000]",
                "SMART_DRONE_GZ_WRENCH_TIMEOUT_S": "0.2",
            })
            result = self._run(root, environment)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("did not advance", result.stderr)
            calls = self._calls(root)
            self.assertIn("/world/test_world/wrench/clear", " ".join(calls[-1]["args"]))

    def test_publish_delay_does_not_consume_wrench_wall_budget(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            environment = self._environment(root)
            environment.update({
                "FAKE_CLOCK_SAMPLES": json.dumps([
                    0, 50_000_000, 100_000_000, 150_000_000,
                    200_000_000, 250_000_000,
                ]),
                "FAKE_CLOCK_DELAY_S": "0.05",
                "FAKE_PERSISTENT_DELAY_S": "0.15",
                "SMART_DRONE_GZ_WRENCH_TIMEOUT_S": "0.22",
            })

            result = self._run(root, environment)

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            calls = self._calls(root)
            persistent = next(
                call for call in calls if "persistent" in " ".join(call["args"])
            )
            final_clear = calls[-1]
            self.assertGreaterEqual(
                float(final_clear["time"]) - float(persistent["time"]), 0.18,
            )

    def test_timed_out_clear_is_bounded_and_cleanup_retries(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            environment = self._environment(root)
            environment.update({
                "FAKE_CLOCK_SAMPLES": "[0, 100000000, 200000000]",
                "FAKE_CLOCK_DELAY_S": "0.02",
                "FAKE_CLEAR_STALL_COUNT": "1",
                "FAKE_CLEAR_DELAY_AFTER_FORCE_S": "1",
                "SMART_DRONE_GZ_COMMAND_TIMEOUT_S": "0.05",
            })
            started = time.monotonic()
            result = self._run(root, environment)
            elapsed = time.monotonic() - started

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("failed to clear persistent wrench", result.stderr)
            calls = self._calls(root)
            persistent_index = next(
                index for index, call in enumerate(calls)
                if "persistent" in " ".join(call["args"])
            )
            post_force_clears = [
                call for call in calls[persistent_index + 1:]
                if "wrench/clear" in " ".join(call["args"])
            ]
            self.assertEqual(len(post_force_clears), 3)
            self.assertFalse(Path(environment["FAKE_FORCE_MARKER"]).exists())
            self.assertLess(elapsed, 1.0)


if __name__ == "__main__":
    unittest.main()
