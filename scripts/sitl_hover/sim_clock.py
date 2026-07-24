"""Flight-stage clocks for PX4/Gazebo scenario orchestration."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Protocol


@dataclass(frozen=True)
class FlightClockMark:
    time_s: float
    reset_counter: int


@dataclass(frozen=True)
class FlightClockSnapshot:
    time_s: float
    received_monotonic_s: float
    rate: float | None
    reset_counter: int


class FlightClock(Protocol):
    source: str

    @property
    def rate(self) -> float | None:
        ...

    def mark(self) -> FlightClockMark:
        ...

    def elapsed_s(self, mark: FlightClockMark) -> float:
        ...

    def close(self) -> None:
        ...


class SteadyFlightClock:
    source = "steady"

    @property
    def rate(self) -> float:
        return 1.0

    def mark(self) -> FlightClockMark:
        return FlightClockMark(time.monotonic(), 0)

    def elapsed_s(self, mark: FlightClockMark) -> float:
        return time.monotonic() - mark.time_s

    def close(self) -> None:
        pass


class GazeboClockReader:
    source = "gazebo"

    def __init__(
        self,
        world_name: str,
        environment: dict[str, str],
        log_path: Path,
        *,
        executable: str = "gz",
        stall_timeout_s: float = 10.0,
        rate_window_s: float = 2.0,
    ) -> None:
        self.topic = f"/world/{world_name}/clock"
        self._stall_timeout_s = stall_timeout_s
        self._rate_window_s = rate_window_s
        self._snapshot: FlightClockSnapshot | None = None
        self._last_advance_monotonic_s: float | None = None
        self._rate_samples: deque[tuple[float, float]] = deque()
        self._error: str | None = None
        self._closing = False
        self._stderr = log_path.open("ab", buffering=0)
        try:
            self._process = subprocess.Popen(
                [executable, "topic", "-e", "--json-output", "-t", self.topic],
                env=environment,
                stdout=subprocess.PIPE,
                stderr=self._stderr,
                start_new_session=True,
            )
        except BaseException:
            self._stderr.close()
            raise
        self._thread = threading.Thread(target=self._consume_output, daemon=True)
        self._thread.start()

    @property
    def rate(self) -> float | None:
        snapshot = self._snapshot
        return None if snapshot is None else snapshot.rate

    def wait_ready(
        self,
        timeout_s: float,
        health_check: Callable[[], None] | None = None,
    ) -> FlightClockSnapshot:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            snapshot = self._snapshot
            if snapshot is not None and snapshot.rate is not None and snapshot.rate > 0.0:
                return snapshot
            self._raise_if_exited()
            if health_check is not None:
                health_check()
            time.sleep(0.02)
        raise TimeoutError(f"Gazebo clock did not advance within {timeout_s:.1f}s: {self.topic}")

    def mark(self) -> FlightClockMark:
        snapshot = self._checked_snapshot()
        return FlightClockMark(snapshot.time_s, snapshot.reset_counter)

    def elapsed_s(self, mark: FlightClockMark) -> float:
        snapshot = self._checked_snapshot()
        if snapshot.reset_counter != mark.reset_counter or snapshot.time_s < mark.time_s:
            raise RuntimeError("Gazebo simulation clock reset during a flight stage")
        return snapshot.time_s - mark.time_s

    def close(self) -> None:
        if self._closing:
            return
        self._closing = True
        self._signal(signal.SIGTERM)
        try:
            self._process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._signal(signal.SIGKILL)
            self._process.wait(timeout=2.0)
        self._thread.join(timeout=2.0)
        if self._process.stdout is not None:
            self._process.stdout.close()
        self._stderr.close()

    def _checked_snapshot(self) -> FlightClockSnapshot:
        snapshot = self._snapshot
        if snapshot is None:
            self._raise_if_exited()
            raise RuntimeError(f"Gazebo clock has not produced a sample: {self.topic}")
        last_advance = self._last_advance_monotonic_s
        if last_advance is not None and time.monotonic() - last_advance > self._stall_timeout_s:
            raise RuntimeError(f"Gazebo simulation clock stopped advancing: {self.topic}")
        self._raise_if_exited()
        return snapshot

    def _raise_if_exited(self) -> None:
        status = self._process.poll()
        if status is None:
            return
        detail = f": {self._error}" if self._error else ""
        raise RuntimeError(f"Gazebo clock reader exited with status {status}{detail}")

    def _consume_output(self) -> None:
        assert self._process.stdout is not None
        buffer = ""
        try:
            while True:
                chunk = os.read(self._process.stdout.fileno(), 65536)
                if not chunk:
                    break
                buffer += chunk.decode("utf-8", errors="replace")
                buffer = self._consume_json(buffer)
        except OSError as error:
            if not self._closing:
                self._error = str(error)
        if buffer.strip() and not self._closing:
            self._consume_json(buffer)

    def _consume_json(self, buffer: str) -> str:
        decoder = json.JSONDecoder()
        while buffer:
            buffer = buffer.lstrip()
            if not buffer:
                return ""
            if not buffer.startswith("{"):
                object_start = buffer.find("{")
                if object_start < 0:
                    return buffer[-4096:]
                buffer = buffer[object_start:]
            try:
                record, end = decoder.raw_decode(buffer)
            except json.JSONDecodeError:
                return buffer[-1048576:]
            self._record_message(record)
            buffer = buffer[end:]
        return ""

    def _record_message(self, record: object) -> None:
        if not isinstance(record, dict) or not isinstance(record.get("sim"), dict):
            return
        sim_time_s = self._message_time_s(record.get("sim"))
        if sim_time_s is None:
            return
        received_s = time.monotonic()
        real_time_s = self._message_time_s(record.get("real"))
        rate_time_s = received_s if real_time_s is None else real_time_s
        self._record_sample(sim_time_s, received_s, rate_time_s)

    @staticmethod
    def _message_time_s(value: object) -> float | None:
        if not isinstance(value, dict):
            return None
        try:
            return float(value["sec"]) + float(value.get("nsec", 0)) / 1_000_000_000.0
        except (KeyError, TypeError, ValueError):
            return None

    def _record_sample(
        self,
        sim_time_s: float,
        received_s: float,
        rate_time_s: float,
    ) -> None:
        previous = self._snapshot
        reset_counter = 0 if previous is None else previous.reset_counter
        if previous is not None and sim_time_s < previous.time_s:
            reset_counter += 1
            self._rate_samples.clear()
        if self._rate_samples and rate_time_s < self._rate_samples[-1][1]:
            self._rate_samples.clear()
        if previous is None or sim_time_s > previous.time_s:
            self._last_advance_monotonic_s = received_s
        self._rate_samples.append((sim_time_s, rate_time_s))
        self._trim_rate_samples(rate_time_s)
        rate = self._calculate_rate()
        self._snapshot = FlightClockSnapshot(sim_time_s, received_s, rate, reset_counter)

    def _trim_rate_samples(self, received_s: float) -> None:
        while len(self._rate_samples) > 2:
            if received_s - self._rate_samples[0][1] <= self._rate_window_s:
                break
            self._rate_samples.popleft()

    def _calculate_rate(self) -> float | None:
        if len(self._rate_samples) < 2:
            return None
        first_sim, first_wall = self._rate_samples[0]
        last_sim, last_wall = self._rate_samples[-1]
        wall_delta = last_wall - first_wall
        if wall_delta <= 0.0:
            return None
        return max(0.0, (last_sim - first_sim) / wall_delta)

    def _signal(self, value: signal.Signals) -> None:
        if self._process.poll() is not None:
            return
        try:
            os.killpg(self._process.pid, value)
        except ProcessLookupError:
            pass
