#!/usr/bin/env python3
"""Terminal DFX panel for EPG graphs.

The first version can run with simulated live data from a DOT topology. A future
EPG runtime exporter can write the same snapshot JSON shape and feed this tool
with --snapshot.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import re
import select
import shutil
import signal
import sys
import termios
import time
import tty
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional


ANSI_RESET = "\033[0m"
ANSI_CLEAR = "\033[2J\033[H"
ANSI_DIM = "\033[2m"
ANSI_BOLD = "\033[1m"
ANSI_RED = "\033[31m"
ANSI_GREEN = "\033[32m"
ANSI_YELLOW = "\033[33m"
ANSI_CYAN = "\033[36m"


@dataclass
class Task:
    name: str
    type_name: str
    trigger: str
    interval_ms: Optional[int] = None
    budget_us: Optional[int] = None
    trigger_queues: List[str] = field(default_factory=list)


@dataclass
class Queue:
    name: str
    type_name: str
    from_task: str
    from_port: int
    to_task: str
    to_port: int
    depth: int
    overflow: str


@dataclass
class Topology:
    tasks: Dict[str, Task]
    queues: List[Queue]


@dataclass
class QueueState:
    size: int = 0
    pushed: int = 0
    popped: int = 0
    dropped_newest: int = 0
    overwritten_oldest: int = 0


@dataclass
class TaskState:
    last_loop_us: int = 0
    max_loop_us: int = 0
    loop_count: int = 0
    error_count: int = 0
    idle_wakeups: int = 0


@dataclass
class Snapshot:
    queues: Dict[str, QueueState]
    tasks: Dict[str, TaskState]
    timestamp_ms: int


def strip_ansi(text: str) -> str:
    return re.sub(r"\x1b\[[0-9;]*m", "", text)


def fit(text: str, width: int) -> str:
    if width <= 0:
        return ""
    plain = strip_ansi(text)
    if len(plain) <= width:
        return text + (" " * (width - len(plain)))
    if width <= 1:
        return plain[:width]
    return plain[: width - 1] + "…"


def color_for_ratio(ratio: float, bad_delta: bool = False) -> str:
    if bad_delta or ratio >= 0.80:
        return ANSI_RED
    if ratio >= 0.50:
        return ANSI_YELLOW
    return ANSI_GREEN


def bar(ratio: float, width: int = 18, bad_delta: bool = False) -> str:
    ratio = max(0.0, min(1.0, ratio))
    filled = int(round(ratio * width))
    color = color_for_ratio(ratio, bad_delta)
    return f"{color}{'█' * filled}{ANSI_DIM}{'░' * (width - filled)}{ANSI_RESET}"


def parse_label_fields(label: str) -> Dict[str, str]:
    label = label.strip()
    if label.startswith("{") and label.endswith("}"):
        label = label[1:-1]
    parts = [part.strip() for part in label.split("|") if part.strip()]
    result: Dict[str, str] = {}
    if parts:
        result["display"] = parts[0]
    for part in parts[1:]:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        result[key.strip()] = value.strip()
    return result


def parse_attrs(attr_text: str) -> Dict[str, str]:
    attrs: Dict[str, str] = {}
    pattern = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)\s*=\s*\"((?:\\.|[^\"])*)\"")
    for match in pattern.finditer(attr_text):
        attrs[match.group(1)] = match.group(2).replace("\\n", "\n")
    return attrs


def extract_subgraph(dot_text: str, graph_name: str) -> str:
    marker = f"subgraph {graph_name}"
    start = dot_text.find(marker)
    if start < 0:
        raise ValueError(f"subgraph not found: {graph_name}")
    brace = dot_text.find("{", start)
    if brace < 0:
        raise ValueError(f"subgraph missing body: {graph_name}")
    depth = 0
    for index in range(brace, len(dot_text)):
        char = dot_text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return dot_text[brace + 1 : index]
    raise ValueError(f"subgraph not closed: {graph_name}")


def parse_edge_label(label: str) -> Dict[str, str]:
    lines = [line.strip() for line in label.splitlines() if line.strip()]
    if not lines:
        return {}
    result = {"type": lines[0]}
    for line in lines[1:]:
        tokens = line.split()
        for token in tokens:
            if "=" in token:
                key, value = token.split("=", 1)
                result[key.strip()] = value.strip()
            elif token in ("drop_newest", "overwrite_oldest", "tail_drop", "circular_overwrite"):
                result["overflow"] = token
    return result


def parse_topology(dot_path: Path, graph_name: str) -> Topology:
    body = extract_subgraph(dot_path.read_text(encoding="utf-8"), graph_name)
    tasks: Dict[str, Task] = {}
    queues: List[Queue] = []

    node_pattern = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*\[(.*?)\]\s*;", re.S | re.M)
    edge_pattern = re.compile(
        r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*->\s*([A-Za-z_][A-Za-z0-9_]*)\s*\[(.*?)\]\s*;",
        re.S | re.M,
    )

    for match in node_pattern.finditer(body):
        name = match.group(1)
        attrs = parse_attrs(match.group(2))
        if "label" not in attrs:
            continue
        fields = parse_label_fields(attrs["label"])
        trigger_queues = [
            item.strip() for item in fields.get("trigger_queues", "").split("+") if item.strip()
        ]
        interval = fields.get("interval_ms")
        budget = fields.get("budget_us")
        tasks[name] = Task(
            name=name,
            type_name=fields.get("type", name),
            trigger=fields.get("trigger", "periodic"),
            interval_ms=int(interval) if interval and interval.isdigit() else None,
            budget_us=int(budget) if budget and budget.isdigit() else None,
            trigger_queues=trigger_queues,
        )

    for match in edge_pattern.finditer(body):
        from_task = match.group(1)
        to_task = match.group(2)
        attrs = parse_attrs(match.group(3))
        edge = parse_edge_label(attrs.get("label", ""))
        from_port = int(attrs.get("taillabel", "0"))
        to_port = int(attrs.get("headlabel", "0"))
        queue_name = f"{from_task}_{from_port}_to_{to_task}_{to_port}"
        queues.append(
            Queue(
                name=queue_name,
                type_name=edge.get("type", "Unknown"),
                from_task=from_task,
                from_port=from_port,
                to_task=to_task,
                to_port=to_port,
                depth=int(edge.get("depth", "1")),
                overflow=edge.get("overflow", "drop_newest"),
            )
        )

    return Topology(tasks=tasks, queues=queues)


def load_snapshot(path: Path) -> Optional[Snapshot]:
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    queues = {
        name: QueueState(
            size=int(item.get("size", 0)),
            pushed=int(item.get("pushed", 0)),
            popped=int(item.get("popped", 0)),
            dropped_newest=int(item.get("droppedNewest", item.get("dropped_newest", 0))),
            overwritten_oldest=int(item.get("overwrittenOldest", item.get("overwritten_oldest", 0))),
        )
        for name, item in data.get("queues", {}).items()
    }
    tasks = {
        name: TaskState(
            last_loop_us=int(item.get("lastLoopUs", item.get("last_loop_us", 0))),
            max_loop_us=int(item.get("maxLoopUs", item.get("max_loop_us", 0))),
            loop_count=int(item.get("loopCount", item.get("loop_count", 0))),
            error_count=int(item.get("errorCount", item.get("error_count", 0))),
            idle_wakeups=int(item.get("idleWakeups", item.get("idle_wakeups", 0))),
        )
        for name, item in data.get("tasks", {}).items()
    }
    return Snapshot(
        queues=queues,
        tasks=tasks,
        timestamp_ms=int(data.get("timestampMs", time.time() * 1000)),
    )


class DemoSource:
    def __init__(self, topology: Topology) -> None:
        self._rng = random.Random(7)
        self._queues = {queue.name: QueueState() for queue in topology.queues}
        self._tasks = {task.name: TaskState() for task in topology.tasks.values()}
        self._topology = topology

    def next(self) -> Snapshot:
        for queue in self._topology.queues:
            state = self._queues[queue.name]
            incoming = self._rng.randint(0, 3)
            outgoing = self._rng.randint(0, 3)
            state.pushed += incoming
            state.popped += min(state.size + incoming, outgoing)
            state.size = max(0, min(queue.depth, state.size + incoming - outgoing))
            if queue.depth <= 1 and self._rng.random() < 0.12:
                state.size = queue.depth
            if state.size >= queue.depth and self._rng.random() < 0.08:
                if queue.overflow == "drop_newest":
                    state.dropped_newest += 1
                else:
                    state.overwritten_oldest += 1

        for task in self._topology.tasks.values():
            state = self._tasks[task.name]
            base = task.interval_ms or 4
            jitter = self._rng.randint(50, 1800)
            state.last_loop_us = max(20, base * 120 + jitter)
            if self._rng.random() < 0.05:
                state.last_loop_us *= self._rng.randint(2, 5)
            state.max_loop_us = max(state.max_loop_us, state.last_loop_us)
            state.loop_count += self._rng.randint(1, 8)
            if self._rng.random() < 0.01:
                state.error_count += 1

        return Snapshot(
            queues=dict(self._queues),
            tasks=dict(self._tasks),
            timestamp_ms=int(time.time() * 1000),
        )


def task_budget_us(task: Task) -> int:
    if task.budget_us:
        return task.budget_us
    if task.interval_ms:
        return max(1000, task.interval_ms * 1000)
    return 5000


def queue_bad_delta(current: QueueState, previous: Optional[QueueState]) -> bool:
    if previous is None:
        return current.dropped_newest > 0 or current.overwritten_oldest > 0
    return (
        current.dropped_newest > previous.dropped_newest
        or current.overwritten_oldest > previous.overwritten_oldest
    )


def task_bad_delta(current: TaskState, previous: Optional[TaskState]) -> bool:
    return previous is not None and current.error_count > previous.error_count


def render(topology: Topology, snapshot: Snapshot, previous: Optional[Snapshot], graph_name: str) -> str:
    width = shutil.get_terminal_size((120, 36)).columns
    now = time.strftime("%H:%M:%S")
    lines = [
        f"{ANSI_BOLD}{ANSI_CYAN}EPG DFX{ANSI_RESET} {graph_name}  {now}  "
        f"{ANSI_DIM}q: quit  Ctrl-C: exit{ANSI_RESET}",
        "",
        f"{ANSI_BOLD}Queues{ANSI_RESET}",
        fit("QUEUE", 46)
        + " "
        + fit("TYPE", 26)
        + " "
        + fit("SIZE", 8)
        + " "
        + fit("CONGESTION", 28)
        + " "
        + fit("DROP/OW", 14),
    ]

    previous_queues = previous.queues if previous else {}
    sorted_queues = sorted(
        topology.queues,
        key=lambda q: (
            -(snapshot.queues.get(q.name, QueueState()).size / max(1, q.depth)),
            q.name,
        ),
    )
    for queue in sorted_queues:
        state = snapshot.queues.get(queue.name, QueueState())
        ratio = state.size / max(1, queue.depth)
        bad = queue_bad_delta(state, previous_queues.get(queue.name))
        colored_bar = bar(ratio, 18, bad)
        ratio_text = f"{state.size}/{queue.depth}"
        drops = f"{state.dropped_newest}/{state.overwritten_oldest}"
        lines.append(
            fit(queue.name, 46)
            + " "
            + fit(queue.type_name, 26)
            + " "
            + fit(ratio_text, 8)
            + " "
            + fit(f"{colored_bar} {ratio * 100:5.1f}%", 28)
            + " "
            + fit(drops, 14)
        )

    lines.extend(["", f"{ANSI_BOLD}Tasks{ANSI_RESET}"])
    lines.append(
        fit("TASK", 38)
        + " "
        + fit("TRIGGER", 24)
        + " "
        + fit("LAST/MAX", 18)
        + " "
        + fit("LATENCY", 28)
        + " "
        + fit("LOOP/ERR", 16)
    )

    previous_tasks = previous.tasks if previous else {}
    sorted_tasks = sorted(
        topology.tasks.values(),
        key=lambda t: (
            -(snapshot.tasks.get(t.name, TaskState()).last_loop_us / max(1, task_budget_us(t))),
            t.name,
        ),
    )
    for task in sorted_tasks:
        state = snapshot.tasks.get(task.name, TaskState())
        budget = task_budget_us(task)
        ratio = state.last_loop_us / max(1, budget)
        bad = task_bad_delta(state, previous_tasks.get(task.name))
        latency = f"{state.last_loop_us / 1000:.2f}/{state.max_loop_us / 1000:.2f}ms"
        trigger = task.trigger
        if task.interval_ms is not None:
            trigger += f" {task.interval_ms}ms"
        loops = f"{state.loop_count}/{state.error_count}"
        lines.append(
            fit(task.name, 38)
            + " "
            + fit(trigger, 24)
            + " "
            + fit(latency, 18)
            + " "
            + fit(f"{bar(ratio, 18, bad)} {min(ratio, 9.99) * 100:5.1f}%", 28)
            + " "
            + fit(loops, 16)
        )

    legend = (
        f"{ANSI_GREEN}green <50%{ANSI_RESET}  "
        f"{ANSI_YELLOW}yellow 50-80%{ANSI_RESET}  "
        f"{ANSI_RED}red >=80% or new drop/error{ANSI_RESET}"
    )
    lines.extend(["", legend])
    return ANSI_CLEAR + "\n".join(line[: max(width + 32, width)] for line in lines) + "\n"


class KeyReader:
    def __init__(self) -> None:
        self._enabled = False
        self._old_settings: Optional[List[object]] = None

    def __enter__(self) -> "KeyReader":
        if sys.stdin.isatty():
            self._old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self._enabled = True
        return self

    def __exit__(self, _exc_type: object, _exc: object, _tb: object) -> None:
        if self._enabled and self._old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)

    def read(self) -> Optional[str]:
        if not self._enabled:
            return None
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            return None
        return sys.stdin.read(1)


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="EPG terminal DFX panel")
    parser.add_argument("--dot", default="config/epg/epg_topology.dot")
    parser.add_argument("--graph", default="cluster_slam_session_graph")
    parser.add_argument("--snapshot", help="JSON snapshot path. If omitted, demo data is used.")
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args(argv)

    topology = parse_topology(Path(args.dot), args.graph)
    demo = DemoSource(topology)
    previous: Optional[Snapshot] = None
    snapshot_path = Path(args.snapshot) if args.snapshot else None
    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    with KeyReader() as keys:
        while running:
            snapshot = load_snapshot(snapshot_path) if snapshot_path else None
            if snapshot is None:
                snapshot = demo.next()
            sys.stdout.write(render(topology, snapshot, previous, args.graph))
            sys.stdout.flush()
            previous = snapshot
            if args.once:
                break
            deadline = time.time() + max(0.1, args.interval)
            while running and time.time() < deadline:
                key = keys.read()
                if key in ("q", "Q"):
                    running = False
                    break
                time.sleep(0.05)

    sys.stdout.write(ANSI_RESET)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
