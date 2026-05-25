#!/usr/bin/env python3
"""Architecture boundary checks for the native runtime source tree."""

from __future__ import annotations

import sys
from pathlib import Path


ALLOWED_ADAPTER_APPLICATION_INCLUDES = set()

EPG_GRAPH_RUNTIME_ENTRYPOINT = (
    "src/native/core/application/epg/epg_graph_runtime.cpp"
)
EPG_RUNTIME_BOUNDARY_ALLOWED_FILES = {
    EPG_GRAPH_RUNTIME_ENTRYPOINT,
    "src/native/core/application/epg/epg_registry.cpp",
    "src/native/core/application/epg/epg_registry.h",
}

EPG_RUNTIME_BOUNDARY_PATTERNS = {
    "RegisterEpgTypes(": "EPG type registration must stay behind StartEpgGraph",
    "CompileEpgConfig(": "EPG config compilation must stay behind StartEpgGraph",
    "std::make_unique<Epg::EventPipelineGraph>":
        "runtime EPG graph construction must stay behind StartEpgGraph",
    "Epg::EventPipelineGraph graph":
        "runtime EPG graph construction must stay behind StartEpgGraph",
}

NON_EPG_THREAD_ALLOWED_FILES = {
    "src/native/common/epg/epg_internal.h",
    "src/native/common/epg/epg_task_runner.cpp",
}

NON_EPG_THREAD_PATTERNS = {
    "#include <thread>": "native runtime thread waiting must be owned by EPG",
    "std::thread": "native runtime threads must be owned by EPG",
    "std::jthread": "native runtime threads must be owned by EPG",
    "std::async": "native runtime async work must be owned by EPG",
    "std::this_thread::sleep_for": "native runtime sleeps must be owned by EPG",
    "std::this_thread::yield": "native runtime yields must be owned by EPG",
    "pthread_create": "native runtime threads must be owned by EPG",
    ".detach(": "native runtime detached threads are not allowed",
}

ORB_SLAM3_SOURCE_DIR = "src/native/adapters/slam/orb/orb_slam3"
ORB_SLAM3_THIRD_PARTY_DIR = "third_party/orb_slam3"
ORB_SLAM3_THREAD_PATTERNS = {
    "#include <thread>": "ORB-SLAM3 must be advanced by EPG tasks, not internal threads",
    "std::thread": "ORB-SLAM3 must be advanced by EPG tasks, not internal threads",
    "std::jthread": "ORB-SLAM3 must be advanced by EPG tasks, not internal threads",
    "std::async": "ORB-SLAM3 must be advanced by EPG tasks, not internal async work",
    "std::this_thread::sleep_for":
        "ORB-SLAM3 must be advanced by EPG tasks, not internal sleeps",
    "std::this_thread::yield":
        "ORB-SLAM3 must be advanced by EPG tasks, not internal yields",
    "pthread_create": "ORB-SLAM3 must be advanced by EPG tasks, not internal pthreads",
    ".detach(": "ORB-SLAM3 detached threads are not allowed",
}
ORB_SLAM3_SELF_SCHEDULING_PATTERNS = {
    "void LocalMapping::Run(": "ORB-SLAM3 local mapping must be advanced by EPG StepBackend",
    "void LoopClosing::Run(": "ORB-SLAM3 loop closing must be advanced by EPG StepBackend",
    "void Tracking::Run(": "ORB-SLAM3 tracking must be advanced by EPG tracking tasks",
}

NATIVE_LOCK_PATTERNS = {
    "#include <mutex>": "native runtime locks are not allowed",
    "#include <shared_mutex>": "native runtime locks are not allowed",
    "#include <condition_variable>": "native runtime condition variables are not allowed",
    "std::mutex": "native runtime locks are not allowed",
    "std::recursive_mutex": "native runtime locks are not allowed",
    "std::shared_mutex": "native runtime locks are not allowed",
    "std::timed_mutex": "native runtime locks are not allowed",
    "std::lock_guard": "native runtime locks are not allowed",
    "std::unique_lock": "native runtime locks are not allowed",
    "std::scoped_lock": "native runtime locks are not allowed",
    "std::shared_lock": "native runtime locks are not allowed",
    "std::condition_variable": "native runtime condition variables are not allowed",
    "std::once_flag": "native runtime once locks are not allowed",
    "std::call_once": "native runtime once locks are not allowed",
    "pthread_mutex": "native runtime locks are not allowed",
}


def relative_path(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def is_orb_slam3_path(rel: str) -> bool:
    return rel == ORB_SLAM3_SOURCE_DIR or rel.startswith(
        f"{ORB_SLAM3_SOURCE_DIR}/")


def source_files(root: Path, directory: str) -> list[Path]:
    base = root / directory
    return [
        path
        for path in base.rglob("*")
        if path.suffix in {".h", ".hpp", ".c", ".cc", ".cpp"}
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


def core_ports_application_include_violations(root: Path) -> list[str]:
    violations: list[str] = []
    for path in source_files(root, "src/native/core/ports"):
        rel = relative_path(path, root)
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


def runtime_epg_boundary_violations(root: Path) -> list[str]:
    violations: list[str] = []
    for path in source_files(root, "src/native/core/application"):
        rel = relative_path(path, root)
        if rel in EPG_RUNTIME_BOUNDARY_ALLOWED_FILES:
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern, message in EPG_RUNTIME_BOUNDARY_PATTERNS.items():
            if pattern in text:
                violations.append(f"{message}: {rel}")
    return violations


def non_epg_thread_violations(root: Path) -> list[str]:
    violations: list[str] = []
    for path in source_files(root, "src/native"):
        rel = relative_path(path, root)
        if rel in NON_EPG_THREAD_ALLOWED_FILES:
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern, message in NON_EPG_THREAD_PATTERNS.items():
            if pattern in text:
                violations.append(f"{message}: {rel}")
    return violations


def orb_slam3_location_violations(root: Path) -> list[str]:
    violations: list[str] = []
    if (root / ORB_SLAM3_THIRD_PARTY_DIR).exists():
        violations.append(
            "ORB-SLAM3 source must live under src/native/adapters/slam/orb/orb_slam3, not third_party/orb_slam3")
    if not (root / ORB_SLAM3_SOURCE_DIR / "CMakeLists.txt").exists():
        violations.append(
            "ORB-SLAM3 source must be present under src/native/adapters/slam/orb/orb_slam3")
    return violations


def orb_slam3_thread_violations(root: Path) -> list[str]:
    violations: list[str] = []
    orb_root = root / ORB_SLAM3_SOURCE_DIR
    if not orb_root.exists():
        return violations
    for path in source_files(root, ORB_SLAM3_SOURCE_DIR):
        rel = relative_path(path, root)
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern, message in ORB_SLAM3_THREAD_PATTERNS.items():
            if pattern in text:
                violations.append(f"{message}: {rel}")
    return violations


def orb_slam3_self_scheduling_violations(root: Path) -> list[str]:
    violations: list[str] = []
    orb_root = root / ORB_SLAM3_SOURCE_DIR
    if not orb_root.exists():
        return violations
    for path in source_files(root, ORB_SLAM3_SOURCE_DIR):
        rel = relative_path(path, root)
        if rel.startswith(f"{ORB_SLAM3_SOURCE_DIR}/visualization/"):
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern, message in ORB_SLAM3_SELF_SCHEDULING_PATTERNS.items():
            if pattern in text:
                violations.append(f"{message}: {rel}")
    return violations


def native_lock_violations(root: Path) -> list[str]:
    violations: list[str] = []
    for path in source_files(root, "src/native"):
        rel = relative_path(path, root)
        if is_orb_slam3_path(rel):
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern, message in NATIVE_LOCK_PATTERNS.items():
            index = text.find(pattern)
            if index >= 0:
                line = text.count("\n", 0, index) + 1
                violations.append(f"{message}: {rel}:{line}")
    return violations


def main() -> int:
    root = Path(sys.argv[1]).resolve()
    violations = []
    for rel in adapter_application_include_violations(root):
        violations.append(f"adapter must not include core/application directly: {rel}")
    for rel in core_ports_application_include_violations(root):
        violations.append(f"core port must not include core/application directly: {rel}")
    for rel in misplaced_application_composition(root):
        violations.append(f"application composition must live under src/native/app: {rel}")
    violations.extend(runtime_epg_boundary_violations(root))
    violations.extend(orb_slam3_location_violations(root))
    violations.extend(orb_slam3_thread_violations(root))
    violations.extend(orb_slam3_self_scheduling_violations(root))
    violations.extend(non_epg_thread_violations(root))
    violations.extend(native_lock_violations(root))
    if violations:
        sys.stderr.write("\n".join(violations) + "\n")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
