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
    "void Viewer::Run(": "ORB-SLAM3 visualization must not own an internal render loop",
}
ORB_SLAM3_LOCK_PATTERNS = {
    "#include <mutex>": "ORB-SLAM3 shared state must be serialized by EPG resources",
    "#include<mutex>": "ORB-SLAM3 shared state must be serialized by EPG resources",
    "#include <shared_mutex>": "ORB-SLAM3 shared state must be serialized by EPG resources",
    "#include <condition_variable>": "ORB-SLAM3 waits must be modeled by EPG scheduling",
    "std::mutex": "ORB-SLAM3 locks are not allowed",
    "mutex ": "ORB-SLAM3 locks are not allowed",
    "std::recursive_mutex": "ORB-SLAM3 locks are not allowed",
    "std::shared_mutex": "ORB-SLAM3 locks are not allowed",
    "std::timed_mutex": "ORB-SLAM3 locks are not allowed",
    "unique_lock": "ORB-SLAM3 locks are not allowed",
    "lock_guard": "ORB-SLAM3 locks are not allowed",
    "scoped_lock": "ORB-SLAM3 locks are not allowed",
    "shared_lock": "ORB-SLAM3 locks are not allowed",
    "condition_variable": "ORB-SLAM3 waits must be modeled by EPG scheduling",
    "std::once_flag": "ORB-SLAM3 once locks are not allowed",
    "std::call_once": "ORB-SLAM3 once locks are not allowed",
    "pthread_mutex": "ORB-SLAM3 locks are not allowed",
}

ORB_SLAM3_SYSTEM_BACKEND_BYPASS_PATTERNS = {
    "SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE":
        "ORB-SLAM3 tracking must not wait for LocalMapping inline",
    "WaitForLocalMappingIdle":
        "ORB-SLAM3 tracking must not wait for LocalMapping inline",
    "WaitForLocalMappingIdleIfRequested":
        "ORB-SLAM3 tracking must not wait for LocalMapping inline",
    "localMapper->Step(":
        "ORB-SLAM3 System must not advance LocalMapping outside StepBackend",
}

ORB_SLAM3_SYSTEM_BACKEND_ALLOWED_METHODS = {
    "System::StepBackend",
}

SLAM_SESSION_BACKEND_BYPASS_PATTERNS = {
    "StepBackendIfIdle":
        "SLAM backend cadence and serialization must be owned by EPG tasks",
    "m_backendBusy":
        "SLAM backend serialization must be owned by EPG resources",
}

SLAM_RUNTIME_CONTROL_BACKEND_FILES = {
    "src/native/core/ports/slam_runtime_control.h",
    "src/native/core/application/session/slam/slam_runtime_control_port.h",
    "src/native/core/application/session/slam/slam_runtime_control_port.cpp",
    "src/native/adapters/slam/engine/slam_engine_control.h",
}

SLAM_RUNTIME_CONTROL_BACKEND_PATTERNS = {
    "StepBackend(":
        "SLAM backend maintenance must stay out of generic runtime control",
}

RUNTIME_CONFIG_BYPASS_PATTERNS = {
    "SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE":
        "ORB-SLAM3 LocalMapping wait config has been replaced by EPG backend ticks",
    "SMART_DRONE_ORB_WAIT_LOCAL_MAPPING_IDLE_TIMEOUT_MS":
        "ORB-SLAM3 LocalMapping wait config has been replaced by EPG backend ticks",
    "--backend-step-every-n":
        "offline replay must not expose non-EPG backend cadence controls",
}

REPLAY_BACKEND_BYPASS_PATTERNS = {
    "StepBackend(": "offline replay backend maintenance must run through EPG",
    "RunBackendStepIfNeeded":
        "offline replay backend maintenance must run through EPG",
    "backendStepEveryN":
        "offline replay must not expose non-EPG backend cadence controls",
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
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern, message in ORB_SLAM3_SELF_SCHEDULING_PATTERNS.items():
            if pattern in text:
                violations.append(f"{message}: {rel}")
    return violations


def orb_slam3_lock_violations(root: Path) -> list[str]:
    violations: list[str] = []
    orb_root = root / ORB_SLAM3_SOURCE_DIR
    if not orb_root.exists():
        return violations
    for path in source_files(root, ORB_SLAM3_SOURCE_DIR):
        rel = relative_path(path, root)
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern, message in ORB_SLAM3_LOCK_PATTERNS.items():
            index = text.find(pattern)
            if index >= 0:
                line = text.count("\n", 0, index) + 1
                violations.append(f"{message}: {rel}:{line}")
    return violations


def orb_slam3_system_backend_bypass_violations(root: Path) -> list[str]:
    path = root / ORB_SLAM3_SOURCE_DIR / "system" / "System.cc"
    if not path.exists():
        return []
    rel = relative_path(path, root)
    text = path.read_text(encoding="utf-8", errors="ignore")
    violations: list[str] = []
    for pattern, message in ORB_SLAM3_SYSTEM_BACKEND_BYPASS_PATTERNS.items():
        index = text.find(pattern)
        if index >= 0:
            line = text.count("\n", 0, index) + 1
            violations.append(f"{message}: {rel}:{line}")
    violations.extend(orb_slam3_system_step_backend_call_violations(text, rel))
    return violations


def containing_function_name(text: str, index: int) -> str:
    prefix = text[:index]
    signature_start = prefix.rfind("\n")
    while signature_start > 0:
        candidate_start = prefix.rfind("\n", 0, signature_start)
        line = prefix[candidate_start + 1:signature_start].strip()
        if line and line.endswith("{") and "(" in line and ")" in line:
            return line[:-1].strip()
        signature_start = candidate_start
    return ""


def orb_slam3_system_step_backend_call_violations(
        text: str, rel: str) -> list[str]:
    violations: list[str] = []
    search_from = 0
    while True:
        index = text.find("StepBackend();", search_from)
        if index < 0:
            return violations
        function_name = containing_function_name(text, index)
        if function_name not in ORB_SLAM3_SYSTEM_BACKEND_ALLOWED_METHODS:
            line = text.count("\n", 0, index) + 1
            violations.append(
                f"ORB-SLAM3 backend stepping must be called by EPG only: {rel}:{line}")
        search_from = index + 1


def slam_session_backend_bypass_violations(root: Path) -> list[str]:
    violations: list[str] = []
    base = root / "src/native/core/application/session/slam"
    if not base.exists():
        return violations
    for path in source_files(root, "src/native/core/application/session/slam"):
        rel = relative_path(path, root)
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern, message in SLAM_SESSION_BACKEND_BYPASS_PATTERNS.items():
            index = text.find(pattern)
            if index >= 0:
                line = text.count("\n", 0, index) + 1
                violations.append(f"{message}: {rel}:{line}")
    return violations


def slam_runtime_control_backend_violations(root: Path) -> list[str]:
    violations: list[str] = []
    for rel in sorted(SLAM_RUNTIME_CONTROL_BACKEND_FILES):
        path = root / rel
        if not path.exists():
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern, message in SLAM_RUNTIME_CONTROL_BACKEND_PATTERNS.items():
            index = text.find(pattern)
            if index >= 0:
                line = text.count("\n", 0, index) + 1
                violations.append(f"{message}: {rel}:{line}")
    return violations


def runtime_config_backend_bypass_violations(root: Path) -> list[str]:
    violations: list[str] = []
    candidates = [
        root / "scripts",
        root / "smart_drone.service",
        root / "tests" / "euroc",
    ]
    for candidate in candidates:
        paths = [candidate] if candidate.is_file() else candidate.rglob("*")
        for path in paths:
            if not path.is_file() or path.suffix not in {
                ".cpp", ".h", ".md", ".py", ".service", ".sh"}:
                continue
            rel = relative_path(path, root)
            if rel == "tests/architecture/test_native_boundaries.py":
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            for pattern, message in RUNTIME_CONFIG_BYPASS_PATTERNS.items():
                index = text.find(pattern)
                if index >= 0:
                    line = text.count("\n", 0, index) + 1
                    violations.append(f"{message}: {rel}:{line}")
    return violations


def replay_backend_bypass_violations(root: Path) -> list[str]:
    violations: list[str] = []
    replay_dir = root / "tests" / "euroc" / "support"
    if not replay_dir.exists():
        return violations
    for path in replay_dir.rglob("*"):
        if not path.is_file() or path.suffix not in {".cpp", ".h"}:
            continue
        rel = relative_path(path, root)
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern, message in REPLAY_BACKEND_BYPASS_PATTERNS.items():
            index = text.find(pattern)
            if index >= 0:
                line = text.count("\n", 0, index) + 1
                violations.append(f"{message}: {rel}:{line}")
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
    violations.extend(orb_slam3_lock_violations(root))
    violations.extend(orb_slam3_system_backend_bypass_violations(root))
    violations.extend(slam_session_backend_bypass_violations(root))
    violations.extend(slam_runtime_control_backend_violations(root))
    violations.extend(runtime_config_backend_bypass_violations(root))
    violations.extend(replay_backend_bypass_violations(root))
    violations.extend(non_epg_thread_violations(root))
    violations.extend(native_lock_violations(root))
    if violations:
        sys.stderr.write("\n".join(violations) + "\n")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
