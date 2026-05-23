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
    "std::thread": "native runtime threads must be owned by EPG",
    "std::jthread": "native runtime threads must be owned by EPG",
    "std::async": "native runtime async work must be owned by EPG",
    "pthread_create": "native runtime threads must be owned by EPG",
    ".detach(": "native runtime detached threads are not allowed",
}


def relative_path(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def source_files(root: Path, directory: str) -> list[Path]:
    base = root / directory
    return [
        path
        for path in base.rglob("*")
        if path.suffix in {".h", ".hpp", ".c", ".cc", ".cpp"}
        and "adapters/slam/orb/orb_slam3" not in path.as_posix()
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
    violations.extend(non_epg_thread_violations(root))
    if violations:
        sys.stderr.write("\n".join(violations) + "\n")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
