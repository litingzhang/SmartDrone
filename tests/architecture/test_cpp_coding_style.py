#!/usr/bin/env python3
"""High-confidence C++ coding-style checks."""

from __future__ import annotations

import re
import sys
from pathlib import Path


SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hpp", ".inc"}
HEADER_SUFFIXES = {".h", ".hpp"}
SOURCE_LINE_LIMIT = 1000
HEADER_LINE_LIMIT = 500

LOWERCASE_NAMESPACE_PATTERN = re.compile(
    r"(?:namespace\s+SmartDrone::[a-z]|SmartDrone::[a-z])")
RELATIVE_NAMESPACE_PATTERN = re.compile(
    r"\b(core|adapters|app|common|tests)::"
    r"(application|ports|domain|camera|command|imu|slam|telemetry|stream|"
    r"bootstrap|composition)\b")
SHORT_RELATIVE_NAMESPACE_PATTERN = re.compile(
    r"\b(ports|domain|application|camera|command|imu|slam|telemetry|stream|"
    r"bootstrap|composition)::")


def RelativePath(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def IsExcluded(path: Path) -> bool:
    text = path.as_posix()
    return any(
        marker in text
        for marker in (
            "/third_party/",
            "/src/android/app/.cxx/",
            "/src/native/adapters/slam/orb_slam3/",
            "/output/",
        )
    )


def SourceFiles(root: Path) -> list[Path]:
    paths: list[Path] = []
    for directory in ("src/native", "src/android/app/src/main/cpp", "tests"):
        base = root / directory
        if not base.exists():
            continue
        paths.extend(
            path for path in base.rglob("*")
            if path.suffix in SOURCE_SUFFIXES and not IsExcluded(path))
    return sorted(paths)


def LineLimit(path: Path) -> int:
    return HEADER_LINE_LIMIT if path.suffix in HEADER_SUFFIXES else SOURCE_LINE_LIMIT


def FileLengthViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        line_count = len(path.read_text(encoding="utf-8",
                                        errors="ignore").splitlines())
        limit = LineLimit(path)
        if line_count > limit:
            violations.append(
                f"{RelativePath(path, root)} has {line_count} lines, limit is {limit}")
    return violations


def NamespaceViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        text = path.read_text(encoding="utf-8", errors="ignore")
        for match in LOWERCASE_NAMESPACE_PATTERN.finditer(text):
            line = text.count("\n", 0, match.start()) + 1
            violations.append(
                f"{RelativePath(path, root)}:{line} uses lowercase SmartDrone namespace")
        for match in RELATIVE_NAMESPACE_PATTERN.finditer(text):
            line = text.count("\n", 0, match.start()) + 1
            violations.append(
                f"{RelativePath(path, root)}:{line} uses lowercase relative namespace")
        for match in SHORT_RELATIVE_NAMESPACE_PATTERN.finditer(text):
            line = text.count("\n", 0, match.start()) + 1
            violations.append(
                f"{RelativePath(path, root)}:{line} uses short lowercase namespace")
    return violations


def main() -> int:
    root = Path(sys.argv[1]).resolve()
    paths = SourceFiles(root)
    violations = FileLengthViolations(root, paths)
    violations.extend(NamespaceViolations(root, paths))
    if violations:
        sys.stderr.write("\n".join(violations) + "\n")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
