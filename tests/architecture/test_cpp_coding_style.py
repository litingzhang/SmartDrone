#!/usr/bin/env python3
"""High-confidence C++ coding-style checks."""

from __future__ import annotations

import re
import sys
from bisect import bisect_right
from pathlib import Path


SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hpp"}
HEADER_SUFFIXES = {".h", ".hpp"}
SOURCE_LINE_LIMIT = 1000
HEADER_LINE_LIMIT = 500
FUNCTION_LINE_LIMIT = 50
HEADER_INLINE_BODY_LINE_LIMIT = 10
DISALLOWED_FILE_PATTERNS = (
    re.compile(r"\.inc$"),
    re.compile(r"_(members|private|public_methods|private_methods)\.(h|hpp|cpp|cc|cxx)$"),
)

LOWERCASE_NAMESPACE_PATTERN = re.compile(
    r"(?:namespace\s+SmartDrone::[a-z]|SmartDrone::[a-z])")
RELATIVE_NAMESPACE_PATTERN = re.compile(
    r"\b(core|adapters|app|common|tests)::"
    r"(application|ports|domain|camera|command|imu|slam|telemetry|stream|"
    r"bootstrap|composition)\b")
SHORT_RELATIVE_NAMESPACE_PATTERN = re.compile(
    r"\b(ports|domain|application|camera|command|imu|slam|telemetry|stream|"
    r"bootstrap|composition)::")
PREPROCESSOR_DIRECTIVE_PATTERN = re.compile(
    r"^\s*#\s*(define|ifdef|ifndef|if\s+defined|if\s+!defined|elif|endif)\b",
    re.MULTILINE)
CMAKE_COMPILE_DEFINITION_PATTERN = re.compile(
    r"\b(target_compile_definitions|add_compile_definitions|add_definitions|"
    r"COMPILE_DEFINITIONS)\b")
BARE_PRINTF_PATTERN = re.compile(r"(?<![A-Za-z0-9_:])printf\s*\(")
STD_COUT_PATTERN = re.compile(r"\bstd::cout\b")
STDERR_WRITE_PATTERN = re.compile(
    r"\b(?:std::)?(?:fprintf|fputs|fwrite)\s*\(\s*stderr\b")
CONTROL_STATEMENT_PATTERN = re.compile(r"^\s*(if|else if|else|for|while|switch|catch)\b")
FUNCTION_SIGNATURE_PATTERN = re.compile(
    r"(?m)(?:^|\n)\s*"
    r"(?!if\b|for\b|while\b|switch\b|catch\b|return\b|sizeof\b|static_assert\b)"
    r"(?:[A-Za-z_~][\w:<>~*&\s,]+\s+)?"
    r"([A-Za-z_~][\w:<>~]*)\s*"
    r"\(([^;{}()]*(?:\([^)]*\)[^;{}()]*)*)\)\s*"
    r"(?:const\s*)?(?:noexcept\s*)?(?:override\s*)?(?:final\s*)?"
    r"(?:=\s*0\s*)?[;{]")
SINGLE_LINE_FUNCTION_BRACE_PATTERN = re.compile(
    r"^\s*(?!if\b|else\b|for\b|while\b|switch\b|catch\b|return\b)"
    r"(?!.*\[[^\]]*\]\s*\()"
    r"(?:template\s*<[^>{;]+>\s*)?"
    r"(?:[A-Za-z_~][A-Za-z0-9_:<>~*&,\s]+\s+)?"
    r"[A-Za-z_~][A-Za-z0-9_:<>~]*\s*\([^;{}]*\)"
    r"\s*(?:const\s*)?(?:noexcept\s*)?(?:override\s*)?(?:final\s*)?\{")
TYPE_DECLARATION_PATTERNS = (
    re.compile(
        r"(?m)^\s*(?:class|struct)\s+([A-Za-z_][A-Za-z0-9_]*)\b"
        r"\s*(?:final\s*)?(?:[:{;]|$)"),
    re.compile(
        r"(?m)^\s*enum\s+(?:class\s+)?([A-Za-z_][A-Za-z0-9_]*)\b"
        r"\s*(?::[^\n{;]+)?[{;]"),
)
TYPE_ALIAS_PATTERN = re.compile(r"(?m)^\s*using\s+([A-Za-z_][A-Za-z0-9_]*)\s*=")
TYPEDEF_PATTERN = re.compile(r"(?m)^\s*typedef\b([^;]+);")
CLASS_DEFINITION_PATTERN = re.compile(
    r"(?m)^\s*class\s+([A-Za-z_][A-Za-z0-9_]*)\b[^;{}]*\{")
STRUCT_DEFINITION_PATTERN = re.compile(
    r"(?m)^\s*struct\s+([A-Za-z_][A-Za-z0-9_]*)\b[^;{}]*\{")
COMMENT_RESIDUE_PATTERN = re.compile(r"\b(TODO|FIXME|HACK|XXX)\b", re.IGNORECASE)
GLOBAL_DECLARATION_START_PATTERN = re.compile(
    r"^(?:(?:static|inline|extern)\s+)*"
    r"(?:(?:constexpr|const|volatile)\s+)*"
    r"(?:(?:std::)?[A-Za-z_][A-Za-z0-9_:]*|"
    r"[A-Za-z_][A-Za-z0-9_:<>]*)")
GLOBAL_DECLARATION_SKIP_PATTERN = re.compile(
    r"^(?:using|typedef|friend|class|struct|enum|namespace|template|"
    r"static_assert|return|if|else|for|while|switch|catch|case|break|"
    r"continue|do|try)\b")

ALLOWED_PREPROCESSOR_PREFIXES = ()

ALLOWED_PREPROCESSOR_DIRECTIVE_PATHS = {
    "src/native/common/environment.cpp",
}

ALLOWED_LONG_PARAMETER_FUNCTION_PREFIXES = (
    "cv::",
    "std::",
    "Eigen::",
    "mavlink_",
)

ALLOWED_EXTERNAL_TYPE_PREFIXES = (
    "v4l2_",
    "gpiod_",
)


def RelativePath(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def IsExcluded(path: Path) -> bool:
    text = path.as_posix()
    return any(
        marker in text
        for marker in (
            "/third_party/",
            "/src/native/adapters/slam/orb/orb_slam3/",
            "/src/native/adapters/slam/openvins/open_vins/",
            "/src/android/app/.cxx/",
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


def RepoFiles(root: Path) -> list[Path]:
    paths: list[Path] = []
    for directory in ("src/native", "src/android/app/src/main/cpp", "tests"):
        base = root / directory
        if not base.exists():
            continue
        paths.extend(path for path in base.rglob("*") if path.is_file() and not IsExcluded(path))
    return sorted(paths)


def CMakeFiles(root: Path) -> list[Path]:
    return sorted(
        path for path in root.rglob("CMakeLists.txt")
        if path.is_file() and not IsExcluded(path))


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


def DisallowedFileNameViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        rel = RelativePath(path, root)
        name = path.name
        if any(pattern.search(name) for pattern in DISALLOWED_FILE_PATTERNS):
            violations.append(f"{rel} uses a disallowed implementation-fragment filename")
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


def PreprocessorDirectiveViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        rel = RelativePath(path, root)
        if rel in ALLOWED_PREPROCESSOR_DIRECTIVE_PATHS:
            continue
        if any(rel.startswith(prefix) for prefix in ALLOWED_PREPROCESSOR_PREFIXES):
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for match in PREPROCESSOR_DIRECTIVE_PATTERN.finditer(text):
            line = text.count("\n", 0, match.start()) + 1
            violations.append(
                f"{rel}:{line} uses preprocessor directive outside allowed platform boundary")
    return violations


def CMakeCompileDefinitionViolations(root: Path) -> list[str]:
    violations: list[str] = []
    for path in CMakeFiles(root):
        rel = RelativePath(path, root)
        text = path.read_text(encoding="utf-8", errors="ignore")
        for match in CMAKE_COMPILE_DEFINITION_PATTERN.finditer(text):
            line = text.count("\n", 0, match.start()) + 1
            violations.append(
                f"{rel}:{line} injects compile definitions; avoid custom compile macros")
    return violations


def BarePrintfViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        rel = RelativePath(path, root)
        text = path.read_text(encoding="utf-8", errors="ignore")
        for match in BARE_PRINTF_PATTERN.finditer(text):
            line = text.count("\n", 0, match.start()) + 1
            violations.append(f"{rel}:{line} uses bare printf; use C++ streams or a logging adapter")
    return violations


def NativeStdoutViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        rel = RelativePath(path, root)
        if not rel.startswith("src/native/"):
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for match in STD_COUT_PATTERN.finditer(text):
            line = text.count("\n", 0, match.start()) + 1
            violations.append(f"{rel}:{line} writes native runtime diagnostics to stdout")
    return violations


def NativeStderrWriteViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        rel = RelativePath(path, root)
        if not rel.startswith("src/native/"):
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for match in STDERR_WRITE_PATTERN.finditer(text):
            line = text.count("\n", 0, match.start()) + 1
            violations.append(f"{rel}:{line} writes stderr through C stdio; use C++ streams or a logging adapter")
    return violations


def FindMatchingParen(text: str, start: int) -> int:
    depth = 0
    index = start
    while index < len(text):
        char = text[index]
        if char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
            if depth == 0:
                return index
        index += 1
    return -1


def HasOpenParenBeforeLine(lines: list[str], lineIndex: int) -> bool:
    depth = 0
    for index in range(lineIndex):
        for char in lines[index]:
            if char == "(":
                depth += 1
            elif char == ")" and depth > 0:
                depth -= 1
    return depth > 0


def IsFunctionPointerMember(line: str) -> bool:
    return "(*" in line and "){" in line


def BlankMatchPreservingNewlines(match: re.Match[str]) -> str:
    return "".join("\n" if char == "\n" else " " for char in match.group(0))


def StripRawStrings(text: str, replacement: str | None) -> str:
    def Replace(match: re.Match[str]) -> str:
        if replacement is None:
            return BlankMatchPreservingNewlines(match)
        return replacement

    return re.sub(r'R"([^\s()\\]{0,16})\(.*?\)\1"', Replace, text, flags=re.S)


def StripCommentsAndStrings(text: str) -> str:
    text = StripRawStrings(text, '""')
    text = re.sub(r"'(?:\\.|[^'\\\n])+'", "''", text)
    text = re.sub(r'"(?:\\.|[^"\\])*"', '""', text)
    text = re.sub(r"//.*", "", text)
    return re.sub(r"/\*.*?\*/", "", text, flags=re.S)


def StripCommentsAndStringsPreservingLines(text: str) -> str:
    text = StripRawStrings(text, None)
    text = re.sub(r"'(?:\\.|[^'\\\n])+'", BlankMatchPreservingNewlines, text)
    text = re.sub(r'"(?:\\.|[^"\\])*"', BlankMatchPreservingNewlines, text)
    text = re.sub(r"//.*", BlankMatchPreservingNewlines, text)
    return re.sub(r"/\*.*?\*/", BlankMatchPreservingNewlines, text, flags=re.S)


def LineOffsets(lines: list[str]) -> list[int]:
    offsets: list[int] = []
    position = 0
    for line in lines:
        offsets.append(position)
        position += len(line) + 1
    return offsets


def SplitParameters(parameterText: str) -> list[str]:
    parameters: list[str] = []
    current: list[str] = []
    depth = 0
    angleDepth = 0
    for char in parameterText:
        if char in "([{":
            depth += 1
        elif char in ")]}":
            depth = max(0, depth - 1)
        elif char == "<":
            angleDepth += 1
        elif char == ">":
            angleDepth = max(0, angleDepth - 1)

        if char == "," and depth == 0 and angleDepth == 0:
            parameters.append("".join(current).strip())
            current = []
        else:
            current.append(char)
    text = "".join(current).strip()
    if text or parameterText.strip():
        parameters.append(text)
    if len(parameters) == 1 and parameters[0] in ("", "void"):
        return []
    return parameters


def IsNativePath(root: Path, path: Path) -> bool:
    return RelativePath(path, root).startswith("src/native/")


def IsBigCamelName(name: str) -> bool:
    return bool(re.match(r"^[A-Z][A-Za-z0-9]*$", name))


def IsUpperSnakeName(name: str) -> bool:
    return bool(re.match(r"^[A-Z][A-Z0-9_]*$", name))


def IsAllowedExternalTypeName(name: str) -> bool:
    return name.startswith(ALLOWED_EXTERNAL_TYPE_PREFIXES)


def TypeNameViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        if not IsNativePath(root, path):
            continue
        rel = RelativePath(path, root)
        text = StripCommentsAndStringsPreservingLines(
            path.read_text(encoding="utf-8", errors="ignore"))
        for pattern in TYPE_DECLARATION_PATTERNS:
            for match in pattern.finditer(text):
                name = match.group(1)
                if IsBigCamelName(name) or IsAllowedExternalTypeName(name):
                    continue
                line = text.count("\n", 0, match.start()) + 1
                violations.append(
                    f"{rel}:{line} declares type {name}; use BigCamel type names")
        for match in TYPE_ALIAS_PATTERN.finditer(text):
            name = match.group(1)
            if IsBigCamelName(name):
                continue
            line = text.count("\n", 0, match.start()) + 1
            violations.append(
                f"{rel}:{line} declares type alias {name}; use BigCamel type names")
        for match in TYPEDEF_PATTERN.finditer(text):
            name = TypedefAliasName(match.group(1))
            if not name or IsBigCamelName(name):
                continue
            line = text.count("\n", 0, match.start()) + 1
            violations.append(
                f"{rel}:{line} declares typedef {name}; use BigCamel type names")
    return violations


def TypedefAliasName(typedefText: str) -> str:
    pointer_match = re.search(r"\(\s*\*\s*([A-Za-z_][A-Za-z0-9_]*)\s*\)",
                              typedefText)
    if pointer_match:
        return pointer_match.group(1)
    tokens = re.findall(r"[A-Za-z_][A-Za-z0-9_]*", typedefText)
    return tokens[-1] if tokens else ""


def ClassDefinitions(text: str) -> list[tuple[str, int, int]]:
    return TypeDefinitions(text, CLASS_DEFINITION_PATTERN)


def StructDefinitions(text: str) -> list[tuple[str, int, int]]:
    return TypeDefinitions(text, STRUCT_DEFINITION_PATTERN)


def TypeDefinitions(text: str,
                    pattern: re.Pattern[str]) -> list[tuple[str, int, int]]:
    definitions: list[tuple[str, int, int]] = []
    braceMatches = BraceMatches(text)
    for match in pattern.finditer(text):
        openBrace = text.find("{", match.start(), match.end())
        closeBrace = braceMatches.get(openBrace)
        if closeBrace is None:
            continue
        definitions.append((match.group(1), openBrace, closeBrace))
    return definitions


def TopLevelStatements(body: str) -> list[tuple[int, str]]:
    statements: list[tuple[int, str]] = []
    start = 0
    depth = 0
    for index, char in enumerate(body):
        if char in "([{":
            depth += 1
        elif char in ")]}":
            depth = max(0, depth - 1)
        if char == ";" and depth == 0:
            statements.append((start, body[start:index + 1]))
            start = index + 1
    return statements


def ClassMemberName(statement: str) -> str:
    compact = " ".join(statement.split())
    if not compact or compact.endswith(":"):
        return ""
    if re.match(r"^(public|private|protected)\s*:", compact):
        return ""
    if re.match(
            r"^(using|typedef|friend|static_assert|class|struct|enum|template)\b",
            compact):
        return ""
    if "(" in compact or ")" in compact or "," in compact:
        return ""
    compact = re.sub(r"^\s*\[\[[^\]]+\]\]\s*", "", compact)
    compact = re.sub(r"\s*=.*;$", ";", compact)
    compact = re.sub(r"\s*\{[^{}]*\}\s*;$", ";", compact)
    compact = re.sub(r"\[[^\]]*\]", "", compact)
    match = re.search(r"\b([A-Za-z_][A-Za-z0-9_]*)\s*;$", compact)
    return match.group(1) if match else ""


def IsStructMemberName(statement: str, name: str) -> bool:
    compact = " ".join(statement.split())
    if re.search(r"\b(static|constexpr)\b", compact):
        return True
    return bool(re.match(r"^[a-z][A-Za-z0-9]*$", name))


def ClassMemberNameViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        if not IsNativePath(root, path):
            continue
        rel = RelativePath(path, root)
        text = StripCommentsAndStringsPreservingLines(
            path.read_text(encoding="utf-8", errors="ignore"))
        lines = text.splitlines()
        offsets = LineOffsets(lines)
        for className, openBrace, closeBrace in ClassDefinitions(text):
            body = text[openBrace + 1:closeBrace]
            for statementStart, statement in TopLevelStatements(body):
                name = ClassMemberName(statement)
                if not name or name.startswith("m_") or IsUpperSnakeName(name):
                    continue
                line = bisect_right(offsets, openBrace + 1 + statementStart)
                violations.append(
                    f"{rel}:{line} class {className} member {name}; use m_ lowerCamel member names")
    return violations


def StructMemberNameViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        if not IsNativePath(root, path):
            continue
        rel = RelativePath(path, root)
        text = StripCommentsAndStringsPreservingLines(
            path.read_text(encoding="utf-8", errors="ignore"))
        lines = text.splitlines()
        offsets = LineOffsets(lines)
        for structName, openBrace, closeBrace in StructDefinitions(text):
            body = text[openBrace + 1:closeBrace]
            for statementStart, statement in TopLevelStatements(body):
                name = ClassMemberName(statement)
                if not name or IsStructMemberName(statement, name):
                    continue
                line = bisect_right(offsets, openBrace + 1 + statementStart)
                violations.append(
                    f"{rel}:{line} struct {structName} member {name}; use lowerCamel member names")
    return violations


def BlankSourceRange(chars: list[str], start: int, end: int) -> None:
    for index in range(max(0, start), min(len(chars), end)):
        if chars[index] != "\n":
            chars[index] = " "


def GlobalScopeText(text: str) -> str:
    text = StripCommentsAndStringsPreservingLines(text)
    chars = list(text)
    lines = text.splitlines()
    offsets = LineOffsets(lines)
    for openBrace, closeBrace in sorted(BraceMatches(text).items()):
        openLine = bisect_right(offsets, openBrace) - 1
        if openLine < 0:
            continue
        startLine = HeaderStartLine(lines, openLine)
        header = text[offsets[startLine]:openBrace]
        compact = " ".join(header.split())
        if re.match(r"^namespace\b", compact):
            BlankSourceRange(chars, offsets[startLine], openBrace + 1)
            BlankSourceRange(chars, closeBrace, closeBrace + 1)
        elif IsIgnoredGlobalScopeBlock(header):
            BlankSourceRange(chars, offsets[startLine], closeBrace + 1)
    return "".join(chars)


def IsIgnoredGlobalScopeBlock(header: str) -> bool:
    compact = " ".join(header.split())
    if not compact:
        return False
    if re.match(r"^(?:class|struct|enum|union)\b", compact):
        return True
    if re.match(
            r"^(?:if|else|for|while|switch|catch|do|try|return|throw|"
            r"static_assert|sizeof|alignof)\b", compact):
        return False
    if re.match(r"^namespace\b", compact):
        return False
    return "(" in compact and ")" in compact


def TopLevelSourceStatements(text: str) -> list[tuple[int, str]]:
    statements: list[tuple[int, str]] = []
    start = 0
    depth = 0
    for index, char in enumerate(text):
        if char in "([{":
            depth += 1
        elif char in ")]}":
            depth = max(0, depth - 1)
        elif char == ";" and depth == 0:
            statements.append((start, text[start:index + 1]))
            start = index + 1
    return statements


def GlobalDeclarationHead(statement: str) -> str:
    depth = 0
    for index, char in enumerate(statement):
        if char in "([{":
            if char == "{" and depth == 0:
                return statement[:index]
            depth += 1
        elif char in ")]}":
            depth = max(0, depth - 1)
        elif char == "=" and depth == 0:
            previous = statement[index - 1] if index > 0 else ""
            nextChar = statement[index + 1] if index + 1 < len(statement) else ""
            if previous not in "=!<>" and nextChar != "=":
                return statement[:index]
        elif char == ";" and depth == 0:
            return statement[:index]
    return statement


def GlobalDeclarationName(statement: str) -> tuple[str, bool]:
    compact = " ".join(statement.split())
    if not compact or GLOBAL_DECLARATION_SKIP_PATTERN.match(compact):
        return "", False
    head = GlobalDeclarationHead(compact).strip()
    if not head or GLOBAL_DECLARATION_SKIP_PATTERN.match(head):
        return "", False
    if "(" in head or ")" in head or "," in head:
        return "", False
    if not GLOBAL_DECLARATION_START_PATTERN.match(head):
        return "", False
    tokens = re.findall(r"[A-Za-z_][A-Za-z0-9_]*", head)
    if len(tokens) < 2:
        return "", False
    name = tokens[-1]
    if name in ("const", "volatile", "constexpr"):
        return "", False
    if "::" + name in head or re.search(r"\boperator\b", head):
        return "", False
    prefix = head[:head.rfind(name)]
    isConst = bool(
        re.match(r"^(?:(?:static|inline|extern)\s+)*constexpr\b", head))
    isConst = isConst or bool(
        re.match(r"^(?:(?:static|inline|extern)\s+)*const\b", head))
    isConst = isConst or bool(re.search(r"\bconst\s*(?:\*|&|&&)?\s*$", prefix))
    return name, isConst


def GlobalNameViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        if not IsNativePath(root, path):
            continue
        rel = RelativePath(path, root)
        text = GlobalScopeText(path.read_text(encoding="utf-8", errors="ignore"))
        offsets = LineOffsets(text.splitlines())
        for statementStart, statement in TopLevelSourceStatements(text):
            name, isConst = GlobalDeclarationName(statement)
            if not name:
                continue
            if isConst and IsUpperSnakeName(name):
                continue
            if not isConst and name.startswith("g_"):
                continue
            line = bisect_right(offsets, statementStart)
            if isConst:
                violations.append(
                    f"{rel}:{line} global constant {name}; use UPPER_SNAKE global constant names")
            else:
                violations.append(
                    f"{rel}:{line} global variable {name}; use g_ lowerCamel global variable names")
    return violations


def LongParameterFunctionViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        rel = RelativePath(path, root)
        text = StripCommentsAndStrings(
            path.read_text(encoding="utf-8", errors="ignore"))
        for match in FUNCTION_SIGNATURE_PATTERN.finditer(text):
            name = match.group(1)
            if name.startswith(ALLOWED_LONG_PARAMETER_FUNCTION_PREFIXES):
                continue
            parameters = SplitParameters(match.group(2))
            if len(parameters) <= 5:
                continue
            line = text.count("\n", 0, match.start()) + 1
            violations.append(
                f"{rel}:{line} function {name} has {len(parameters)} parameters; use a parameter object")
    return violations


def HeaderStartLine(lines: list[str], openLine: int) -> int:
    startLine = openLine
    while startLine > 0 and openLine - startLine < 20:
        previous = lines[startLine - 1].strip()
        if not previous or previous.startswith("#"):
            break
        if previous.endswith(";") or previous.endswith("{") or previous.endswith("}"):
            break
        startLine -= 1
    return startLine


def IsFunctionHeader(header: str) -> bool:
    compact = " ".join(header.split())
    if "(" not in compact or ")" not in compact:
        return False
    if re.match(r"^(if|else|for|while|switch|catch|do|try)\b", compact):
        return False
    if re.match(r"^(namespace|class|struct|enum|union)\b", compact):
        return False
    if re.match(r"^(return|throw|static_assert|sizeof|alignof)\b", compact):
        return False
    if re.search(r"\[[^\]]*\]\s*\(", compact):
        return False
    return True


def FunctionNameFromHeader(header: str) -> str:
    compact = " ".join(header.split())
    paren = compact.rfind("(")
    if paren < 0:
        return "<unknown>"
    prefix = compact[:paren].rstrip()
    if not prefix:
        return "<unknown>"
    return prefix.split()[-1].strip("*&")


def BraceMatches(text: str) -> dict[int, int]:
    matches: dict[int, int] = {}
    stack: list[int] = []
    for index, char in enumerate(text):
        if char == "{":
            stack.append(index)
        elif char == "}" and stack:
            matches[stack.pop()] = index
    return matches


def FunctionLineCountViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        rel = RelativePath(path, root)
        text = StripCommentsAndStringsPreservingLines(
            path.read_text(encoding="utf-8", errors="ignore"))
        lines = text.splitlines()
        offsets = LineOffsets(lines)
        for openBrace, closeBrace in BraceMatches(text).items():
            openLine = bisect_right(offsets, openBrace) - 1
            closeLine = bisect_right(offsets, closeBrace) - 1
            if openLine < 0 or closeLine < 0:
                continue
            startLine = HeaderStartLine(lines, openLine)
            header = text[offsets[startLine]:openBrace]
            if not IsFunctionHeader(header):
                continue
            lineCount = sum(
                1 for line in lines[startLine:closeLine + 1] if line.strip())
            if lineCount >= FUNCTION_LINE_LIMIT:
                violations.append(
                    f"{rel}:{startLine + 1} function {FunctionNameFromHeader(header)} has {lineCount} lines; limit is less than {FUNCTION_LINE_LIMIT}")
    return violations


def HeaderInlineFunctionViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        rel = RelativePath(path, root)
        if not rel.startswith("src/native/") or path.suffix not in HEADER_SUFFIXES:
            continue
        text = StripCommentsAndStringsPreservingLines(
            path.read_text(encoding="utf-8", errors="ignore"))
        lines = text.splitlines()
        offsets = LineOffsets(lines)
        for openBrace, closeBrace in BraceMatches(text).items():
            openLine = bisect_right(offsets, openBrace) - 1
            closeLine = bisect_right(offsets, closeBrace) - 1
            if openLine < 0 or closeLine < 0:
                continue
            startLine = HeaderStartLine(lines, openLine)
            header = text[offsets[startLine]:openBrace]
            if not IsFunctionHeader(header):
                continue
            bodyLines = sum(
                1 for line in lines[openLine + 1:closeLine] if line.strip())
            if bodyLines > HEADER_INLINE_BODY_LINE_LIMIT:
                violations.append(
                    f"{rel}:{openLine + 1} header inline function {FunctionNameFromHeader(header)} has {bodyLines} body lines; limit is {HEADER_INLINE_BODY_LINE_LIMIT}")
    return violations


def ControlBraceViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        rel = RelativePath(path, root)
        text = path.read_text(encoding="utf-8", errors="ignore")
        lines = text.splitlines()
        offsets: list[int] = []
        position = 0
        for line in lines:
            offsets.append(position)
            position += len(line) + 1
        for lineIndex, line in enumerate(lines):
            if not CONTROL_STATEMENT_PATTERN.match(line):
                continue
            openParen = line.find("(")
            endIndex = offsets[lineIndex] + len(line)
            if openParen >= 0:
                closeParen = FindMatchingParen(text, offsets[lineIndex] + openParen)
                if closeParen < 0:
                    continue
                endIndex = closeParen
            suffix = text[endIndex:text.find("\n", endIndex)
                          if "\n" in text[endIndex:] else len(text)]
            if "{" in suffix:
                continue
            nextLine = lineIndex + 1
            if nextLine < len(lines) and lines[nextLine].strip().startswith("{"):
                violations.append(
                    f"{rel}:{lineIndex + 1} puts control-statement brace on the next line")
    return violations


def FunctionBraceViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        rel = RelativePath(path, root)
        lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
        for lineIndex, line in enumerate(lines):
            if HasOpenParenBeforeLine(lines, lineIndex) or IsFunctionPointerMember(line):
                continue
            if SINGLE_LINE_FUNCTION_BRACE_PATTERN.match(line):
                violations.append(
                    f"{rel}:{lineIndex + 1} puts function brace on the signature line")
    return violations


def CommentSegments(line: str, inBlockComment: bool) -> tuple[list[str], bool]:
    segments: list[str] = []
    while line:
        if inBlockComment:
            end = line.find("*/")
            if end < 0:
                segments.append(line)
                return segments, True
            segments.append(line[:end])
            line = line[end + 2:]
            inBlockComment = False
            continue

        line_comment = line.find("//")
        block_comment = line.find("/*")
        if line_comment < 0 and block_comment < 0:
            return segments, False
        if line_comment >= 0 and (block_comment < 0 or line_comment < block_comment):
            segments.append(line[line_comment + 2:])
            return segments, False
        end = line.find("*/", block_comment + 2)
        if end < 0:
            segments.append(line[block_comment + 2:])
            return segments, True
        segments.append(line[block_comment + 2:end])
        line = line[end + 2:]
    return segments, inBlockComment


def CommentResidueViolations(root: Path, paths: list[Path]) -> list[str]:
    violations: list[str] = []
    for path in paths:
        if not IsNativePath(root, path):
            continue
        rel = RelativePath(path, root)
        in_block_comment = False
        for line_index, line in enumerate(
                path.read_text(encoding="utf-8", errors="ignore").splitlines()):
            segments, in_block_comment = CommentSegments(line, in_block_comment)
            if COMMENT_RESIDUE_PATTERN.search(" ".join(segments)):
                violations.append(
                    f"{rel}:{line_index + 1} leaves TODO/FIXME/HACK/XXX comment residue")
    return violations


def main() -> int:
    root = Path(sys.argv[1]).resolve()
    paths = SourceFiles(root)
    violations = DisallowedFileNameViolations(root, RepoFiles(root))
    violations.extend(FileLengthViolations(root, paths))
    violations.extend(NamespaceViolations(root, paths))
    violations.extend(TypeNameViolations(root, paths))
    violations.extend(ClassMemberNameViolations(root, paths))
    violations.extend(StructMemberNameViolations(root, paths))
    violations.extend(GlobalNameViolations(root, paths))
    violations.extend(PreprocessorDirectiveViolations(root, paths))
    violations.extend(CMakeCompileDefinitionViolations(root))
    violations.extend(ControlBraceViolations(root, paths))
    violations.extend(FunctionBraceViolations(root, paths))
    violations.extend(LongParameterFunctionViolations(root, paths))
    violations.extend(FunctionLineCountViolations(root, paths))
    violations.extend(HeaderInlineFunctionViolations(root, paths))
    violations.extend(CommentResidueViolations(root, paths))
    violations.extend(BarePrintfViolations(root, paths))
    violations.extend(NativeStdoutViolations(root, paths))
    violations.extend(NativeStderrWriteViolations(root, paths))
    if violations:
        sys.stderr.write("\n".join(violations) + "\n")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
