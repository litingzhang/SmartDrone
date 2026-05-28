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

NATIVE_RUNTIME_MAIN_FILE = "src/native/main.cpp"
RUNTIME_HOST_LIFECYCLE_FILE = "src/native/app/bootstrap/runtime_host.cpp"
RUNTIME_HOST_LIFECYCLE_FUNCTION = "RunSystemGraphUntilStopped"
RUNTIME_HOST_ALLOWED_LOOP = (
    "while (!SmartDrone::Common::RuntimeStopRequested())")
RUNTIME_HOST_BUSINESS_STEP_PATTERNS = {
    "PollRxOnce(": "vehicle telemetry polling must run in the system EPG graph",
    "StepSetpointStream(": "setpoint streaming must run in the system EPG graph",
    "StepManualControl(": "manual control must run in the system EPG graph",
    "StepForceRestart(": "restart checks must run in the system EPG graph",
    "OnSessionSupervisorGraphTick(":
        "session supervision must run in the system EPG graph",
    "StepEpgRedeploy(": "session redeploy must run in the system EPG graph",
    "StepReceive(": "UDP receive must run in the system EPG graph",
    "StepHeartbeat": "UDP heartbeat must run in the system EPG graph",
    "StepStateTx(": "UDP state TX must run in the system EPG graph",
    "StepPointCloudTx(": "UDP point cloud TX must run in the system EPG graph",
}
RUNTIME_HOST_ADAPTER_BOUNDARY_PATTERNS = {
    '#include "common/tlv/udp_server.h"':
        "runtime host must not depend on UDP wire helpers",
    '#include "core/application/runtime/payload_builders.h"':
        "runtime host must not build UDP command payloads",
    "RuntimeGateSnapshot":
        "runtime host must use live-pose runtime adapter helpers",
    "UdpRuntimeStateSnapshot":
        "runtime host must use live-pose runtime adapter helpers",
    "UdpPeerToIpString(":
        "runtime host must not format UDP peers",
    "ReadSnapshot(":
        "runtime host must not read LivePoseState snapshots directly",
    "UpdatePeer(":
        "runtime host must not write command peer state directly",
    "SetRuntimeMode(":
        "runtime host must not write runtime mode state directly",
    "SetVehicleFlightState(":
        "runtime host must not write PX4 flight state directly",
}

SYSTEM_RUNTIME_GRAPH_CONFIG_FILE = (
    "src/native/core/application/runtime/system_runtime_graph_service.h")
SYSTEM_RUNTIME_GRAPH_CONFIG_BOUNDARY_PATTERNS = {
    '#include "core/application/runtime/udp_command_runtime.h"':
        "system graph config must not depend on concrete UDP command runtime",
    "UdpCommandRuntime ":
        "system graph config must not expose concrete UDP command runtime",
    "std::shared_ptr<UdpCommandRuntime>":
        "system graph config must not expose concrete UDP command runtime",
    "BuildCapabilitiesPayloadFn":
        "system graph config must accept provider metadata, not UDP payload callbacks",
    "BuildConfigPayloadFn":
        "system graph config must accept provider metadata, not UDP payload callbacks",
    "PeerToIpStringFn":
        "system graph config must not expose UDP peer formatting callbacks",
}

RUNTIME_CONFIG_FRAME_CODEC_FILES = (
    "src/native/core/application/runtime/runtime_config_frame_codec.h",
    "src/native/core/application/runtime/runtime_config_frame_codec.cpp",
)
RUNTIME_CONFIG_FRAME_CODEC_BOUNDARY_PATTERNS = {
    '#include "common/tlv/udp_server.h"':
        "runtime config frame codec must not depend on UDP peer types",
    '#include "core/application/runtime/udp_command_runtime.h"':
        "runtime config frame codec must not depend on UDP command runtime",
    '#include "core/application/runtime/runtime_command_service.h"':
        "runtime config frame codec must not depend on command services",
    '#include "core/application/config/config_registry.h"':
        "runtime config frame codec must not own config key-to-update mapping",
    '#include "core/application/runtime/runtime_config_update.h"':
        "runtime config frame codec must not expose command config updates",
    "UdpPeer":
        "runtime config frame codec must receive peer data as config values",
    "PeerToIpStringFn":
        "runtime config frame codec must not depend on UDP peer formatting callbacks",
    "RuntimeCommandService":
        "runtime config frame codec must not depend on command dispatch",
    "CommandResult":
        "runtime config frame codec must not depend on command results",
    "ConfigRegistry::":
        "runtime config frame codec must not own config key-to-update mapping",
    "ConfigUpdate":
        "runtime config frame codec must not expose command config updates",
    "BuildRuntimeConfigUpdate(":
        "runtime config frame codec must not own config key-to-update mapping",
    "ApplyPeerIp(":
        "runtime config frame codec must not combine peer formatting with config parsing",
}

RUNTIME_CONFIG_SERVICE_FILES = (
    "src/native/core/application/runtime/runtime_config_service.h",
    "src/native/core/application/runtime/runtime_config_service.cpp",
)
RUNTIME_CONFIG_SERVICE_BOUNDARY_PATTERNS = {
    '#include "core/application/runtime/runtime_command_service.h"':
        "runtime config service must not depend on command dispatch",
    '#include "core/application/config/config_registry.h"':
        "runtime config service must not own config key-to-field mapping",
    '#include "core/application/runtime/runtime_config_value_assignments.h"':
        "runtime config service must not assign individual config values",
    '#include "core/application/session/slam/slam_settings_loader.h"':
        "runtime config service must not apply calibrated runtime settings directly",
    "RuntimeCommandService":
        "runtime config service must not depend on command dispatch",
    "IRuntimeCommandTarget":
        "runtime config service must not depend on command targets",
    "RuntimeAction":
        "runtime config service must not depend on runtime actions",
    "ConfigRegistry::":
        "runtime config service must not own config key-to-field mapping",
    "AssignStrictInt(":
        "runtime config service must not assign individual config values",
    "AssignNumericInt(":
        "runtime config service must not assign individual config values",
    "AssignFloat(":
        "runtime config service must not assign individual config values",
    "AssignBool(":
        "runtime config service must not assign individual config values",
    "AssignString(":
        "runtime config service must not assign individual config values",
    "LoadStereoBodyExtrinsics(":
        "runtime config service must not apply calibrated runtime settings directly",
    "ResolveSettingsForSlamBackend(":
        "runtime config service must not resolve backend settings directly",
    "RuntimeRestartNeeded(":
        "runtime config service must not own restart diff details",
    "currentConfig.app.camera":
        "runtime config service must not project current config fields directly",
    "currentConfig.app.runtime":
        "runtime config service must not project current config fields directly",
    "currentConfig.app.udp":
        "runtime config service must not project current config fields directly",
}

SYSTEM_RUNTIME_TASK_BOUNDARY_FILES = (
    "src/native/core/application/runtime/system_runtime_tasks.h",
    "src/native/core/application/runtime/system_runtime_task_factory.h",
)
SYSTEM_RUNTIME_TASK_BOUNDARY_PATTERNS = {
    '#include "core/application/runtime/udp_command_runtime.h"':
        "system runtime tasks must depend on UDP phase interface",
    "std::shared_ptr<UdpCommandRuntime>":
        "system runtime tasks must depend on UDP phase interface",
    "UdpCommandRuntime *":
        "system runtime tasks must depend on UDP phase interface",
}

UDP_COMMAND_RUNTIME_HEADER_FILE = (
    "src/native/core/application/runtime/udp_command_runtime.h")
UDP_COMMAND_RUNTIME_HEADER_BOUNDARY_PATTERNS = {
    '#include "common/tlv/udp_server.h"':
        "UDP command runtime class header must not expose UDP wire helpers",
    '#include "common/tlv/runtime_command_hooks.h"':
        "UDP command runtime class header must not expose command hook details",
    "struct UdpCommandRuntimeConfig":
        "UDP command runtime config must live in its own header",
    "struct UdpRuntimeStateSnapshot":
        "UDP runtime state snapshot must live in config/value header",
    "BuildCapabilitiesPayloadFn":
        "UDP command runtime callbacks must live in config/value header",
    "PeerToIpStringFn":
        "UDP command runtime callbacks must live in config/value header",
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


def native_runtime_entrypoint_violations(root: Path) -> list[str]:
    violations: list[str] = []
    main_files: list[str] = []
    for path in source_files(root, "src/native"):
        text = path.read_text(encoding="utf-8", errors="ignore")
        if " main(" not in text and " main (" not in text:
            continue
        main_files.append(relative_path(path, root))
    if main_files != [NATIVE_RUNTIME_MAIN_FILE]:
        violations.append(
            "native runtime must have exactly one production main entrypoint: " +
            ", ".join(main_files))
        return violations

    text = (root / NATIVE_RUNTIME_MAIN_FILE).read_text(
        encoding="utf-8", errors="ignore")
    if "RuntimeHost runtimeHost" not in text or "runtimeHost.Run(" not in text:
        violations.append(
            "native runtime main must delegate execution to RuntimeHost/EPG")
    return violations


def extract_function_body(text: str, function_name: str) -> str:
    name_index = text.find(function_name)
    if name_index < 0:
        return ""
    brace_index = text.find("{", name_index)
    if brace_index < 0:
        return ""
    depth = 0
    for index in range(brace_index, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[brace_index:index + 1]
    return ""


def runtime_host_lifecycle_loop_violations(root: Path) -> list[str]:
    path = root / RUNTIME_HOST_LIFECYCLE_FILE
    if not path.exists():
        return [f"runtime host lifecycle file missing: {RUNTIME_HOST_LIFECYCLE_FILE}"]
    text = path.read_text(encoding="utf-8", errors="ignore")
    body = extract_function_body(text, RUNTIME_HOST_LIFECYCLE_FUNCTION)
    if not body:
        return [
            "runtime host lifecycle loop must be isolated in " +
            RUNTIME_HOST_LIFECYCLE_FUNCTION]
    loop_count = body.count("while (")
    if loop_count != 1 or RUNTIME_HOST_ALLOWED_LOOP not in body:
        return [
            "runtime host may only keep the stop/redeploy lifecycle wait loop"]
    violations: list[str] = []
    for pattern, message in RUNTIME_HOST_BUSINESS_STEP_PATTERNS.items():
        if pattern in body:
            violations.append(f"{message}: {RUNTIME_HOST_LIFECYCLE_FILE}")
    return violations


def runtime_host_adapter_boundary_violations(root: Path) -> list[str]:
    path = root / RUNTIME_HOST_LIFECYCLE_FILE
    if not path.exists():
        return [f"runtime host file missing: {RUNTIME_HOST_LIFECYCLE_FILE}"]
    text = path.read_text(encoding="utf-8", errors="ignore")
    violations: list[str] = []
    for pattern, message in RUNTIME_HOST_ADAPTER_BOUNDARY_PATTERNS.items():
        index = text.find(pattern)
        if index >= 0:
            line = text.count("\n", 0, index) + 1
            violations.append(f"{message}: {RUNTIME_HOST_LIFECYCLE_FILE}:{line}")
    return violations


def system_runtime_graph_config_boundary_violations(root: Path) -> list[str]:
    path = root / SYSTEM_RUNTIME_GRAPH_CONFIG_FILE
    if not path.exists():
        return [f"system runtime graph config file missing: {SYSTEM_RUNTIME_GRAPH_CONFIG_FILE}"]
    text = path.read_text(encoding="utf-8", errors="ignore")
    violations: list[str] = []
    for pattern, message in SYSTEM_RUNTIME_GRAPH_CONFIG_BOUNDARY_PATTERNS.items():
        index = text.find(pattern)
        if index >= 0:
            line = text.count("\n", 0, index) + 1
            violations.append(f"{message}: {SYSTEM_RUNTIME_GRAPH_CONFIG_FILE}:{line}")
    return violations


def runtime_config_frame_codec_boundary_violations(root: Path) -> list[str]:
    violations: list[str] = []
    for rel in RUNTIME_CONFIG_FRAME_CODEC_FILES:
        path = root / rel
        if not path.exists():
            violations.append(f"runtime config frame codec file missing: {rel}")
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern, message in RUNTIME_CONFIG_FRAME_CODEC_BOUNDARY_PATTERNS.items():
            index = text.find(pattern)
            if index >= 0:
                line = text.count("\n", 0, index) + 1
                violations.append(f"{message}: {rel}:{line}")
    return violations


def runtime_config_service_boundary_violations(root: Path) -> list[str]:
    violations: list[str] = []
    for rel in RUNTIME_CONFIG_SERVICE_FILES:
        path = root / rel
        if not path.exists():
            violations.append(f"runtime config service file missing: {rel}")
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern, message in RUNTIME_CONFIG_SERVICE_BOUNDARY_PATTERNS.items():
            index = text.find(pattern)
            if index >= 0:
                line = text.count("\n", 0, index) + 1
                violations.append(f"{message}: {rel}:{line}")
    return violations


def system_runtime_task_boundary_violations(root: Path) -> list[str]:
    violations: list[str] = []
    for rel in SYSTEM_RUNTIME_TASK_BOUNDARY_FILES:
        path = root / rel
        if not path.exists():
            violations.append(f"system runtime task boundary file missing: {rel}")
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern, message in SYSTEM_RUNTIME_TASK_BOUNDARY_PATTERNS.items():
            index = text.find(pattern)
            if index >= 0:
                line = text.count("\n", 0, index) + 1
                violations.append(f"{message}: {rel}:{line}")
    return violations


def udp_command_runtime_header_boundary_violations(root: Path) -> list[str]:
    path = root / UDP_COMMAND_RUNTIME_HEADER_FILE
    if not path.exists():
        return [f"UDP command runtime header missing: {UDP_COMMAND_RUNTIME_HEADER_FILE}"]
    text = path.read_text(encoding="utf-8", errors="ignore")
    violations: list[str] = []
    for pattern, message in UDP_COMMAND_RUNTIME_HEADER_BOUNDARY_PATTERNS.items():
        index = text.find(pattern)
        if index >= 0:
            line = text.count("\n", 0, index) + 1
            violations.append(f"{message}: {UDP_COMMAND_RUNTIME_HEADER_FILE}:{line}")
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
    violations.extend(native_runtime_entrypoint_violations(root))
    violations.extend(runtime_host_lifecycle_loop_violations(root))
    violations.extend(runtime_host_adapter_boundary_violations(root))
    violations.extend(system_runtime_graph_config_boundary_violations(root))
    violations.extend(runtime_config_frame_codec_boundary_violations(root))
    violations.extend(runtime_config_service_boundary_violations(root))
    violations.extend(system_runtime_task_boundary_violations(root))
    violations.extend(udp_command_runtime_header_boundary_violations(root))
    if violations:
        sys.stderr.write("\n".join(violations) + "\n")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
