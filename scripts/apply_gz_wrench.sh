#!/usr/bin/env bash
set -euo pipefail

Usage()
{
    cat <<'EOF'
Usage: scripts/apply_gz_wrench.sh [options]

Apply a horizontal persistent Gazebo wrench, then clear it after a fixed
simulation-time duration.

Options:
  --world NAME          Gazebo world name (default: smartdrone_hover)
  --model NAME          Target model name (default: smartdrone_x500)
  --force-n NEWTONS     World-frame X force (default: 4.0)
  --duration-ms MS      Force duration in simulation milliseconds (default: 250)
  -h, --help            Show this help
EOF
}

Fail()
{
    echo "error: $*" >&2
    exit 2
}

WORLD_NAME="smartdrone_hover"
MODEL_NAME="smartdrone_x500"
FORCE_N="4.0"
DURATION_MS="250"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --world) [[ $# -ge 2 ]] || Fail "--world requires a value"; WORLD_NAME="$2"; shift 2 ;;
        --model) [[ $# -ge 2 ]] || Fail "--model requires a value"; MODEL_NAME="$2"; shift 2 ;;
        --force-n) [[ $# -ge 2 ]] || Fail "--force-n requires a value"; FORCE_N="$2"; shift 2 ;;
        --duration-ms) [[ $# -ge 2 ]] || Fail "--duration-ms requires a value"; DURATION_MS="$2"; shift 2 ;;
        -h|--help) Usage; exit 0 ;;
        *) Fail "unknown option: $1" ;;
    esac
done

[[ "$WORLD_NAME" =~ ^[A-Za-z0-9_.-]+$ ]] || Fail "invalid world name"
[[ "$MODEL_NAME" =~ ^[A-Za-z0-9_.:-]+$ ]] || Fail "invalid model name"
[[ "$FORCE_N" =~ ^-?([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || Fail "force must be numeric"
[[ "$DURATION_MS" =~ ^[0-9]+$ ]] || Fail "duration must be an integer"
awk -v force="$FORCE_N" 'BEGIN { exit !(force != 0 && force >= -100 && force <= 100) }' || \
    Fail "force magnitude must be in (0, 100] N"
DURATION_MS=$((10#$DURATION_MS))
((DURATION_MS >= 1 && DURATION_MS <= 5000)) || Fail "duration must be in [1, 5000] ms"
command -v gz >/dev/null 2>&1 || Fail "gz command not found"
command -v python3 >/dev/null 2>&1 || Fail "python3 command not found"
command -v timeout >/dev/null 2>&1 || Fail "timeout command not found"

PERSISTENT_TOPIC="/world/${WORLD_NAME}/wrench/persistent"
CLEAR_TOPIC="/world/${WORLD_NAME}/wrench/clear"
CLOCK_TOPIC="/world/${WORLD_NAME}/clock"
ENTITY="name: \"${MODEL_NAME}\" type: MODEL"
WRENCH="entity { ${ENTITY} } wrench { force { x: ${FORCE_N} y: 0 z: 0 } torque { x: 0 y: 0 z: 0 } }"
CLOCK_READY_TIMEOUT_S="${SMART_DRONE_GZ_CLOCK_READY_TIMEOUT_S:-5}"
CLOCK_WALL_TIMEOUT_S="${SMART_DRONE_GZ_WRENCH_TIMEOUT_S:-}"
GZ_COMMAND_TIMEOUT_S="${SMART_DRONE_GZ_COMMAND_TIMEOUT_S:-2}"
if [[ -z "$CLOCK_WALL_TIMEOUT_S" ]]; then
    CLOCK_WALL_TIMEOUT_S="$(awk -v duration_ms="$DURATION_MS" \
        'BEGIN { value = duration_ms / 50.0 + 10.0; print value < 30.0 ? 30.0 : value }')"
fi
awk -v value="$CLOCK_READY_TIMEOUT_S" 'BEGIN { exit !(value > 0) }' || \
    Fail "SMART_DRONE_GZ_CLOCK_READY_TIMEOUT_S must be positive"
awk -v value="$CLOCK_WALL_TIMEOUT_S" 'BEGIN { exit !(value > 0) }' || \
    Fail "SMART_DRONE_GZ_WRENCH_TIMEOUT_S must be positive"
awk -v value="$GZ_COMMAND_TIMEOUT_S" 'BEGIN { exit !(value > 0) }' || \
    Fail "SMART_DRONE_GZ_COMMAND_TIMEOUT_S must be positive"
CLEANUP_REQUIRED=0

ClearWrench()
{
    timeout --signal=TERM --kill-after=1 "$GZ_COMMAND_TIMEOUT_S" \
        gz topic -t "$CLEAR_TOPIC" -m gz.msgs.Entity -p "$ENTITY"
}

Cleanup()
{
    local status=$?
    trap - EXIT INT TERM
    if ((CLEANUP_REQUIRED)); then
        ClearWrench || true
    fi
    exit "$status"
}

ApplyWrenchForSimulationDuration()
{
    python3 - "$CLOCK_TOPIC" "$PERSISTENT_TOPIC" "$CLEAR_TOPIC" \
        "$WRENCH" "$ENTITY" "$DURATION_MS" "$CLOCK_READY_TIMEOUT_S" \
        "$CLOCK_WALL_TIMEOUT_S" "$GZ_COMMAND_TIMEOUT_S" <<'PY'
import json
import os
import selectors
import subprocess
import sys
import time


clock_topic = sys.argv[1]
persistent_topic = sys.argv[2]
clear_topic = sys.argv[3]
wrench = sys.argv[4]
entity = sys.argv[5]
duration_ns = int(sys.argv[6]) * 1_000_000
ready_timeout_s = float(sys.argv[7])
wall_timeout_s = float(sys.argv[8])
command_timeout_s = sys.argv[9]
decoder = json.JSONDecoder()
clock_process = subprocess.Popen(
    ["gz", "topic", "-e", "--json-output", "-t", clock_topic],
    stdout=subprocess.PIPE,
    stderr=subprocess.DEVNULL,
)


def simulation_time_ns(record):
    if not isinstance(record, dict) or not isinstance(record.get("sim"), dict):
        return None
    sim = record["sim"]
    try:
        seconds = int(sim.get("sec", 0))
        nanoseconds = int(sim.get("nsec", 0))
    except (TypeError, ValueError):
        return None
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        return None
    return seconds * 1_000_000_000 + nanoseconds


def consume_records(buffer):
    records = []
    while buffer:
        buffer = buffer.lstrip()
        if not buffer:
            break
        start = buffer.find("{")
        if start < 0:
            return records, buffer[-4096:]
        buffer = buffer[start:]
        try:
            record, end = decoder.raw_decode(buffer)
        except json.JSONDecodeError:
            return records, buffer[-1_048_576:]
        records.append(record)
        buffer = buffer[end:]
    return records, buffer


def publish_topic(topic, message_type, payload):
    command = [
        "timeout", "--signal=TERM", "--kill-after=1", command_timeout_s,
        "gz", "topic", "-t", topic, "-m", message_type, "-p", payload,
    ]
    return subprocess.run(command, check=False).returncode


started_wall_s = time.monotonic()
ready_deadline_s = started_wall_s + ready_timeout_s
wall_deadline_s = None
start_sim_ns = None
last_sim_ns = None
wrench_published = False
wrench_may_be_active = False
buffer = ""
selector = selectors.DefaultSelector()
assert clock_process.stdout is not None
selector.register(clock_process.stdout, selectors.EVENT_READ)
status = 0
message = ""
try:
    while True:
        now_s = time.monotonic()
        if wrench_published:
            assert wall_deadline_s is not None
            deadline_s = wall_deadline_s
        else:
            deadline_s = ready_deadline_s
        if now_s >= deadline_s:
            status = 4 if wrench_published else 3
            message = (
                f"Gazebo simulation clock did not advance by {duration_ns // 1_000_000} ms before timeout"
                if wrench_published
                else f"Gazebo simulation clock is unavailable: {clock_topic}"
            )
            break
        events = selector.select(min(0.1, deadline_s - now_s))
        if not events:
            if clock_process.poll() is not None:
                status = 5 if wrench_published else 3
                message = f"Gazebo simulation clock reader exited early: {clock_topic}"
                break
            continue
        chunk = os.read(clock_process.stdout.fileno(), 65536)
        if not chunk:
            status = 5 if wrench_published else 3
            message = f"Gazebo simulation clock reader closed early: {clock_topic}"
            break
        buffer += chunk.decode("utf-8", errors="replace")
        records, buffer = consume_records(buffer)
        duration_reached = False
        batch_last_sim_ns = None
        establishing_start = wrench_published and start_sim_ns is None
        for record in records:
            current_sim_ns = simulation_time_ns(record)
            if current_sim_ns is None:
                continue
            if wrench_published and last_sim_ns is not None and current_sim_ns < last_sim_ns:
                status = 6
                message = f"Gazebo simulation clock reset while wrench was active: {clock_topic}"
                break
            last_sim_ns = current_sim_ns
            batch_last_sim_ns = current_sim_ns
            if not wrench_published:
                continue
            if establishing_start:
                continue
            if current_sim_ns - start_sim_ns >= duration_ns:
                duration_reached = True
                break
        if message:
            break
        if establishing_start and batch_last_sim_ns is not None:
            start_sim_ns = batch_last_sim_ns
        if not wrench_published and last_sim_ns is not None:
            wrench_may_be_active = True
            if publish_topic(persistent_topic, "gz.msgs.EntityWrench", wrench) != 0:
                status = 7
                message = f"failed to publish persistent wrench: {persistent_topic}"
                break
            wrench_published = True
            wall_deadline_s = time.monotonic() + wall_timeout_s
        if duration_reached:
            if publish_topic(clear_topic, "gz.msgs.Entity", entity) != 0:
                status = 8
                message = f"failed to clear persistent wrench: {clear_topic}"
            else:
                wrench_published = False
                wrench_may_be_active = False
            break
finally:
    if wrench_may_be_active:
        if publish_topic(clear_topic, "gz.msgs.Entity", entity) == 0:
            wrench_published = False
            wrench_may_be_active = False
    selector.close()
    if clock_process.poll() is None:
        clock_process.terminate()
        try:
            clock_process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            clock_process.kill()
            clock_process.wait(timeout=1.0)
    clock_process.stdout.close()

if message:
    print(f"error: {message}", file=sys.stderr)
raise SystemExit(status)
PY
}

trap Cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Clear stale persistent force from an interrupted earlier scenario.
ClearWrench
CLEANUP_REQUIRED=1
ApplyWrenchForSimulationDuration || \
    Fail "failed to apply wrench for the requested Gazebo simulation duration"
CLEANUP_REQUIRED=0
trap - EXIT INT TERM
